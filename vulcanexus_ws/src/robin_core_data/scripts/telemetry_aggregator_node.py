#!/usr/bin/env python3
"""
ROBIN Telemetry Aggregator Node

Combines geometry and welder data from multiple ROS2 topics into a single
/robin/telemetry topic (ProcessTelemetry) at a configurable minimum publish rate.

The output topic is read by Orion-LD's DDS bridge (config-dds.json allowlist:
rt/robin/telemetry) and stored in TimescaleDB via TROE for temporal queries.

Subscribes to:
- geometry_topic  (default /robin/weld_dimensions)  — BeadGeometry
- fronius_topic   (default /robin/data/fronius)     — FroniusSample / WelderData

Publishes:
- output_topic    (default /robin/telemetry)         — ProcessTelemetry

Type-compatibility notes
------------------------
Bags recorded before the WelderData refactor carry fronius data with the
FroniusSample CDR layout (bead_id, progression, current, voltage,
wire_feed_speed, power — all float32) under the type name
"robin_interfaces/msg/WelderData".  The aggregator handles both versions:

 * It uses create_subscription(..., raw=True) for both topics so that the
   callback receives raw CDR bytes instead of a deserialized object, bypassing
   the ROS 2 type-name check.  Fronius bytes are then deserialized with the
   FroniusSample class, which is binary-compatible with the old WelderData CDR
   layout.

 * BeadGeometry in old bags includes an extra leading "string bead_id" field.
   The raw-bytes geometry callback detects the format from the remaining byte
   count after the Header and reads the float32 fields accordingly.
"""

import struct

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.serialization import deserialize_message

from robin_interfaces.msg import BeadGeometry, FroniusSample, ProcessTelemetry, WelderData


def _parse_geometry_cdr(data: bytes) -> tuple:
    """Parse BeadGeometry CDR bytes into (height_mm, width_mm, csa_mm2).

    Handles two on-wire layouts:
      New (no bead_id):  Header + float32 height_mm + float32 width_mm + float32 csa
      Old (with bead_id): Header + string bead_id + float32 height_mm + …

    CDR is little-endian (the standard ROS 2 DDS serialisation).
    """
    try:
        off = 4  # skip CDR encapsulation header (4 bytes)

        # Header.stamp  (sec int32 + nanosec uint32 = 8 bytes)
        off += 8

        # Header.frame_id  (4-byte length prefix + N bytes)
        (fid_len,) = struct.unpack_from('<I', data, off)
        off += 4 + fid_len
        off = (off + 3) & ~3  # align to 4 bytes

        remaining = len(data) - off

        if 12 <= remaining < 16:
            # New BeadGeometry: directly height_mm, width_mm, csa_mm2
            # Accept 12–15 bytes to tolerate trailing CDR alignment padding.
            h, w, a = struct.unpack_from('<fff', data, off)
            return h, w, a

        if remaining >= 16:
            # Old BeadGeometry: string bead_id precedes the float32 fields
            (bid_len,) = struct.unpack_from('<I', data, off)
            if bid_len < 256:  # sanity-check: bead IDs are short strings
                off += 4 + bid_len
                off = (off + 3) & ~3
                h, w, a = struct.unpack_from('<fff', data, off)
                return h, w, a

    except Exception as exc:
        import sys
        print(f'[AGG] CDR geometry parse error (len={len(data)}, remaining={remaining if "remaining" in dir() else "?"} ): {exc}', file=sys.stderr)

    return 0.0, 0.0, 0.0


class TelemetryAggregatorNode(Node):
    """Aggregates geometry and welder telemetry into a single output topic."""

    def __init__(self):
        super().__init__('telemetry_aggregator')

        # Parameters
        self.declare_parameter('geometry_topic', '/robin/weld_dimensions')
        self.declare_parameter('fronius_topic', '/robin/data/fronius')
        self.declare_parameter('output_topic', '/robin/telemetry')
        self.declare_parameter('min_publish_period', 1.0)

        geometry_topic = self.get_parameter('geometry_topic').value
        fronius_topic = self.get_parameter('fronius_topic').value
        output_topic = self.get_parameter('output_topic').value
        min_publish_period = self.get_parameter('min_publish_period').value

        self._callback_group = ReentrantCallbackGroup()

        # Latest values from geometry topic
        self._width_mm: float = 0.0
        self._height_mm: float = 0.0
        self._cross_sectional_area_mm2: float = 0.0

        # Latest values from fronius topic
        self._current: float = 0.0
        self._voltage: float = 0.0
        self._wire_feed_speed: float = 0.0

        self._publish_count = 0

        # raw=True subscriptions receive bytes instead of a deserialized object,
        # bypassing the CDR type-name check.  This is needed because bags may
        # declare the fronius type as "WelderData" even though the bytes match
        # the old FroniusSample CDR layout.  BeadGeometry bags may carry an
        # extra leading "string bead_id" field not present in the current msg.
        self._geometry_sub = self.create_subscription(
            BeadGeometry,
            geometry_topic,
            self._geometry_raw_callback,
            10,
            raw=True,
            callback_group=self._callback_group,
        )
        self._fronius_sub = self.create_subscription(
            WelderData,
            fronius_topic,
            self._fronius_raw_callback,
            10,
            raw=True,
            callback_group=self._callback_group,
        )

        # Publisher
        self._telemetry_pub = self.create_publisher(ProcessTelemetry, output_topic, 10)

        # Timer fires at min_publish_period
        self._publish_timer = self.create_timer(
            min_publish_period,
            self._publish_telemetry,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            f'Telemetry aggregator started'
            f' | geometry: {geometry_topic}'
            f' | fronius: {fronius_topic}'
            f' | output: {output_topic}'
            f' | period: {min_publish_period}s'
        )

    # -------------------------------------------------------------------------
    # Raw-bytes callbacks
    # -------------------------------------------------------------------------

    def _fronius_raw_callback(self, msg) -> None:
        """Deserialise fronius bytes as FroniusSample (CDR-compatible with old WelderData)."""
        try:
            sample: FroniusSample = deserialize_message(bytes(msg), FroniusSample)
            self._current = sample.current
            self._voltage = sample.voltage
            self._wire_feed_speed = sample.wire_feed_speed
        except Exception as exc:
            self.get_logger().debug(f'Fronius deserialise error: {exc}')

    def _geometry_raw_callback(self, msg) -> None:
        """Parse geometry CDR bytes, handling old (bead_id) and new layouts."""
        h, w, a = _parse_geometry_cdr(bytes(msg))
        self._height_mm = h
        self._width_mm = w
        self._cross_sectional_area_mm2 = a

    # -------------------------------------------------------------------------
    # Publishing
    # -------------------------------------------------------------------------

    def _publish_telemetry(self) -> None:
        now = self.get_clock().now()

        out = ProcessTelemetry()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'base_link'

        out.current = self._current
        out.voltage = self._voltage
        out.speed = self._wire_feed_speed

        out.width = self._width_mm
        out.height = self._height_mm
        out.cross_sectional_area = self._cross_sectional_area_mm2

        self._telemetry_pub.publish(out)
        self._publish_count += 1
        self.get_logger().info(
            f'[AGG] Published telemetry #{self._publish_count}'
            f' current={out.current:.1f}A voltage={out.voltage:.1f}V'
            f' width={out.width:.2f}mm height={out.height:.2f}mm'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
