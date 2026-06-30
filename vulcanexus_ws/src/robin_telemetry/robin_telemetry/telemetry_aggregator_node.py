#!/usr/bin/env python3
"""
ROBIN Telemetry Aggregator Node

Combines geometry and welder data from recorded ROS bags into a single
``/robin/telemetry`` topic (``robin_interfaces/msg/ProcessTelemetry``) at a
configurable minimum publish rate. The output topic is read by Orion-LD's DDS
bridge (``config-dds.json`` allowlist: ``rt/robin/telemetry``) and stored in
TimescaleDB via TROE for temporal queries — i.e. it is what lights up the
dashboard when replaying real recorded welding data.

Subscribes to (both topics, configurable types per bag generation):
- geometry_topic (default /robin/weld_dimensions)
- fronius_topic  (default /robin/data/fronius)

Publishes:
- output_topic   (default /robin/telemetry) -- ProcessTelemetry

Bag-format compatibility
------------------------
Different recordings carry different message types on the same topics:

  newer bags (correct_process_params, bag_2026-03-16):
    fronius   -> robin_interfaces/msg/WelderData
    geometry  -> robin_interfaces/msg/BeadGeometry
  legacy bag (exp001_rosbag_real):
    fronius   -> robin_interfaces/msg/FroniusSample
    geometry  -> std_msgs/msg/Float32MultiArray

The subscription *type* must match the bag's recorded type for DDS to connect,
so it is selected with the ``fronius_type`` / ``geometry_type`` parameters (the
demo scripts pass the right value for their bag). All subscriptions use
``raw=True`` so the callback gets raw CDR bytes, bypassing the type-hash check
that would otherwise block replay across message revisions:

 * Fronius bytes are deserialized as FroniusSample, which is the shared leading
   layout of WelderData (WelderData = FroniusSample + a trailing ``energy``
   field), so one path reads both.
 * BeadGeometry bytes are struct-parsed, tolerating an old extra leading
   ``string bead_id`` field.
 * Float32MultiArray is deserialized and height/width are taken from configurable
   indices (``marray_height_index`` / ``marray_width_index``); the array and any
   dimension labels are logged once so the convention can be confirmed against a
   given bag.
"""

import struct

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray

from robin_interfaces.msg import (
    BeadGeometry,
    FroniusSample,
    ProcessTelemetry,
    WelderData,
)

# Parameter value -> message class for the per-bag subscription type.
_FRONIUS_TYPES = {'WelderData': WelderData, 'FroniusSample': FroniusSample}
_GEOMETRY_TYPES = {'BeadGeometry': BeadGeometry, 'Float32MultiArray': Float32MultiArray}


def _parse_geometry_cdr(data: bytes) -> tuple:
    """Parse BeadGeometry CDR bytes into (height_mm, width_mm, csa_mm2).

    Handles two on-wire layouts:
      New (no bead_id):   Header + float32 height + float32 width + float32 csa
      Old (with bead_id): Header + string bead_id + float32 height + ...

    CDR is little-endian (the standard ROS 2 DDS serialisation).
    """
    remaining = -1
    try:
        off = 4  # skip CDR encapsulation header (4 bytes)
        off += 8  # Header.stamp (sec int32 + nanosec uint32)

        # Header.frame_id (4-byte length prefix + N bytes), then align to 4
        (fid_len,) = struct.unpack_from('<I', data, off)
        off += 4 + fid_len
        off = (off + 3) & ~3

        remaining = len(data) - off

        if 12 <= remaining < 16:
            # New BeadGeometry: height, width, csa (tolerate trailing padding)
            return struct.unpack_from('<fff', data, off)

        if remaining >= 16:
            # Old BeadGeometry: string bead_id precedes the float32 fields
            (bid_len,) = struct.unpack_from('<I', data, off)
            if bid_len < 256:  # sanity-check: bead IDs are short strings
                off += 4 + bid_len
                off = (off + 3) & ~3
                return struct.unpack_from('<fff', data, off)
    except Exception:  # noqa: BLE001 - tolerate any malformed frame
        pass

    return 0.0, 0.0, 0.0


class TelemetryAggregatorNode(Node):
    """Aggregates geometry and welder telemetry into a single output topic."""

    def __init__(self):
        super().__init__('telemetry_aggregator')

        self.declare_parameter('geometry_topic', '/robin/weld_dimensions')
        self.declare_parameter('fronius_topic', '/robin/data/fronius')
        self.declare_parameter('output_topic', '/robin/telemetry')
        self.declare_parameter('min_publish_period', 1.0)
        # Per-bag message types (default: the current/newer bag generation).
        self.declare_parameter('fronius_type', 'WelderData')
        self.declare_parameter('geometry_type', 'BeadGeometry')
        # Float32MultiArray index mapping (exp001-style geometry).
        self.declare_parameter('marray_height_index', 0)
        self.declare_parameter('marray_width_index', 1)

        geometry_topic = self.get_parameter('geometry_topic').value
        fronius_topic = self.get_parameter('fronius_topic').value
        output_topic = self.get_parameter('output_topic').value
        min_publish_period = self.get_parameter('min_publish_period').value
        fronius_type = self.get_parameter('fronius_type').value
        geometry_type = self.get_parameter('geometry_type').value
        self._marray_h_idx = int(self.get_parameter('marray_height_index').value)
        self._marray_w_idx = int(self.get_parameter('marray_width_index').value)

        if fronius_type not in _FRONIUS_TYPES:
            self.get_logger().warning(
                f"Unknown fronius_type '{fronius_type}', falling back to WelderData"
            )
            fronius_type = 'WelderData'
        if geometry_type not in _GEOMETRY_TYPES:
            self.get_logger().warning(
                f"Unknown geometry_type '{geometry_type}', falling back to BeadGeometry"
            )
            geometry_type = 'BeadGeometry'

        self._callback_group = ReentrantCallbackGroup()

        # Latest values
        self._width_mm = 0.0
        self._height_mm = 0.0
        self._cross_sectional_area_mm2 = 0.0
        self._current = 0.0
        self._voltage = 0.0
        self._wire_feed_speed = 0.0
        self._publish_count = 0
        self._marray_logged = False

        # raw=True bypasses the CDR type-hash check (needed for cross-revision replay).
        self._fronius_sub = self.create_subscription(
            _FRONIUS_TYPES[fronius_type], fronius_topic, self._fronius_raw_callback,
            10, raw=True, callback_group=self._callback_group,
        )
        geometry_cb = (
            self._geometry_marray_callback
            if geometry_type == 'Float32MultiArray'
            else self._geometry_cdr_callback
        )
        self._geometry_sub = self.create_subscription(
            _GEOMETRY_TYPES[geometry_type], geometry_topic, geometry_cb,
            10, raw=True, callback_group=self._callback_group,
        )

        self._telemetry_pub = self.create_publisher(ProcessTelemetry, output_topic, 10)
        self._publish_timer = self.create_timer(
            min_publish_period, self._publish_telemetry,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            f'Telemetry aggregator started | geometry: {geometry_topic} ({geometry_type})'
            f' | fronius: {fronius_topic} ({fronius_type}) | output: {output_topic}'
            f' | period: {min_publish_period}s'
        )

    # -- Raw-bytes callbacks --------------------------------------------------

    def _fronius_raw_callback(self, msg) -> None:
        """Read current/voltage/wfs; FroniusSample is the shared WelderData prefix."""
        try:
            sample = deserialize_message(bytes(msg), FroniusSample)
            self._current = sample.current
            self._voltage = sample.voltage
            self._wire_feed_speed = sample.wire_feed_speed
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f'Fronius deserialise error: {exc}')

    def _geometry_cdr_callback(self, msg) -> None:
        """BeadGeometry path (old bead_id + new layouts)."""
        h, w, a = _parse_geometry_cdr(bytes(msg))
        self._height_mm = h
        self._width_mm = w
        self._cross_sectional_area_mm2 = a

    def _geometry_marray_callback(self, msg) -> None:
        """std_msgs/Float32MultiArray path (e.g. exp001_rosbag_real)."""
        try:
            arr = deserialize_message(bytes(msg), Float32MultiArray)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f'MultiArray deserialise error: {exc}')
            return
        data = list(arr.data)
        if not data:
            return
        if 0 <= self._marray_h_idx < len(data):
            self._height_mm = data[self._marray_h_idx]
        if 0 <= self._marray_w_idx < len(data):
            self._width_mm = data[self._marray_w_idx]
        if not self._marray_logged:
            labels = [d.label for d in arr.layout.dim]
            self.get_logger().info(
                f'[AGG] weld_dimensions Float32MultiArray len={len(data)} '
                f'labels={labels} sample={data[:6]} '
                f'(using height=data[{self._marray_h_idx}], width=data[{self._marray_w_idx}]; '
                f'override with -p marray_height_index / -p marray_width_index)'
            )
            self._marray_logged = True

    # -- Publishing -----------------------------------------------------------

    def _publish_telemetry(self) -> None:
        out = ProcessTelemetry()
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
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
