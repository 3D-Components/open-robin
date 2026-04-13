#!/usr/bin/env python3
"""
ROBIN Telemetry Aggregator Node.

Combines geometry and welder data from multiple ROS2 topics into a single
``/robin/telemetry`` topic (``ProcessTelemetry``) at a configurable minimum
publish rate.

Unlike the earlier "latest value wins" behavior, this node now aligns geometry
and Fronius samples by bead progression:

* ``/robin/data/fronius`` carries ``bead_id`` + ``progression`` directly.
* ``/robin/data/progression`` provides the progression timeline for geometry.
* geometry samples are assigned the nearest progression sample by timestamp.
* the published telemetry snapshot is only updated when a geometry sample and a
  Fronius sample match on bead/progression within a configurable tolerance.

The output topic is read by Orion-LD's DDS bridge (``config-dds.json``
allowlist: ``rt/robin/telemetry``) and stored in TimescaleDB via TROE for
temporal queries.
"""

from collections import deque
import struct

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.serialization import deserialize_message

from robin_core_data.telemetry_alignment import (
    AlignedTelemetrySample,
    clamp_progression,
    find_nearest_progression,
    FroniusTelemetrySample,
    GeometrySample,
    match_fronius_to_geometry,
    match_geometry_to_fronius,
    PendingGeometrySample,
    ProgressionSample,
    seconds_to_ns,
    time_msg_to_ns,
)
from robin_interfaces.msg import (
    BeadGeometry,
    FroniusSample,
    ProcessTelemetry,
    WelderData,
    WeldProgression,
)


def _parse_geometry_cdr(data: bytes) -> tuple[int, float, float, float]:
    """
    Parse BeadGeometry CDR bytes into ``(stamp_ns, height_mm, width_mm, csa_mm2)``.

    Handles two on-wire layouts:
      New (no bead_id):  Header + float32 height_mm + float32 width_mm + float32 csa
      Old (with bead_id): Header + string bead_id + float32 height_mm + …

    CDR is little-endian (the standard ROS 2 DDS serialisation).
    """
    try:
        off = 4  # skip CDR encapsulation header (4 bytes)

        # Header.stamp  (sec int32 + nanosec uint32 = 8 bytes)
        sec, nanosec = struct.unpack_from('<iI', data, off)
        stamp_ns = sec * 1_000_000_000 + nanosec
        off += 8

        # Header.frame_id  (4-byte length prefix + N bytes)
        (fid_len,) = struct.unpack_from('<I', data, off)
        off += 4 + fid_len
        off = (off + 3) & ~3  # align to 4 bytes

        remaining = len(data) - off

        if remaining == 12:
            # New BeadGeometry: directly height_mm, width_mm, csa_mm2
            h, w, a = struct.unpack_from('<fff', data, off)
            return stamp_ns, h, w, a

        if remaining >= 16:
            # Old BeadGeometry: string bead_id precedes the float32 fields
            (bid_len,) = struct.unpack_from('<I', data, off)
            if bid_len < 256:  # sanity-check: bead IDs are short strings
                off += 4 + bid_len
                off = (off + 3) & ~3
                h, w, a = struct.unpack_from('<fff', data, off)
                return stamp_ns, h, w, a

    except Exception:
        pass

    return 0, 0.0, 0.0, 0.0


class TelemetryAggregatorNode(Node):
    """Aggregates geometry and welder telemetry into a single aligned output topic."""

    def __init__(self):
        super().__init__('telemetry_aggregator')

        # Parameters
        self.declare_parameter('geometry_topic', '/robin/weld_dimensions')
        self.declare_parameter('progression_topic', '/robin/data/progression')
        self.declare_parameter('fronius_topic', '/robin/data/fronius')
        self.declare_parameter('output_topic', '/robin/telemetry')
        self.declare_parameter('min_publish_period', 1.0)
        self.declare_parameter('history_size', 500)
        self.declare_parameter('progression_lookup_tolerance_sec', 0.25)
        self.declare_parameter('alignment_progression_tolerance', 0.03)
        self.declare_parameter('alignment_time_tolerance_sec', 30.0)

        geometry_topic = self.get_parameter('geometry_topic').value
        progression_topic = self.get_parameter('progression_topic').value
        fronius_topic = self.get_parameter('fronius_topic').value
        output_topic = self.get_parameter('output_topic').value
        min_publish_period = self.get_parameter('min_publish_period').value
        history_size = int(self.get_parameter('history_size').value)
        progression_lookup_tolerance_sec = float(
            self.get_parameter('progression_lookup_tolerance_sec').value
        )
        alignment_progression_tolerance = float(
            self.get_parameter('alignment_progression_tolerance').value
        )
        alignment_time_tolerance_sec = float(
            self.get_parameter('alignment_time_tolerance_sec').value
        )

        self._callback_group = ReentrantCallbackGroup()
        self._history_size = max(history_size, 10)
        self._progression_lookup_tolerance_ns = seconds_to_ns(
            progression_lookup_tolerance_sec
        )
        self._alignment_progression_tolerance = max(
            0.0, alignment_progression_tolerance
        )
        self._alignment_time_tolerance_ns = seconds_to_ns(
            alignment_time_tolerance_sec
        )
        self._publish_count = 0
        self._last_source_stamp_ns = None
        self._latest_aligned_sample = None

        self._progression_history = deque(maxlen=self._history_size)
        self._pending_geometry_history = deque(maxlen=self._history_size)
        self._geometry_history = deque(maxlen=self._history_size)
        self._fronius_history = deque(maxlen=self._history_size)

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
        self._progression_sub = self.create_subscription(
            WeldProgression,
            progression_topic,
            self._progression_callback,
            10,
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
            f' | progression: {progression_topic}'
            f' | fronius: {fronius_topic}'
            f' | output: {output_topic}'
            f' | period: {min_publish_period}s'
            f' | progression tolerance: {self._alignment_progression_tolerance:.3f}'
        )

    # -------------------------------------------------------------------------
    # Alignment helpers
    # -------------------------------------------------------------------------

    def _note_source_stamp(self, stamp_ns: int) -> None:
        if stamp_ns <= 0:
            return

        if (
            self._last_source_stamp_ns is not None
            and stamp_ns + seconds_to_ns(1.0) < self._last_source_stamp_ns
        ):
            self.get_logger().info(
                'Source timestamps moved backwards; resetting alignment history'
            )
            self._reset_alignment_history()

        self._last_source_stamp_ns = stamp_ns

    def _reset_alignment_history(self) -> None:
        self._progression_history.clear()
        self._pending_geometry_history.clear()
        self._geometry_history.clear()
        self._fronius_history.clear()
        self._latest_aligned_sample = None
        self._last_source_stamp_ns = None

    def _record_aligned_sample(
        self,
        fronius_sample: FroniusTelemetrySample,
        geometry_sample: GeometrySample,
    ) -> None:
        self._latest_aligned_sample = AlignedTelemetrySample(
            bead_id=geometry_sample.bead_id or fronius_sample.bead_id,
            progression=geometry_sample.progression,
            height=geometry_sample.height,
            width=geometry_sample.width,
            cross_sectional_area=geometry_sample.cross_sectional_area,
            current=fronius_sample.current,
            voltage=fronius_sample.voltage,
            speed=fronius_sample.speed,
            geometry_stamp_ns=geometry_sample.stamp_ns,
            fronius_stamp_ns=fronius_sample.stamp_ns,
        )

    def _resolve_pending_geometry(self) -> None:
        remaining_geometry = []

        for pending_geometry in self._pending_geometry_history:
            progression_sample = find_nearest_progression(
                self._progression_history,
                pending_geometry.stamp_ns,
                self._progression_lookup_tolerance_ns,
            )
            if progression_sample is None:
                remaining_geometry.append(pending_geometry)
                continue

            geometry_sample = GeometrySample(
                stamp_ns=pending_geometry.stamp_ns,
                bead_id=progression_sample.bead_id,
                progression=progression_sample.progression,
                height=pending_geometry.height,
                width=pending_geometry.width,
                cross_sectional_area=pending_geometry.cross_sectional_area,
            )
            self._geometry_history.append(geometry_sample)

            fronius_match = match_fronius_to_geometry(
                geometry_sample,
                self._fronius_history,
                max_progression_delta=self._alignment_progression_tolerance,
                max_time_delta_ns=self._alignment_time_tolerance_ns,
            )
            if fronius_match is not None:
                self._record_aligned_sample(fronius_match, geometry_sample)

        self._pending_geometry_history = deque(
            remaining_geometry,
            maxlen=self._history_size,
        )

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _progression_callback(self, msg: WeldProgression) -> None:
        stamp_ns = time_msg_to_ns(msg.header.stamp)
        self._note_source_stamp(stamp_ns)
        self._progression_history.append(
            ProgressionSample(
                stamp_ns=stamp_ns,
                bead_id=msg.bead_id,
                progression=clamp_progression(msg.progression),
            )
        )
        self._resolve_pending_geometry()

    def _fronius_raw_callback(self, msg) -> None:
        """Deserialize fronius bytes as FroniusSample and match by progression."""
        try:
            sample = deserialize_message(bytes(msg), FroniusSample)
        except Exception as exc:
            self.get_logger().debug(f'Fronius deserialise error: {exc}')
            return

        stamp_ns = time_msg_to_ns(sample.header.stamp)
        self._note_source_stamp(stamp_ns)

        fronius_sample = FroniusTelemetrySample(
            stamp_ns=stamp_ns,
            bead_id=sample.bead_id,
            progression=clamp_progression(sample.progression),
            current=sample.current,
            voltage=sample.voltage,
            speed=sample.wire_feed_speed,
        )
        self._fronius_history.append(fronius_sample)

        geometry_match = match_geometry_to_fronius(
            fronius_sample,
            self._geometry_history,
            max_progression_delta=self._alignment_progression_tolerance,
            max_time_delta_ns=self._alignment_time_tolerance_ns,
        )
        if geometry_match is not None:
            self._record_aligned_sample(fronius_sample, geometry_match)

    def _geometry_raw_callback(self, msg) -> None:
        """Parse geometry CDR bytes, then assign progression from the progression stream."""
        stamp_ns, height_mm, width_mm, csa_mm2 = _parse_geometry_cdr(bytes(msg))
        self._note_source_stamp(stamp_ns)
        self._pending_geometry_history.append(
            PendingGeometrySample(
                stamp_ns=stamp_ns,
                height=height_mm,
                width=width_mm,
                cross_sectional_area=csa_mm2,
            )
        )
        self._resolve_pending_geometry()

    # -------------------------------------------------------------------------
    # Publishing
    # -------------------------------------------------------------------------

    def _publish_telemetry(self) -> None:
        if self._latest_aligned_sample is None:
            return

        now = self.get_clock().now()
        aligned_sample = self._latest_aligned_sample
        out = ProcessTelemetry()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'base_link'
        out.bead_id = aligned_sample.bead_id
        out.progression = aligned_sample.progression

        out.current = aligned_sample.current
        out.voltage = aligned_sample.voltage
        out.speed = aligned_sample.speed

        out.width = aligned_sample.width
        out.height = aligned_sample.height
        out.cross_sectional_area = aligned_sample.cross_sectional_area

        self._telemetry_pub.publish(out)
        self._publish_count += 1
        self.get_logger().info(
            f'[AGG] Published telemetry #{self._publish_count}'
            f' bead={out.bead_id or "-"} progression={out.progression:.3f}'
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
