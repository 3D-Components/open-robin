#!/usr/bin/env python3
"""
ROBIN Progression Node — single source of truth for bead progression.

Tracks the TCP position along the active bead path and publishes a
normalised progression value (0.0–1.0).  All other nodes that need
progression (weld_data, process_data, …) subscribe to the output topic
rather than computing it themselves.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

import tf2_ros
from tf2_ros import TransformException

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
from robin_interfaces.msg import WeldProgression, ActiveBead

from robin_core.progression_utils import calculate_progression


class ProgressionNode(Node):
    """Publishes normalised bead progression from TCP position."""

    def __init__(self):
        super().__init__('progression_node')

        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tcp_frame', 'wire_tip')

        self._subscriptions = []
        self._lock = threading.Lock()

        # State
        self._active_tcp_frame = 'wire_tip'
        self._active_bead_id = ''
        self._bead_path = []
        self._total_length = 0.0
        self._is_welding = False
        self._is_scanning = False

        self.publish_timer = None
        self._timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._tf_buffer = None
        self._tf_listener = None
        self._tf_fail_count = 0
        self._tf_reset_failure_threshold = 20
        self._last_tf_reset_monotonic = 0.0

        # -- Configure --
        self.publish_rate = self.get_parameter('publish_rate').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tcp_frame = self.get_parameter('tcp_frame').value
        self._active_tcp_frame = self.tcp_frame
        self._tf_reset_failure_threshold = max(5, int(self.publish_rate * 2))

        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.pub_cb_group = MutuallyExclusiveCallbackGroup()

        self._reset_tf_listener()

        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self._subscriptions = [
            self.create_subscription(
                ActiveBead, 'robin/data/active_bead',
                self._active_bead_cb, 10,
                callback_group=self.sub_cb_group,
            ),
            self.create_subscription(
                Bool, 'robin/data/is_welding',
                self._welding_state_cb, latched_qos,
                callback_group=self.sub_cb_group,
            ),
            self.create_subscription(
                Bool, 'robin/data/is_scanning',
                self._scanning_state_cb, latched_qos,
                callback_group=self.sub_cb_group,
            ),
            self.create_subscription(
                String, 'tcp/active_frame',
                self._tcp_frame_cb, latched_qos,
                callback_group=self.sub_cb_group,
            ),
        ]

        self._progression_pub = self.create_publisher(
            WeldProgression, 'robin/data/progression', 10
        )

        self.get_logger().info(
            f'Configured: publish_rate={self.publish_rate}Hz'
        )

        # -- Activate --
        period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.1
        self.publish_timer = self.create_timer(
            period, self._publish_progression,
            callback_group=self.pub_cb_group,
            clock=self._timer_clock,
        )

        self.get_logger().info(
            f'Activated: publishing at {self.publish_rate}Hz'
        )

    # -- Subscription callbacks -----------------------------------------------

    def _active_bead_cb(self, msg: ActiveBead):
        with self._lock:
            if msg.bead_id != self._active_bead_id:
                self.get_logger().info(
                    f'Active bead updated: {msg.bead_id} '
                    f'(len={msg.total_length * 1000:.1f}mm, '
                    f'waypoints={len(msg.path)})'
                )
            self._active_bead_id = msg.bead_id
            self._bead_path = list(msg.path)
            self._total_length = msg.total_length

    def _welding_state_cb(self, msg: Bool):
        with self._lock:
            changed = msg.data != self._is_welding
            self._is_welding = msg.data
            path_snapshot = list(self._bead_path) if changed and msg.data else None

        if changed:
            state_str = 'ARC ON' if msg.data else 'ARC OFF'
            self.get_logger().info(f'Welding state changed -> {state_str}')

        if path_snapshot and len(path_snapshot) >= 1:
            tcp = self._get_tcp_position()
            p0 = path_snapshot[0]
            if tcp is not None:
                import math
                dist = math.sqrt(
                    (tcp.x - p0.x) ** 2
                    + (tcp.y - p0.y) ** 2
                    + (tcp.z - p0.z) ** 2
                )
                self.get_logger().info(
                    f'[ARC-ON] tcp=({tcp.x:.4f}, {tcp.y:.4f}, {tcp.z:.4f}) '
                    f'path[0]=({p0.x:.4f}, {p0.y:.4f}, {p0.z:.4f}) '
                    f'dist={dist * 1000:.2f}mm'
                )

    def _scanning_state_cb(self, msg: Bool):
        with self._lock:
            if msg.data != self._is_scanning:
                state_str = 'SCAN START' if msg.data else 'SCAN END'
                self.get_logger().info(f'Scanning state changed -> {state_str}')
            self._is_scanning = msg.data

    def _tcp_frame_cb(self, msg: String):
        with self._lock:
            if msg.data != self._active_tcp_frame:
                self.get_logger().info(f'TCP frame updated: {msg.data}')
                self._active_tcp_frame = msg.data

    # -- TF helpers -----------------------------------------------------------

    def _destroy_tf_listener(self):
        if self._tf_listener is not None:
            for attr in ('tf_sub', 'tf_static_sub'):
                sub = getattr(self._tf_listener, attr, None)
                if sub is not None:
                    try:
                        self.destroy_subscription(sub)
                    except Exception:
                        pass
            self._tf_listener = None
        self._tf_buffer = None

    def _reset_tf_listener(self):
        self._destroy_tf_listener()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_fail_count = 0
        self._last_tf_reset_monotonic = time.monotonic()

    def _get_tcp_position(self) -> Point | None:
        if self._tf_buffer is None:
            return None
        try:
            with self._lock:
                frame_id = self._active_tcp_frame
            transform = self._tf_buffer.lookup_transform(
                self.base_frame, frame_id, rclpy.time.Time()
            )
            self._tf_fail_count = 0
            pos = Point()
            pos.x = transform.transform.translation.x
            pos.y = transform.transform.translation.y
            pos.z = transform.transform.translation.z
            return pos
        except TransformException as e:
            self._tf_fail_count += 1
            if (
                self._tf_fail_count >= self._tf_reset_failure_threshold
                and (time.monotonic() - self._last_tf_reset_monotonic) >= 2.0
            ):
                self.get_logger().warn(
                    'TF listener appears stale; recreating subscriptions',
                    throttle_duration_sec=5.0,
                )
                self._reset_tf_listener()
            self.get_logger().info(
                f'TF lookup failed: {e}', throttle_duration_sec=2.0
            )
            return None

    # -- Publishing -----------------------------------------------------------

    def _publish_progression(self):
        with self._lock:
            is_welding = self._is_welding
            is_scanning = self._is_scanning
            bead_id = self._active_bead_id
            tot_len = self._total_length
            path = list(self._bead_path)

        if not is_welding and not is_scanning:
            return

        now = self.get_clock().now()

        tcp_pos = self._get_tcp_position()
        if tcp_pos is None:
            tcp_pos = Point()

        progression = (
            calculate_progression(tcp_pos, path, tot_len)
            if (bead_id and len(path) >= 2)
            else 0.0
        )

        self.get_logger().info(
            f'[Progression] prog={progression:.3f} | '
            f'tcp=({tcp_pos.x:.3f}, {tcp_pos.y:.3f}, {tcp_pos.z:.3f}) | '
            f'path_len={len(path)} | tot_len={tot_len:.3f} | '
            f'welding={is_welding} scanning={is_scanning}',
            throttle_duration_sec=1.0,
        )

        msg = WeldProgression()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = str(self.base_frame)
        msg.bead_id = str(bead_id)
        msg.progression = float(progression)
        msg.is_welding = bool(is_welding)
        msg.is_scanning = bool(is_scanning)
        msg.tcp_position = tcp_pos
        self._progression_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ProgressionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
