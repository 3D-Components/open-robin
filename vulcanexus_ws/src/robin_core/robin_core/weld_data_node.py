#!/usr/bin/env python3
"""
ROBIN Weld Data Node — Fronius data aggregation with progression stamping.

Subscribes to the progression topic (single source of truth for bead
position) and the Fronius power-source readings, then publishes a
combined WelderData message aligned by progression value.

Progression tracking itself is handled by progression_node.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, Float32
from robin_interfaces.msg import WeldProgression, WelderData
from rclpy.qos import qos_profile_sensor_data

from robin_core.utils.tcp_utils import LATCHED_QOS
from robin_core.utils.time_utils import (
    RosTimeHealth,
    format_time_policy,
    make_steady_clock,
    monotonic_age_seconds,
    stamp_age_seconds,
)


class WeldDataNode(Node):
    """Stamps Fronius readings with bead progression and republishes."""

    def __init__(self):
        super().__init__('weld_data_node')
        self.declare_parameter('publish_rate', 100.0)

        self._subscriptions = []

        # Fronius state (simple float assignments are GIL-atomic)
        self._current = 0.0
        self._voltage = 0.0
        self._wire_feed_speed = 0.0
        self._power = 0.0
        self._energy = 0.0

        # Latest progression snapshot (updated by callback)
        self._last_progression = None  # type: WeldProgression | None
        self._last_progression_monotonic: float | None = None
        self._last_fronius_input_monotonic: float | None = None
        self._is_welding = False
        self._progression_timeout = 1.0

        self.publish_timer = None
        self._timer_clock = make_steady_clock()
        self._clock_health = None

        # -- Configure --
        self.publish_rate = self.get_parameter('publish_rate').value
        self._clock_health = RosTimeHealth(
            self,
            name='weld_data_node',
            stall_after_s=1.0,
            warn_interval_s=2.0,
        )

        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.pub_cb_group = MutuallyExclusiveCallbackGroup()

        self._subscriptions = [
            self.create_subscription(
                Float32, 'fronius/display_current',
                self._current_cb, 10,
                callback_group=self.sub_cb_group,
            ),
            self.create_subscription(
                Float32, 'fronius/display_voltage',
                self._voltage_cb, 10,
                callback_group=self.sub_cb_group,
            ),
            self.create_subscription(
                Float32, 'fronius/display_wfs',
                self._wfs_cb, 10,
                callback_group=self.sub_cb_group,
            ),
            self.create_subscription(
                Float32, 'fronius/display_power',
                self._power_cb, 10,
                callback_group=self.sub_cb_group,
            ),
            self.create_subscription(
                Float32, 'fronius/display_energy',
                self._energy_cb, 10,
                callback_group=self.sub_cb_group,
            ),
            self.create_subscription(
                WeldProgression, 'robin/data/progression',
                self._progression_cb, 10,
                callback_group=self.sub_cb_group,
            ),
            self.create_subscription(
                Bool, 'robin/data/is_welding',
                self._welding_state_cb, LATCHED_QOS,
                callback_group=self.sub_cb_group,
            ),
        ]

        # BEST_EFFORT so rqt_plot (and other sensor-data consumers) can subscribe.
        # Fronius readings are high-rate streaming data — dropped messages are
        # acceptable and RELIABLE would block rqt_plot's default subscriber.
        self._fronius_pub = self.create_publisher(
            WelderData, 'robin/data/fronius', qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Configured: publish_rate={self.publish_rate}Hz'
        )
        self.get_logger().info(
            format_time_policy(
                self,
                ros_time_inputs=['/clock', 'robin/data/progression'],
                steady_time_inputs=['publish timer', 'Fronius input freshness'],
            )
        )

        # -- Activate --
        period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.1
        self.publish_timer = self.create_timer(
            period, self._publish_data,
            callback_group=self.pub_cb_group,
            clock=self._timer_clock,
        )

        self.get_logger().info(
            f'Activated: publishing at {self.publish_rate}Hz'
        )

    # -- Subscription callbacks -----------------------------------------------

    def _current_cb(self, msg: Float32):
        self._current = msg.data
        self._last_fronius_input_monotonic = time.monotonic()

    def _voltage_cb(self, msg: Float32):
        self._voltage = msg.data
        self._last_fronius_input_monotonic = time.monotonic()

    def _wfs_cb(self, msg: Float32):
        self._wire_feed_speed = msg.data
        self._last_fronius_input_monotonic = time.monotonic()

    def _power_cb(self, msg: Float32):
        self._power = msg.data
        self._last_fronius_input_monotonic = time.monotonic()

    def _energy_cb(self, msg: Float32):
        self._energy = msg.data
        self._last_fronius_input_monotonic = time.monotonic()

    def _progression_cb(self, msg: WeldProgression):
        self._last_progression = msg
        self._last_progression_monotonic = time.monotonic()

    def _welding_state_cb(self, msg: Bool):
        if msg.data != self._is_welding:
            # Clear cached progression whenever the weld pass boundary changes
            # so we never keep stamping with stale data from the previous pass.
            self._last_progression = None
            self._last_progression_monotonic = None
        self._is_welding = msg.data

    def _describe_progression_gap(self) -> str:
        if self._last_progression is None:
            return 'no progression message has been received'

        receipt_age = monotonic_age_seconds(self._last_progression_monotonic)
        if receipt_age is not None and receipt_age > self._progression_timeout:
            return f'last progression arrived {receipt_age:.2f}s ago'

        stamp_age = stamp_age_seconds(self.get_clock(), self._last_progression.header.stamp)
        if stamp_age is not None and stamp_age > self._progression_timeout:
            return f'latest progression stamp is {stamp_age:.2f}s old in ROS time'

        if not self._last_progression.is_welding:
            return 'latest progression message is not marked as welding'

        return 'progression context is incomplete'

    # -- Publishing -----------------------------------------------------------

    def _publish_data(self):
        if not self._is_welding:
            return

        # Warn but do NOT block — the Fronius data is still valid even
        # when /clock is slow or bursty in simulation.
        self._clock_health.warn_if_unhealthy(
            active=True,
            context='welder data publish (ROS time may be slow)',
        )

        prog = self._last_progression
        if prog is None or not prog.is_welding:
            if (
                self._last_fronius_input_monotonic is not None
                and (time.monotonic() - self._last_fronius_input_monotonic) <= 1.0
            ):
                self.get_logger().warn(
                    'Raw Fronius inputs are arriving during an active weld '
                    f'pass, but no fresh progression is available '
                    f'({self._describe_progression_gap()}); '
                    'skipping /robin/data/fronius publish',
                    throttle_duration_sec=2.0,
                )
            return

        progression_age = monotonic_age_seconds(self._last_progression_monotonic)
        if progression_age is not None and progression_age > self._progression_timeout:
            self.get_logger().warn(
                'Active weld pass still has Fronius data, but progression has gone '
                f'stale ({self._describe_progression_gap()}); '
                'skipping /robin/data/fronius publish',
                throttle_duration_sec=2.0,
            )
            return

        now = self.get_clock().now()

        msg = WelderData()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = prog.header.frame_id
        msg.bead_id = prog.bead_id
        msg.progression = prog.progression
        msg.current = float(self._current)
        msg.voltage = float(self._voltage)
        msg.wire_feed_speed = float(self._wire_feed_speed)
        msg.power = float(self._power)
        msg.energy = float(self._energy)
        self._fronius_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WeldDataNode()
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
