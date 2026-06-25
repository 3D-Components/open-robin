#!/usr/bin/env python3
"""Monitor sim clock startup and advancement using a steady watchdog."""

import rclpy
from rclpy.node import Node

from robin_core.utils.time_utils import RosTimeHealth, format_time_policy, make_steady_clock


class SimClockMonitor(Node):
    """Log clear diagnostics when `/clock` has not started or stops advancing."""

    def __init__(self):
        super().__init__('sim_clock_monitor')
        self.declare_parameter('warn_after', 1.0)
        self.declare_parameter('monitor_period', 0.5)

        warn_after = float(self.get_parameter('warn_after').value)
        monitor_period = float(self.get_parameter('monitor_period').value)

        self._clock_health = RosTimeHealth(
            self,
            name='sim_clock_monitor',
            stall_after_s=warn_after,
            warn_interval_s=max(2.0, monitor_period),
        )
        self._steady_clock = make_steady_clock()
        self._timer = self.create_timer(
            monitor_period,
            self._check_clock,
            clock=self._steady_clock,
        )

        self.get_logger().info(
            format_time_policy(
                self,
                ros_time_inputs=['/clock'],
                steady_time_inputs=['sim clock watchdog'],
            )
        )

    def _check_clock(self):
        self._clock_health.warn_if_unhealthy(
            active=True,
            context='simulation bringup expects Gazebo to publish `/clock`',
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimClockMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
