"""Shared timing helpers for sim-safe stamping and watchdog behavior."""

from __future__ import annotations

import time
from typing import Iterable

from builtin_interfaces.msg import Time as TimeMsg
from rclpy.clock import Clock, ClockType


def make_steady_clock() -> Clock:
    """Return a steady clock for watchdogs and retry loops."""
    return Clock(clock_type=ClockType.STEADY_TIME)


def stamp_to_nanoseconds(stamp: TimeMsg | None) -> int:
    """Convert a ROS stamp message to nanoseconds."""
    if stamp is None:
        return 0
    return (int(stamp.sec) * 1_000_000_000) + int(stamp.nanosec)


def stamp_age_seconds(clock, stamp: TimeMsg | None) -> float | None:
    """Return stamp age in ROS time seconds when available."""
    stamp_ns = stamp_to_nanoseconds(stamp)
    now_ns = int(clock.now().nanoseconds)
    if stamp_ns <= 0 or now_ns <= 0:
        return None
    return max(0.0, (now_ns - stamp_ns) / 1_000_000_000.0)


def monotonic_age_seconds(last_monotonic: float | None) -> float | None:
    """Return wall-clock age from a saved monotonic timestamp."""
    if last_monotonic is None:
        return None
    return max(0.0, time.monotonic() - last_monotonic)


def is_sim_time_enabled(node) -> bool:
    """Return True when the node is configured to consume `/clock`."""
    try:
        return bool(node.get_parameter("use_sim_time").value)
    except Exception:
        return False


def format_time_policy(
    node,
    *,
    ros_time_inputs: Iterable[str] = (),
    steady_time_inputs: Iterable[str] = (),
) -> str:
    """Build a one-line startup summary for node timing policy."""
    mode = "sim_time" if is_sim_time_enabled(node) else "system_time"
    ros_inputs = ", ".join(ros_time_inputs) or "<none>"
    steady_inputs = ", ".join(steady_time_inputs) or "<none>"
    return (
        f"Time policy: clock_mode={mode}, "
        f"ros_time=[{ros_inputs}], steady_time=[{steady_inputs}]"
    )


class RosTimeHealth:
    """Track whether ROS time has started and is still advancing."""

    def __init__(
        self,
        node,
        *,
        name: str,
        stall_after_s: float = 1.0,
        warn_interval_s: float = 2.0,
    ) -> None:
        self._node = node
        self._name = name
        self._stall_after_s = float(stall_after_s)
        self._warn_interval_s = float(warn_interval_s)
        self._last_ros_ns: int | None = None
        self._last_advance_monotonic = time.monotonic()
        self._last_warn_monotonic = 0.0

    def update(self, ros_now_ns: int | None = None) -> int:
        """Record the latest observed ROS time sample."""
        if ros_now_ns is None:
            ros_now_ns = int(self._node.get_clock().now().nanoseconds)
        now_mono = time.monotonic()
        if self._last_ros_ns is None:
            self._last_ros_ns = ros_now_ns
            if ros_now_ns > 0:
                self._last_advance_monotonic = now_mono
            return ros_now_ns

        if ros_now_ns != self._last_ros_ns:
            self._last_advance_monotonic = now_mono
        self._last_ros_ns = ros_now_ns
        return ros_now_ns

    def issue(self, *, active: bool) -> str | None:
        """Return a human-readable clock issue when sim time is unhealthy."""
        if not active or not is_sim_time_enabled(self._node):
            return None

        ros_now_ns = self.update()
        if ros_now_ns <= 0:
            return "ROS time has not started yet; `/clock` may be missing or stalled"

        stalled_for = time.monotonic() - self._last_advance_monotonic
        if stalled_for >= self._stall_after_s:
            return f"ROS time has not advanced for {stalled_for:.2f}s"
        return None

    def warn_if_unhealthy(self, *, active: bool, context: str) -> str | None:
        """Log a throttled warning when sim time is unhealthy."""
        issue = self.issue(active=active)
        if issue is None:
            return None

        now_mono = time.monotonic()
        if (now_mono - self._last_warn_monotonic) >= self._warn_interval_s:
            self._node.get_logger().warn(
                f"[ClockHealth:{self._name}] {context}: {issue}"
            )
            self._last_warn_monotonic = now_mono
        return issue
