#!/usr/bin/env python3
"""
Telemetry Aggregator (ROS 2 -> DDS-friendly output)

This node aggregates multiple ROS 2 topics into a single message with primitive
float32 fields so Orion-LD's DDS bridge can deserialize and store it reliably.

Default subscriptions are aligned with the existing welding rosbag demo, but
all topic names are configurable via parameters to support other processes.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

from robin_interfaces.msg import ProcessTelemetry, FroniusSample, WelderData

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


def _load_profile_ros2(path: str) -> Dict[str, Any]:
    """Load the ros2 section from a profile YAML, or return empty dict."""
    if not path or yaml is None:
        return {}
    p = Path(path)
    if not p.is_file():
        return {}
    with open(p) as f:
        data = yaml.safe_load(f) or {}
    return data.get('ros2', {})


@dataclass
class _PoseSample:
    t_sec: float
    x_m: float
    y_m: float
    z_m: float


class TelemetryAggregator(Node):
    def __init__(self) -> None:
        super().__init__('telemetry_aggregator_node')

        self.declare_parameter('profile_config', '')
        profile_path = self.get_parameter('profile_config').get_parameter_value().string_value
        ros2_cfg = _load_profile_ros2(profile_path)
        topics = ros2_cfg.get('topics', {})

        self.declare_parameter('geometry_topic', topics.get('geometry', '/robin/weld_dimensions'))
        self.declare_parameter('fronius_topic', topics.get('process_params', '/robin/data/fronius'))
        self.declare_parameter('welder_topic', '/robin/data/welder')
        self.declare_parameter('pose_topic', topics.get('pose', '/tcp_pose_broadcaster/pose'))
        self.declare_parameter('output_topic', topics.get('telemetry_output', '/robin/telemetry'))
        self.declare_parameter('min_publish_period', ros2_cfg.get('min_publish_period', 0.1))

        self.geometry_topic = (
            self.get_parameter('geometry_topic').get_parameter_value().string_value
        )
        self.fronius_topic = (
            self.get_parameter('fronius_topic').get_parameter_value().string_value
        )
        self.welder_topic = (
            self.get_parameter('welder_topic').get_parameter_value().string_value
        )
        self.pose_topic = (
            self.get_parameter('pose_topic').get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter('output_topic').get_parameter_value().string_value
        )
        self.min_publish_period = (
            self.get_parameter('min_publish_period')
            .get_parameter_value()
            .double_value
        )

        if profile_path:
            self.get_logger().info(f'Loaded topic config from profile: {profile_path}')

        # Latest cached signals
        self._height: Optional[float] = None
        self._width: Optional[float] = None
        self._current: Optional[float] = None
        self._voltage: Optional[float] = None
        self._speed_mm_s: Optional[float] = None

        self._last_pose: Optional[_PoseSample] = None
        self._last_publish_wall: float = 0.0
        self._published: int = 0

        # QoS: match typical bag playback + DDS bridge BEST_EFFORT behavior.
        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscribers
        self.create_subscription(
            Float32MultiArray, self.geometry_topic, self._geometry_cb, qos
        )
        # Support both a vendor-specific topic (legacy demos) and a generic one.
        self.create_subscription(FroniusSample, self.fronius_topic, self._fronius_cb, qos)
        self.create_subscription(WelderData, self.welder_topic, self._welder_cb, qos)
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, qos)

        # Publisher (DDS-friendly aggregate)
        self._pub = self.create_publisher(ProcessTelemetry, self.output_topic, qos)

        self.get_logger().info(
            'Telemetry aggregator ready '
            f'(geometry={self.geometry_topic}, fronius={self.fronius_topic}, '
            f'welder={self.welder_topic}, pose={self.pose_topic}) -> {self.output_topic}'
        )

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------
    def _geometry_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        # Convention in existing demos: data[0]=width, data[1]=height
        self._width = float(msg.data[0])
        self._height = float(msg.data[1])

    def _fronius_cb(self, msg: FroniusSample) -> None:
        self._current = float(msg.current)
        self._voltage = float(msg.voltage)
        self._maybe_publish()

    def _welder_cb(self, msg: WelderData) -> None:
        # Generic alternative to FroniusSample.
        self._current = float(msg.current_a)
        self._voltage = float(msg.voltage_v)
        self._maybe_publish()

    def _pose_cb(self, msg: PoseStamped) -> None:
        try:
            t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        except Exception:
            return

        p = msg.pose.position
        sample = _PoseSample(t_sec=t, x_m=float(p.x), y_m=float(p.y), z_m=float(p.z))

        if self._last_pose is not None:
            dt = sample.t_sec - self._last_pose.t_sec
            if dt > 1e-6:
                dx = sample.x_m - self._last_pose.x_m
                dy = sample.y_m - self._last_pose.y_m
                dz = sample.z_m - self._last_pose.z_m
                dist_m = math.sqrt(dx * dx + dy * dy + dz * dz)
                self._speed_mm_s = (dist_m * 1000.0) / dt

        self._last_pose = sample

    # ------------------------------------------------------------------
    # Publish logic
    # ------------------------------------------------------------------
    def _maybe_publish(self) -> None:
        now_wall = time.time()
        if (now_wall - self._last_publish_wall) < self.min_publish_period:
            return

        if self._height is None or self._width is None:
            return
        if self._current is None or self._voltage is None:
            return

        msg = ProcessTelemetry()

        # Use wall-clock time for "live" demo behavior (independent of /use_sim_time).
        sec = int(now_wall)
        nanosec = int((now_wall - sec) * 1e9)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = ''

        msg.height = float(self._height)
        msg.width = float(self._width)
        msg.speed = float(self._speed_mm_s or 0.0)
        msg.current = float(self._current)
        msg.voltage = float(self._voltage)

        self._pub.publish(msg)
        self._last_publish_wall = now_wall
        self._published += 1
        if self._published % 50 == 1:
            self.get_logger().info(
                f'Published telemetry #{self._published}: '
                f'h={msg.height:.2f}, w={msg.width:.2f}, '
                f'speed={msg.speed:.1f}mm/s, I={msg.current:.1f}A, V={msg.voltage:.1f}V'
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TelemetryAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

