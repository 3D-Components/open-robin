#!/usr/bin/env python3
"""
Geometry Republisher Node

Converts Float32MultiArray from /robin/weld_dimensions to BeadGeometry
so that Orion-LD's DDS bridge can deserialize the message type.

Uses timestamp from fronius topic to preserve original bag timestamps.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from std_msgs.msg import Float32MultiArray
from robin_interfaces.msg import BeadGeometry, FroniusSample
from builtin_interfaces.msg import Time


class GeometryRepublisher(Node):
    def __init__(self):
        super().__init__('geometry_republisher')

        # Parameters
        self.declare_parameter('input_topic', '/robin/weld_dimensions')
        self.declare_parameter('output_topic', '/robin/bead_geometry')
        self.declare_parameter('fronius_topic', '/robin/data/fronius')
        self.declare_parameter(
            'min_publish_period', 0.1
        )  # Rate limit: max 10 Hz

        input_topic = (
            self.get_parameter('input_topic')
            .get_parameter_value()
            .string_value
        )
        output_topic = (
            self.get_parameter('output_topic')
            .get_parameter_value()
            .string_value
        )
        fronius_topic = (
            self.get_parameter('fronius_topic')
            .get_parameter_value()
            .string_value
        )
        self.min_publish_period = (
            self.get_parameter('min_publish_period')
            .get_parameter_value()
            .double_value
        )

        # Track last fronius timestamp for syncing
        self._last_fronius_stamp = None
        self._last_publish_time = 0.0  # Wall clock for rate limiting

        # QoS for subscribers - BEST_EFFORT to receive from any publisher
        sub_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # QoS for publisher - BEST_EFFORT to match Orion DDS bridge
        pub_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscriber for Float32MultiArray (geometry data)
        self.subscription = self.create_subscription(
            Float32MultiArray, input_topic, self.geometry_callback, sub_qos
        )

        # Subscriber for FroniusSample (to get timestamps)
        self.fronius_sub = self.create_subscription(
            FroniusSample, fronius_topic, self.fronius_callback, sub_qos
        )

        # Publisher for BeadGeometry
        self.publisher = self.create_publisher(
            BeadGeometry, output_topic, pub_qos
        )

        self.msg_count = 0
        self._published_count = 0
        self.get_logger().info(
            f'Republishing {input_topic} → {output_topic} (using timestamps from {fronius_topic})'
        )

    def fronius_callback(self, msg: FroniusSample):
        """Store the latest fronius timestamp for syncing geometry data."""
        self._last_fronius_stamp = msg.header.stamp

    def geometry_callback(self, msg: Float32MultiArray):
        """Convert Float32MultiArray to BeadGeometry using fronius timestamp."""
        import time

        self.msg_count += 1

        # Debug: log every 50th message received
        if self.msg_count % 50 == 1:
            self.get_logger().info(
                f'Received msg #{self.msg_count}, data: {list(msg.data)[:4]}'
            )

        if len(msg.data) < 2:
            return

        width = float(msg.data[0])
        height = float(msg.data[1])

        # Skip zero/near-zero values (sensor not producing valid data)
        if width < 0.1 or height < 0.1:
            return

        # Rate limiting using wall clock
        now = time.time()
        if now - self._last_publish_time < self.min_publish_period:
            return

        geom = BeadGeometry()
        # Use fronius timestamp if available, otherwise use current ROS time
        if self._last_fronius_stamp is not None:
            geom.header.stamp = self._last_fronius_stamp
        else:
            geom.header.stamp = self.get_clock().now().to_msg()
        geom.header.frame_id = 'base_link'
        geom.width_mm = width
        geom.height_mm = height
        geom.cross_sectional_area_mm2 = (
            width * height * 0.5
        )  # Approximate triangular

        self.publisher.publish(geom)
        self._last_publish_time = now

        self._published_count += 1
        if self._published_count % 10 == 0:
            stamp_sec = geom.header.stamp.sec
            self.get_logger().info(
                f'Published {self._published_count} BeadGeometry (w={width:.2f}, h={height:.2f}, stamp_sec={stamp_sec})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = GeometryRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
