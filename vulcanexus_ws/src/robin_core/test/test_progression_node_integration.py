#!/usr/bin/env python3
"""
Integration tests for ProgressionNode.

Requires the progression_node to be running separately.
Run with: ros2 run robin_core progression_node
"""

import unittest
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point, TransformStamped
from robin_interfaces.msg import WeldProgression, ActiveBead

import tf2_ros


LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MockTFBroadcaster(Node):
    """Broadcasts mock TF transforms for testing."""

    def __init__(self):
        super().__init__('mock_tf_broadcaster')
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._tcp_x = 0.0
        self._tcp_y = 0.0
        self._tcp_z = 0.0
        self._timer = self.create_timer(0.01, self._broadcast_tf)

    def set_tcp_position(self, x: float, y: float, z: float):
        self._tcp_x = x
        self._tcp_y = y
        self._tcp_z = z

    def _broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'wire_tip'
        t.transform.translation.x = self._tcp_x
        t.transform.translation.y = self._tcp_y
        t.transform.translation.z = self._tcp_z
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)


class Harness(Node):
    """Test harness for publishing inputs and capturing outputs."""

    def __init__(self):
        super().__init__('test_harness')

        self._active_bead_pub = self.create_publisher(
            ActiveBead, '/robin/data/active_bead', LATCHED_QOS)
        self._welding_state_pub = self.create_publisher(
            Bool, '/robin/data/is_welding', LATCHED_QOS)
        self._scanning_state_pub = self.create_publisher(
            Bool, '/robin/data/is_scanning', LATCHED_QOS)

        self._received_progression = []
        self._progression_sub = self.create_subscription(
            WeldProgression, '/robin/data/progression',
            self._progression_callback, 10)

    def _progression_callback(self, msg: WeldProgression):
        self._received_progression.append(msg)

    def publish_active_bead(self, bead_id: str, start: tuple, end: tuple,
                            length: float, speed: float = 0.005,
                            wire_speed: float = 8.0):
        msg = ActiveBead()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.bead_id = bead_id
        start_pt = Point()
        start_pt.x, start_pt.y, start_pt.z = start
        end_pt = Point()
        end_pt.x, end_pt.y, end_pt.z = end
        msg.path = [start_pt, end_pt]
        msg.total_length = length
        msg.weld_speed = speed
        msg.wire_feed_speed = wire_speed
        self._active_bead_pub.publish(msg)

    def publish_welding_state(self, is_welding: bool):
        msg = Bool()
        msg.data = is_welding
        self._welding_state_pub.publish(msg)

    def publish_scanning_state(self, is_scanning: bool):
        msg = Bool()
        msg.data = is_scanning
        self._scanning_state_pub.publish(msg)

    def get_latest_progression(self) -> WeldProgression | None:
        return self._received_progression[-1] if self._received_progression else None

    def clear_received(self):
        self._received_progression.clear()


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_nodes(ros_context):
    harness = Harness()
    tf_broadcaster = MockTFBroadcaster()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    executor.add_node(tf_broadcaster)

    yield harness, tf_broadcaster, executor

    harness.destroy_node()
    tf_broadcaster.destroy_node()


def _spin(executor, duration_sec: float = 0.5):
    """Spin the executor for a fixed duration."""
    iterations = int(duration_sec / 0.01)
    for _ in range(iterations):
        executor.spin_once(timeout_sec=0.01)
        time.sleep(0.01)


class TestProgressionNodeIntegration:
    """
    Integration tests for ProgressionNode.

    NOTE: These tests require the progression_node to be running separately.
    Run with: ros2 run robin_core progression_node
    """

    @pytest.mark.skip(reason="Requires progression_node to be running")
    def test_progression_at_bead_start(self, test_nodes):
        """Progression is ~0.0 when TCP is at bead start."""
        harness, tf_broadcaster, executor = test_nodes

        harness.publish_active_bead(
            'test_bead_001', (0.5, 0.0, 0.1), (0.5, 0.3, 0.1), 0.3)
        tf_broadcaster.set_tcp_position(0.5, 0.0, 0.1)
        harness.publish_welding_state(True)

        _spin(executor)

        prog = harness.get_latest_progression()
        assert prog is not None
        assert prog.bead_id == 'test_bead_001'
        assert prog.progression == pytest.approx(0.0, abs=0.05)
        assert prog.is_welding is True

    @pytest.mark.skip(reason="Requires progression_node to be running")
    def test_progression_at_bead_midpoint(self, test_nodes):
        """Progression is ~0.5 when TCP is at bead midpoint."""
        harness, tf_broadcaster, executor = test_nodes

        harness.publish_active_bead(
            'test_bead_002', (0.5, 0.0, 0.1), (0.5, 0.3, 0.1), 0.3)
        tf_broadcaster.set_tcp_position(0.5, 0.15, 0.1)
        harness.publish_welding_state(True)

        _spin(executor)

        prog = harness.get_latest_progression()
        assert prog is not None
        assert prog.progression == pytest.approx(0.5, abs=0.05)

    @pytest.mark.skip(reason="Requires progression_node to be running")
    def test_progression_during_scan_pass(self, test_nodes):
        """Progression publishes during scan pass with is_scanning=True."""
        harness, tf_broadcaster, executor = test_nodes

        harness.publish_active_bead(
            'test_bead_003', (0.5, 0.0, 0.1), (0.5, 0.3, 0.1), 0.3)
        tf_broadcaster.set_tcp_position(0.5, 0.15, 0.1)
        harness.publish_scanning_state(True)

        _spin(executor)

        prog = harness.get_latest_progression()
        assert prog is not None
        assert prog.is_scanning is True
        assert prog.is_welding is False
        assert prog.progression == pytest.approx(0.5, abs=0.05)

    @pytest.mark.skip(reason="Requires progression_node to be running")
    def test_no_publish_when_idle(self, test_nodes):
        """No progression messages when neither welding nor scanning."""
        harness, tf_broadcaster, executor = test_nodes
        harness.clear_received()

        harness.publish_active_bead(
            'test_bead_004', (0.5, 0.0, 0.1), (0.5, 0.3, 0.1), 0.3)
        tf_broadcaster.set_tcp_position(0.5, 0.15, 0.1)
        harness.publish_welding_state(False)
        harness.publish_scanning_state(False)

        _spin(executor)

        assert harness.get_latest_progression() is None

    @pytest.mark.skip(reason="Requires progression_node restart mid-pass")
    def test_late_joiner_recovers_latched_active_bead(self, test_nodes):
        """A restarted progression node should recover bead context from latched topics."""
        harness, tf_broadcaster, executor = test_nodes

        harness.publish_active_bead(
            'test_bead_restart', (0.5, 0.0, 0.1), (0.5, 0.3, 0.1), 0.3)
        tf_broadcaster.set_tcp_position(0.5, 0.15, 0.1)
        harness.publish_welding_state(True)

        _spin(executor)

        prog = harness.get_latest_progression()
        assert prog is not None
        assert prog.bead_id == 'test_bead_restart'


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
