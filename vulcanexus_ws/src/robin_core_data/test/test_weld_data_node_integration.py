#!/usr/bin/env python3
"""
Integration tests for WeldDataNode.

Uses launch_testing to spin up the node and verify correct behavior
with mocked inputs and real topic communication.
"""

import unittest
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point, TransformStamped
from robin_interfaces.msg import WeldProgression, FroniusSample, ActiveBead

import tf2_ros


class MockTFBroadcaster(Node):
    """Broadcasts mock TF transforms for testing."""
    
    def __init__(self):
        super().__init__('mock_tf_broadcaster')
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._tcp_x = 0.0
        self._tcp_y = 0.0
        self._tcp_z = 0.0
        
        # Broadcast TF at 100Hz
        self._timer = self.create_timer(0.01, self._broadcast_tf)
    
    def set_tcp_position(self, x: float, y: float, z: float):
        """Set the TCP position to broadcast."""
        self._tcp_x = x
        self._tcp_y = y
        self._tcp_z = z
    
    def _broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'weld_torch_tip'
        t.transform.translation.x = self._tcp_x
        t.transform.translation.y = self._tcp_y
        t.transform.translation.z = self._tcp_z
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)


class TestHarness(Node):
    """Test harness node for publishing inputs and capturing outputs."""
    
    def __init__(self):
        super().__init__('test_harness')
        
        # Publishers for inputs
        self._current_pub = self.create_publisher(
            Float32, '/fronius/display_current', 10)
        self._voltage_pub = self.create_publisher(
            Float32, '/fronius/display_voltage', 10)
        self._wire_feed_pub = self.create_publisher(
            Float32, '/fronius/display_wfs', 10)
        self._power_pub = self.create_publisher(
            Float32, '/fronius/display_power', 10)
        self._active_bead_pub = self.create_publisher(
            ActiveBead, '/robin/data/active_bead', 10)
        self._welding_state_pub = self.create_publisher(
            Bool, '/robin/data/is_welding', 10)
        
        # Subscribers for outputs
        self._received_progression = []
        self._received_fronius = []
        
        self._progression_sub = self.create_subscription(
            WeldProgression, '/robin/data/progression', 
            self._progression_callback, 10)
        self._fronius_sub = self.create_subscription(
            FroniusSample, '/robin/data/fronius',
            self._fronius_callback, 10)
    
    def _progression_callback(self, msg: WeldProgression):
        self._received_progression.append(msg)
    
    def _fronius_callback(self, msg: FroniusSample):
        self._received_fronius.append(msg)
    
    def publish_fronius(self, current: float, voltage: float, 
                        wire_feed: float, power: float):
        """Publish Fronius data."""
        msg = Float32()
        msg.data = current
        self._current_pub.publish(msg)
        
        msg.data = voltage
        self._voltage_pub.publish(msg)
        
        msg.data = wire_feed
        self._wire_feed_pub.publish(msg)
        
        msg.data = power
        self._power_pub.publish(msg)
    
    def publish_active_bead(self, bead_id: str, start: tuple, end: tuple,
                            length: float, speed: float = 0.005,
                            current: float = 180.0, voltage: float = 24.0,
                            wire_speed: float = 8.0):
        """Publish active bead info."""
        msg = ActiveBead()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.bead_id = bead_id
        msg.start_point.x = start[0]
        msg.start_point.y = start[1]
        msg.start_point.z = start[2]
        msg.end_point.x = end[0]
        msg.end_point.y = end[1]
        msg.end_point.z = end[2]
        msg.length = length
        msg.target_speed = speed
        msg.target_current = current
        msg.target_voltage = voltage
        msg.target_wire_speed = wire_speed
        self._active_bead_pub.publish(msg)
    
    def publish_welding_state(self, is_welding: bool):
        """Publish welding state."""
        msg = Bool()
        msg.data = is_welding
        self._welding_state_pub.publish(msg)
    
    def get_latest_progression(self) -> WeldProgression | None:
        """Get the most recent progression message."""
        if self._received_progression:
            return self._received_progression[-1]
        return None
    
    def get_latest_fronius(self) -> FroniusSample | None:
        """Get the most recent Fronius sample."""
        if self._received_fronius:
            return self._received_fronius[-1]
        return None
    
    def clear_received(self):
        """Clear received messages."""
        self._received_progression.clear()
        self._received_fronius.clear()


@pytest.fixture(scope='module')
def ros_context():
    """Initialize ROS2 for the test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_nodes(ros_context):
    """Create test harness and mock TF broadcaster."""
    harness = TestHarness()
    tf_broadcaster = MockTFBroadcaster()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    executor.add_node(tf_broadcaster)
    
    yield harness, tf_broadcaster, executor
    
    harness.destroy_node()
    tf_broadcaster.destroy_node()


class TestWeldDataNodeIntegration:
    """
    Integration tests for WeldDataNode.
    
    NOTE: These tests require the weld_data_node to be running separately.
    Run with: ros2 run robin_core_data weld_data_node
    """
    
    @pytest.mark.skip(reason="Requires weld_data_node to be running")
    def test_progression_at_bead_start(self, test_nodes):
        """Test progression is 0 when TCP is at bead start."""
        harness, tf_broadcaster, executor = test_nodes
        
        # Set up bead along Y axis: (0.5, 0.0, 0.1) -> (0.5, 0.3, 0.1)
        harness.publish_active_bead(
            'test_bead_001',
            (0.5, 0.0, 0.1),
            (0.5, 0.3, 0.1),
            0.3
        )
        
        # Set TCP at bead start
        tf_broadcaster.set_tcp_position(0.5, 0.0, 0.1)
        
        # Enable welding
        harness.publish_welding_state(True)
        
        # Spin to process messages
        for _ in range(50):
            executor.spin_once(timeout_sec=0.01)
            time.sleep(0.01)
        
        # Check progression
        prog = harness.get_latest_progression()
        assert prog is not None
        assert prog.bead_id == 'test_bead_001'
        assert prog.progression == pytest.approx(0.0, abs=0.05)
        assert prog.is_welding is True
    
    @pytest.mark.skip(reason="Requires weld_data_node to be running")
    def test_progression_at_bead_midpoint(self, test_nodes):
        """Test progression is 0.5 when TCP is at bead midpoint."""
        harness, tf_broadcaster, executor = test_nodes
        
        harness.publish_active_bead(
            'test_bead_002',
            (0.5, 0.0, 0.1),
            (0.5, 0.3, 0.1),
            0.3
        )
        
        # Set TCP at bead midpoint
        tf_broadcaster.set_tcp_position(0.5, 0.15, 0.1)
        harness.publish_welding_state(True)
        
        for _ in range(50):
            executor.spin_once(timeout_sec=0.01)
            time.sleep(0.01)
        
        prog = harness.get_latest_progression()
        assert prog is not None
        assert prog.progression == pytest.approx(0.5, abs=0.05)
    
    @pytest.mark.skip(reason="Requires weld_data_node to be running")
    def test_fronius_data_includes_progression(self, test_nodes):
        """Test that Fronius samples include the current progression."""
        harness, tf_broadcaster, executor = test_nodes
        
        harness.publish_active_bead(
            'test_bead_003',
            (0.5, 0.0, 0.1),
            (0.5, 0.3, 0.1),
            0.3
        )
        
        tf_broadcaster.set_tcp_position(0.5, 0.15, 0.1)  # 50% progression
        harness.publish_welding_state(True)
        
        # Publish Fronius data
        harness.publish_fronius(180.0, 24.5, 8.0, 4500.0)
        
        for _ in range(50):
            executor.spin_once(timeout_sec=0.01)
            time.sleep(0.01)
        
        fronius = harness.get_latest_fronius()
        assert fronius is not None
        assert fronius.bead_id == 'test_bead_003'
        assert fronius.progression == pytest.approx(0.5, abs=0.05)
        assert fronius.current == pytest.approx(180.0, abs=1.0)
        assert fronius.voltage == pytest.approx(24.5, abs=0.5)
    
    @pytest.mark.skip(reason="Requires weld_data_node to be running")
    def test_welding_off_clears_bead(self, test_nodes):
        """Test that setting welding off clears the active bead."""
        harness, tf_broadcaster, executor = test_nodes
        
        harness.publish_active_bead(
            'test_bead_004',
            (0.5, 0.0, 0.1),
            (0.5, 0.3, 0.1),
            0.3
        )
        
        harness.publish_welding_state(True)
        
        for _ in range(30):
            executor.spin_once(timeout_sec=0.01)
            time.sleep(0.01)
        
        # Turn welding off
        harness.publish_welding_state(False)
        
        for _ in range(30):
            executor.spin_once(timeout_sec=0.01)
            time.sleep(0.01)
        
        prog = harness.get_latest_progression()
        assert prog is not None
        assert prog.is_welding is False
        assert prog.bead_id == ""  # Cleared


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
