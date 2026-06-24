#!/usr/bin/env python3
"""
Integration tests for WeldDataNode (Fronius aggregation).

Requires the weld_data_node to be running separately.
Run with: ros2 run robin_core weld_data_node
"""

import unittest
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from robin_interfaces.msg import WeldProgression, WelderData


class Harness(Node):
    """Test harness for publishing inputs and capturing outputs."""

    def __init__(self):
        super().__init__('test_harness')

        # Input publishers
        self._current_pub = self.create_publisher(
            Float32, '/fronius/display_current', 10)
        self._voltage_pub = self.create_publisher(
            Float32, '/fronius/display_voltage', 10)
        self._wire_feed_pub = self.create_publisher(
            Float32, '/fronius/display_wfs', 10)
        self._power_pub = self.create_publisher(
            Float32, '/fronius/display_power', 10)
        self._energy_pub = self.create_publisher(
            Float32, '/fronius/display_energy', 10)
        self._progression_pub = self.create_publisher(
            WeldProgression, '/robin/data/progression', 10)

        # Output subscriber
        self._received_fronius = []
        self._fronius_sub = self.create_subscription(
            WelderData, '/robin/data/fronius',
            self._fronius_callback, qos_profile_sensor_data)

    def _fronius_callback(self, msg: WelderData):
        self._received_fronius.append(msg)

    def publish_fronius(self, current: float, voltage: float,
                        wire_feed: float, power: float, energy: float = 0.0):
        """Publish Fronius readings."""
        for pub, val in [
            (self._current_pub, current),
            (self._voltage_pub, voltage),
            (self._wire_feed_pub, wire_feed),
            (self._power_pub, power),
            (self._energy_pub, energy),
        ]:
            msg = Float32()
            msg.data = val
            pub.publish(msg)

    def publish_progression(self, bead_id: str, progression: float,
                            is_welding: bool, is_scanning: bool = False):
        """Publish a mock progression message."""
        msg = WeldProgression()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.bead_id = bead_id
        msg.progression = progression
        msg.is_welding = is_welding
        msg.is_scanning = is_scanning
        msg.tcp_position = Point()
        self._progression_pub.publish(msg)

    def get_latest_fronius(self) -> WelderData | None:
        return self._received_fronius[-1] if self._received_fronius else None

    def clear_received(self):
        self._received_fronius.clear()


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_nodes(ros_context):
    harness = Harness()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)

    yield harness, executor

    harness.destroy_node()


def _spin(executor, duration_sec: float = 0.5):
    iterations = int(duration_sec / 0.01)
    for _ in range(iterations):
        executor.spin_once(timeout_sec=0.01)
        time.sleep(0.01)


class TestWeldDataNodeIntegration:
    """
    Integration tests for WeldDataNode.

    NOTE: These tests require the weld_data_node to be running separately.
    Run with: ros2 run robin_core weld_data_node
    """

    @pytest.mark.skip(reason="Requires weld_data_node to be running")
    def test_fronius_data_stamped_with_progression(self, test_nodes):
        """Fronius output carries bead_id and progression from the progression topic."""
        harness, executor = test_nodes

        harness.publish_fronius(180.0, 24.5, 8.0, 4500.0, 12.0)
        harness.publish_progression('test_bead_001', 0.5, is_welding=True)

        _spin(executor)

        fronius = harness.get_latest_fronius()
        assert fronius is not None
        assert fronius.bead_id == 'test_bead_001'
        assert fronius.progression == pytest.approx(0.5, abs=0.05)
        assert fronius.current == pytest.approx(180.0, abs=1.0)
        assert fronius.voltage == pytest.approx(24.5, abs=0.5)
        assert fronius.wire_feed_speed == pytest.approx(8.0, abs=0.5)
        assert fronius.power == pytest.approx(4500.0, abs=10.0)
        assert fronius.energy == pytest.approx(12.0, abs=1.0)

    @pytest.mark.skip(reason="Requires weld_data_node to be running")
    def test_no_fronius_during_scan_pass(self, test_nodes):
        """Fronius data is not published during scan pass."""
        harness, executor = test_nodes
        harness.clear_received()

        harness.publish_fronius(180.0, 24.5, 8.0, 4500.0)
        harness.publish_progression('test_bead_002', 0.3, is_welding=False, is_scanning=True)

        _spin(executor)

        assert harness.get_latest_fronius() is None

    @pytest.mark.skip(reason="Requires weld_data_node to be running")
    def test_no_fronius_when_idle(self, test_nodes):
        """Fronius data is not published when no progression is received."""
        harness, executor = test_nodes
        harness.clear_received()

        harness.publish_fronius(180.0, 24.5, 8.0, 4500.0)
        # No progression published

        _spin(executor)

        assert harness.get_latest_fronius() is None

    @pytest.mark.skip(reason="Requires weld_data_node to be running")
    def test_fronius_updates_with_changing_progression(self, test_nodes):
        """Fronius output tracks changing progression values."""
        harness, executor = test_nodes

        harness.publish_fronius(200.0, 25.0, 9.0, 5000.0)
        harness.publish_progression('test_bead_003', 0.2, is_welding=True)
        _spin(executor, 0.3)

        first = harness.get_latest_fronius()
        assert first is not None
        assert first.progression == pytest.approx(0.2, abs=0.05)

        harness.publish_progression('test_bead_003', 0.8, is_welding=True)
        _spin(executor, 0.3)

        second = harness.get_latest_fronius()
        assert second is not None
        assert second.progression == pytest.approx(0.8, abs=0.05)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
