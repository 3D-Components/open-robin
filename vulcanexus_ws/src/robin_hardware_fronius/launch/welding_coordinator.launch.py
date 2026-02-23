"""
Launch file for the Welding Coordinator node.

This node coordinates welding operations via the OPC UA bridge services.
It requires the opcua_bridge node to be running.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    welding_coordinator_node = Node(
        package="robin_hardware_fronius",
        executable="welding_coordinator",
        name="welding_coordinator",
        output="screen",
    )

    return LaunchDescription([
        welding_coordinator_node,
    ])
