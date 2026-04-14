"""
Launch file for the TCP Manager node.

Manages the active TCP (Tool Center Point) frame and stickout value.
Publishes dynamic wire_tip TF and exposes services for TCP mode switching
and stickout calibration updates.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_stickout = LaunchConfiguration("default_stickout")
    publish_rate = LaunchConfiguration("publish_rate")

    declared_arguments = [
        DeclareLaunchArgument(
            "default_stickout",
            default_value="0.015",
            description="Default wire stickout in meters (CTWD).",
        ),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="100.0",
            description="Dynamic wire_tip TF publish rate in Hz.",
        ),
    ]

    tcp_manager_node = Node(
        package="robin_hardware_fronius",
        executable="tcp_manager",
        name="tcp_manager",
        output="screen",
        parameters=[{
            "default_stickout": default_stickout,
            "publish_rate": publish_rate,
            "default_mode": "welding",
        }],
    )

    return LaunchDescription(
        declared_arguments + [tcp_manager_node]
    )
