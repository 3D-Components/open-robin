"""Launch file for the robin_experiment node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robin_experiment",
            executable="experiment_node.py",
            name="robin_experiment",
            output="screen",
            parameters=[{
                "default_bead_pitch": 0.030,
                "default_margin_x": 0.015,
                "default_margin_y": 0.015,
                "default_experiment_spacing_x": 0.030,
                "default_experiment_spacing_y": 0.030,
                "default_experiment_bead_length": 0.100,
                "inter_bead_clearance_height": 0.250,
                "base_frame": "base_link",
                "preview_topic": "/robin/plates/markers",
                "preview_path_topic": "/robin/experiment/preview_path",
                "execute_bead_timeout": 120.0,
            }],
        ),
    ])
