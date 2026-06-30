from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='welding_seam_skill',
            executable='welding_seam_skill_node',
            name='welding_seam_skill',
            output='screen',
            emulate_tty=True,
        ),
    ])
