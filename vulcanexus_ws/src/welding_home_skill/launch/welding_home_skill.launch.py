from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='welding_home_skill',
            executable='welding_home_skill_node',
            name='welding_home_skill',
            output='screen',
            emulate_tty=True,
        ),
    ])
