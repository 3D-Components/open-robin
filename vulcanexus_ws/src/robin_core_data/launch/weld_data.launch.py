import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robin_core_data'),
        'config',
        'weld_data.yaml',
    )

    return LaunchDescription([
        Node(
            package='robin_core_data',
            executable='weld_data_node.py',
            name='weld_data_node',
            output='screen',
            parameters=[config],
        ),
    ])
