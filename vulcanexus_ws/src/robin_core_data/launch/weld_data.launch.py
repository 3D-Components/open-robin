from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robin_core_data',
            executable='weld_data_node.py',
            name='weld_data_node',
            output='screen',
            parameters=[{
                'publish_rate': 100.0,
                'current_topic': '/fronius/display_current',
                'voltage_topic': '/fronius/display_voltage',
                'wire_feed_topic': '/fronius/display_wfs',
                'power_topic': '/fronius/display_power',
                'base_frame': 'base_link',
                'tcp_frame': 'weld_torch_tip',
            }],
        ),
    ])
