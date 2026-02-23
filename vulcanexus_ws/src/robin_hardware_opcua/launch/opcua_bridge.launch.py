"""Launch file for OPC UA Bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for OPC UA Bridge."""
    
    pkg_share = FindPackageShare('robin_hardware_opcua')
    
    # Default config file path
    default_config = PathJoinSubstitution([pkg_share, 'config', 'opcua_bridge.yaml'])
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to YAML configuration file for OPC UA bridge'
    )

    # OPC UA Bridge node
    opcua_bridge_node = Node(
        package='robin_hardware_opcua',
        executable='opcua_bridge_node',
        name='opcua_bridge',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
        }],
    )

    return LaunchDescription([
        config_file_arg,
        opcua_bridge_node,
    ])
