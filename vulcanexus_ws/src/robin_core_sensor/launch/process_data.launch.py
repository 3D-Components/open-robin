from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configurations
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")

    declared_arguments = [
        DeclareLaunchArgument(
            "input_topic",
            default_value="/robin/pointcloud",
            description="Input pointcloud topic from the sensor.",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/robin/weld_dimensions",
            description="Output topic for processed weld dimensions.",
        ),
    ]

    # Node to process pointcloud data and publish weld dimensions
    process_data_node = Node(
        package="robin_core_sensor",
        executable="process_data",
        name="weld_profile_processor",
        output="screen",
        parameters=[{
            "input_topic": input_topic,
            "output_topic": output_topic,
        }],
    )

    return LaunchDescription(
        declared_arguments
        + [
            process_data_node,
        ]
    )
