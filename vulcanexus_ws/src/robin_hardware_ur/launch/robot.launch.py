from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.101",
            description="IP address by which the robot can be reached.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robin_hardware_ur"), "config", "ur10e_controllers.yaml"]
            ),
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robin_bringup"), "urdf", "ur_fronius_garmo.urdf.xacro"]
            ),
            description="URDF xacro for the robot.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names.",
        ),
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="30",
            description="Timeout used when spawning controllers.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        ),
    ]

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robin_hardware_ur"), "launch", "robin_ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "description_file": LaunchConfiguration("description_file"),
            "tf_prefix": LaunchConfiguration("tf_prefix"),
            "controller_spawner_timeout": LaunchConfiguration("controller_spawner_timeout"),
            "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
            "activate_joint_controller": LaunchConfiguration("activate_joint_controller"),
            "launch_rviz": "false",
            "headless_mode": "true",
        }.items(),
    )

    return LaunchDescription(declared_arguments + [control_launch])
