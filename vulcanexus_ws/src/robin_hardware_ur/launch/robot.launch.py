from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.101",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "is_simulation",
            default_value="false",
            description="Whether to launch the robot in simulation mode.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "config", "ur_controllers.yaml"]
            ),
            description="YAML file with the controllers configuration.",
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    is_simulation = LaunchConfiguration("is_simulation")
    controllers_file = LaunchConfiguration("controllers_file")

    # Define description file
    description_file_real = PathJoinSubstitution([FindPackageShare("robin_core_bringup"), "urdf", "ur_fronius_garmo.urdf.xacro"])
    description_file_sim = PathJoinSubstitution([FindPackageShare("robin_core_bringup"), "urdf", "sim_ur_fronius_garmo.urdf.xacro"])

    # Include UR Control launch file. Different launch file is used for simulation vs real robot.
    control_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"])),
        launch_arguments={
            "ur_type": "ur10e",
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "launch_rviz": "false",
            "description_file": description_file_sim,
            "controllers_file": controllers_file,
            "gazebo_gui": "true",
        }.items(),
        condition=IfCondition(is_simulation),
    )
    control_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": robot_ip,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "launch_rviz": "false",
            "description_file": description_file_real,
            "controllers_file": controllers_file,
            "headless_mode": "true",
        }.items(),
        condition=UnlessCondition(is_simulation),
    )

    return LaunchDescription(declared_arguments + [control_sim_launch, control_real_launch])