import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(package_name: str, *path_parts) -> dict:
    """Load a YAML file from a package share directory and return as dict."""
    file_path = os.path.join(
        get_package_share_directory(package_name), *path_parts
    )
    with open(file_path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────
    launch_servo_arg = DeclareLaunchArgument(
        "launch_servo",
        default_value="true",
        description="Start MoveIt Servo for Cartesian jogging (idle until enabled via service)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── MoveIt configuration ──────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur10e", package_name="robin_moveit_config")
        .robot_description_semantic("srdf/robin.srdf.xacro", {"name": "ur10e"})
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    # ── Robin planner node (MoveItPy) ─────────────────────────────────
    planner_params = PathJoinSubstitution([
        FindPackageShare('robin_core_planner'), 'config', 'robin_planner_params.yaml'
    ])

    moveit_planner_node = Node(
        name="robin_moveit_planner",
        package="robin_core_planner",
        executable="robin_planner.py",
        output="both",
        parameters=[moveit_config.to_dict(), planner_params],
    )

    # ── MoveIt Servo node (Cartesian jog) ─────────────────────────────
    #    Runs continuously and stays effectively idle while paused.
    #    Operator panel toggles jog behavior via /servo_node/pause_servo
    #    and /servo_node/switch_command_type.
    #
    #    The servo config is loaded as a Python dict and wrapped under the
    #    'moveit_servo' namespace — this matches the generate_parameter_library
    #    convention used by servo_node (all params live under moveit_servo.*).
    servo_yaml = _load_yaml("robin_moveit_config", "config", "servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # The Butterworth smoothing plugin uses generate_parameter_library at the
    # NODE root (online_signal_smoothing.*), NOT under moveit_servo.*.  If we
    # put it inside servo_config.yaml it would be wrapped under moveit_servo
    # and the plugin would never see it — falling back to the nearly-useless
    # default of 1.5 (~21 Hz cutoff, no real smoothing at 100 Hz servo rate).
    # coeff 5.0 → ~3 Hz cutoff → proper smoothing of staircase commands and
    # IK jitter while keeping <150 ms latency (fine for manual jog).
    butterworth_params = {
        "online_signal_smoothing": {
            "butterworth_filter_coeff": 5.0,
        },
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        condition=IfCondition(launch_servo),
        output="screen",
        parameters=[
            servo_params,
            butterworth_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            launch_servo_arg,
            use_sim_time_arg,
            moveit_planner_node,
            servo_node,
        ]
    )