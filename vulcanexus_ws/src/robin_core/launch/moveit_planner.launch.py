import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import LifecycleNode, Node
from moveit_configs_utils import MoveItConfigsBuilder


def _is_truthy(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _load_yaml(package_name: str, *path_parts) -> dict:
    file_path = os.path.join(
        get_package_share_directory(package_name), *path_parts
    )
    with open(file_path) as f:
        return yaml.safe_load(f)


def _launch_setup(context):
    is_sim = _is_truthy(context.launch_configurations.get("use_sim", "false"))
    launch_servo = _is_truthy(context.launch_configurations.get("launch_servo", "true"))

    bringup_share = get_package_share_directory("robin_bringup")

    if is_sim:
        urdf_xacro = os.path.join(bringup_share, "urdf", "sim_ur_fronius_garmo.urdf.xacro")
        sim_controllers = os.path.join(
            get_package_share_directory("robin_simulation"), "config", "sim_controllers.yaml")
        urdf_mappings = {
            "name": "ur",
            "ur_type": "ur10e",
            "simulation_controllers": sim_controllers,
        }
        traj_exec_file = "config/sim_moveit_controllers.yaml"
    else:
        urdf_xacro = os.path.join(bringup_share, "urdf", "ur_fronius_garmo.urdf.xacro")
        urdf_mappings = {"name": "ur10e"}
        traj_exec_file = "config/moveit_controllers.yaml"

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur10e", package_name="robin_moveit_config")
        .robot_description(file_path=urdf_xacro, mappings=urdf_mappings)
        .robot_description_semantic("srdf/robin.srdf.xacro", {"name": "robin_robot"})
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .trajectory_execution(file_path=traj_exec_file)
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    planner_params = os.path.join(
        get_package_share_directory("robin_core"), "config", "robin_planner_params.yaml")

    params = [moveit_config.to_dict(), planner_params]
    if is_sim:
        params.append({
            "controller_name": "joint_trajectory_controller",
            "use_sim_time": True,
            # MoveItPy creates an internal rclcpp node in-process; keep its
            # /clock override explicit and complete so QoS validation passes.
            "qos_overrides./clock.subscription.history": "keep_last",
            "qos_overrides./clock.subscription.depth": 1,
            "qos_overrides./clock.subscription.reliability": "best_effort",
            "qos_overrides./clock.subscription.durability": "volatile",
        })

    moveit_planner_node = LifecycleNode(
        name="robin_planner_node",
        package="robin_core",
        executable="robin_planner_node",
        namespace="",
        output="both",
        parameters=params,
    )

    servo_yaml = _load_yaml("robin_moveit_config", "config", "servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    butterworth_params = {
        "online_signal_smoothing": {"butterworth_filter_coeff": 5.0},
    }

    nodes = [moveit_planner_node,]

    if launch_servo:
        servo_extra_params = [{"use_sim_time": True}] if is_sim else []
        servo_node = Node(
            package="moveit_servo",
            executable="servo_node",
            output="screen",
            parameters=[
                servo_params,
                butterworth_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ] + servo_extra_params,
        )
        nodes.append(servo_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="Use simulation URDF and controllers"),
        DeclareLaunchArgument("launch_servo", default_value="true",
                              description="Start MoveIt Servo for Cartesian jogging"),
        OpaqueFunction(function=_launch_setup),
    ])
