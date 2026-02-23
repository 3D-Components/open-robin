import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
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

    moveit_planner_node = Node(
        name="robin_moveit_planner",
        package="robin_moveit_control",
        executable="robin_planner.py",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            moveit_planner_node,
        ]
    )