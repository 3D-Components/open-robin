# Copyright (c) 2024 ROBIN Project
#
# Launch file for MoveIt with Pilz Industrial Motion Planner
# Designed for use with MoveItCpp in robin_core_planner

import os
import yaml

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def declare_arguments():
    """Declare launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Launch RViz with MoveIt plugin?",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "publish_robot_description_semantic",
                default_value="true",
                description="Have move_group publish the SRDF",
            ),
        ]
    )


def generate_launch_description():
    # Launch configurations
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )

    # Build MoveIt configuration using moveit_configs_builder
    # This loads robot_description from robot_state_publisher topic
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur10e", package_name="robin_moveit_config")
        .robot_description_semantic(Path("srdf") / "robin.srdf.xacro", {"name": "ur10e"})
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    ld = LaunchDescription()
    ld.add_entity(declare_arguments())

    # Wait for robot description to be published by robot_state_publisher
    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )
    ld.add_action(wait_robot_description)

    # move_group node - provides planning services and trajectory execution
    # Your MoveItCpp planner can either:
    #   1. Use move_group's services (/plan_kinematic_path, /execute_trajectory)
    #   2. Plan internally with MoveItCpp and execute via controller directly
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    # RViz for visualization (optional)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robin_moveit_config"), "rviz", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Start move_group (and optionally RViz) after robot description is available
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node, rviz_node],
            )
        ),
    )

    return ld