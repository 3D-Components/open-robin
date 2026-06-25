"""Launch rosbag recording for a weld experiment.

Reads the topic list from config/rosbag_topics.yaml and starts
``ros2 bag record`` with mcap storage alongside the experiment node.

Usage (standalone):
    ros2 launch robin_core experiment_record.launch.py

Usage (as part of robin_main with record_bag:=true):
    ros2 launch robin_bringup robin_main.launch.py record_bag:=true
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("robin_core")

    # ---- Launch arguments ----
    declared_arguments = [
        DeclareLaunchArgument(
            "bag_dir",
            default_value="/workspace/ros2_packages/bags",
            description="Parent directory for rosbag output.",
        ),
        DeclareLaunchArgument(
            "bag_name",
            default_value="",
            description="Bag subdirectory name. Empty = auto-timestamped by ros2 bag.",
        ),
        DeclareLaunchArgument(
            "storage",
            default_value="mcap",
            description="Storage backend (mcap | sqlite3).",
        ),
        DeclareLaunchArgument(
            "compression",
            default_value="none",
            description="Compression mode (none | zstd_fast | zstd_small).",
        ),
        DeclareLaunchArgument(
            "include_experiment",
            default_value="true",
            description="Also start the experiment node (set false when launched from robin_main).",
        ),
    ]

    bag_dir = LaunchConfiguration("bag_dir")
    bag_name = LaunchConfiguration("bag_name")
    storage = LaunchConfiguration("storage")
    compression = LaunchConfiguration("compression")
    include_experiment = LaunchConfiguration("include_experiment")

    # ---- Load topic list from YAML ----
    topics_yaml = os.path.join(pkg_share, "config", "rosbag_topics.yaml")
    with open(topics_yaml) as f:
        topics = yaml.safe_load(f).get("rosbag_topics", [])

    # ---- Build ros2 bag record command ----
    cmd = [
        "ros2", "bag", "record",
        "--storage", storage,
        "--topics",
    ] + topics

    # Output directory
    cmd += ["-o", [bag_dir, "/", bag_name]]

    # Optional compression preset
    # (LaunchConfiguration is resolved at runtime, so we pass it directly)
    cmd += ["--storage-preset-profile", compression]

    rosbag_record = ExecuteProcess(
        cmd=cmd,
        output="screen",
        name="rosbag_record",
    )

    info_msg = LogInfo(
        msg=["Recording ", str(len(topics)), " topics to bag in ", bag_dir],
    )

    # ---- Optionally include experiment node ----
    experiment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robin_core"), "launch", "experiment.launch.py",
            ])
        ]),
        condition=IfCondition(include_experiment),
    )

    return LaunchDescription(
        declared_arguments + [info_msg, rosbag_record, experiment]
    )
