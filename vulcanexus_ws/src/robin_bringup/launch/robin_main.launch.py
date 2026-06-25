"""ROBIN main launch — event-driven staging via OnProcessExit.

Stages:
  IMMEDIATE   Hardware drivers + independent application nodes
  GATED       MoveIt, planner, experiment, skills (after controller spawners exit)

The trajectory-controller spawner is a short-lived process that exits
with code 0 once the controller is loaded and activated.  OnProcessExit
on that spawner gates all downstream nodes that depend on a working
controller manager, eliminating blind timer delays.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def _is_truthy(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _build_staged_launch(context):
    """Build the full launch graph with event-driven staging."""
    # ── Resolve arguments ────────────────────────────────────────────
    is_sim = _is_truthy(
        context.launch_configurations.get("use_sim", "false")
    )
    robot_ip = LaunchConfiguration("robot_ip")
    sensor_ip = LaunchConfiguration("sensor_ip")
    beads_config = LaunchConfiguration("beads_config")
    plates_config = LaunchConfiguration("plates_config")
    ctwd_file = LaunchConfiguration("ctwd_file")
    use_sim = LaunchConfiguration("use_sim")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_foxglove = LaunchConfiguration("launch_foxglove")

    # ── Shared paths ─────────────────────────────────────────────────
    bringup_share = FindPackageShare("robin_bringup")
    rviz_config = PathJoinSubstitution(
        [bringup_share, "rviz", "view_scene.rviz"]
    )
    progression_yaml = PathJoinSubstitution(
        [FindPackageShare("robin_core"), "config", "progression.yaml"]
    )
    weld_data_yaml = PathJoinSubstitution(
        [FindPackageShare("robin_core"), "config", "weld_data.yaml"]
    )
    garmo_config_yaml = PathJoinSubstitution(
        [FindPackageShare("robin_hardware_garmo"), "config", "garmo_config.yaml"]
    )
    rqt_perspective = PathJoinSubstitution(
        [FindPackageShare("robin_rqt"), "config", "Default.perspective"]
    )

    # ════════════════════════════════════════════════════════════════
    # STAGE 1 (IMMEDIATE): Hardware drivers
    # ════════════════════════════════════════════════════════════════

    hw_nodes = []

    if not is_sim:
        # ── Real hardware: inline UR control for OnProcessExit access ──
        ur_share = FindPackageShare("ur_robot_driver")
        robin_ur_share = FindPackageShare("robin_hardware_ur")

        controllers_file = PathJoinSubstitution(
            [robin_ur_share, "config", "ur10e_controllers.yaml"]
        )
        description_file = PathJoinSubstitution(
            [bringup_share, "urdf", "ur_fronius_garmo.urdf.xacro"]
        )
        update_rate_config = PathJoinSubstitution(
            [ur_share, "config", "ur10e_update_rate.yaml"]
        )

        # Set launch configurations needed by ParameterFile substitutions
        # in the controllers YAML ($(var tf_prefix) etc.) and by
        # ur_rsp.launch.py.  These were previously declared by the
        # upstream ur_control.launch.py that we no longer include.
        set_tf_prefix = SetLaunchConfiguration("tf_prefix", "")

        # Controller manager
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                update_rate_config,
                ParameterFile(controllers_file, allow_substs=True),
            ],
            output="screen",
        )

        # Robot State Publisher (from upstream ur_rsp.launch.py)
        rsp = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [ur_share, "launch", "ur_rsp.launch.py"]
                )
            ),
            launch_arguments={
                "robot_ip": robot_ip,
                "ur_type": "ur10e",
                "description_file": description_file,
                "headless_mode": "true",
            }.items(),
        )

        # Dashboard client
        dashboard_client = IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [ur_share, "launch", "ur_dashboard_client.launch.py"]
                )
            ),
            launch_arguments={"robot_ip": robot_ip}.items(),
        )

        # UR-specific helper nodes
        robot_state_helper = Node(
            package="ur_robot_driver",
            executable="robot_state_helper",
            name="ur_robot_state_helper",
            output="screen",
            parameters=[
                {"headless_mode": True},
                {"robot_ip": robot_ip},
            ],
        )
        urscript_interface = Node(
            package="ur_robot_driver",
            executable="urscript_interface",
            parameters=[{"robot_ip": robot_ip}],
            output="screen",
        )
        controller_stopper = Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            name="controller_stopper",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"headless_mode": True},
                {"joint_controller_active": True},
                {
                    "consistent_controllers": [
                        "io_and_status_controller",
                        "force_torque_sensor_broadcaster",
                        "joint_state_broadcaster",
                        "speed_scaling_state_broadcaster",
                        "tcp_pose_broadcaster",
                        "ur_configuration_controller",
                        "forward_velocity_controller",
                    ]
                },
            ],
        )
        trajectory_until = Node(
            package="ur_robot_driver",
            executable="trajectory_until_node",
            name="trajectory_until_node",
            output="screen",
            parameters=[
                {"motion_controller": "scaled_joint_trajectory_controller"},
            ],
        )

        # Controller spawners — we need Python references for OnProcessExit
        spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
        active_spawner = Node(
            package="controller_manager",
            executable="spawner",
            name="active_controller_spawner",
            arguments=[
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", spawner_timeout,
                "joint_state_broadcaster",
                "io_and_status_controller",
                "speed_scaling_state_broadcaster",
                "force_torque_sensor_broadcaster",
                "tcp_pose_broadcaster",
                "ur_configuration_controller",
                "scaled_joint_trajectory_controller",
            ],
        )
        inactive_spawner = Node(
            package="controller_manager",
            executable="spawner",
            name="inactive_controller_spawner",
            arguments=[
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", spawner_timeout,
                "--inactive",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "forward_effort_controller",
                "force_mode_controller",
                "passthrough_trajectory_controller",
                "freedrive_mode_controller",
                "tool_contact_controller",
            ],
        )

        # OPC UA bridge (independent, starts immediately)
        opcua_bridge = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("robin_hardware_opcua"),
                        "launch",
                        "opcua_bridge.launch.xml",
                    ]
                )
            ),
        )

        gate_process = active_spawner
        hw_nodes = [
            set_tf_prefix,
            control_node, rsp, dashboard_client,
            robot_state_helper, urscript_interface,
            controller_stopper, trajectory_until,
            active_spawner,
            opcua_bridge,
        ]
    else:
        # ── Simulation: inline Gazebo + controller spawning ──────────
        sim_share = FindPackageShare("robin_simulation")
        sim_controllers = PathJoinSubstitution(
            [sim_share, "config", "sim_controllers.yaml"]
        )
        world_file = PathJoinSubstitution(
            [sim_share, "worlds", "welding_cell.sdf"]
        )
        sim_description_file = PathJoinSubstitution(
            [bringup_share, "urdf", "sim_ur_fronius_garmo.urdf.xacro"]
        )

        # Environment for Gazebo mesh/plugin discovery
        gz_resource_path = SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [
                PathJoinSubstitution(
                    [FindPackageShare("robin_hardware_fronius"), ".."]
                ),
                ":",
                PathJoinSubstitution(
                    [FindPackageShare("robin_hardware_garmo"), ".."]
                ),
                ":",
                PathJoinSubstitution([bringup_share, ".."]),
                ":/opt/ros/jazzy/share",
            ],
        )
        gz_plugin_path = SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            PathJoinSubstitution(
                [FindPackageShare("robin_sim_gz"), "..", "..", "lib"]
            ),
        )

        # Robot description via xacro
        robot_description_content = Command(
            [
                FindExecutable(name="xacro"),
                " ",
                sim_description_file,
                " name:=ur ur_type:=ur10e",
                " simulation_controllers:=",
                sim_controllers,
                " sim:=true",
            ]
        )

        sim_rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {"use_sim_time": True},
                {
                    "robot_description": ParameterValue(
                        robot_description_content, value_type=str
                    )
                },
            ],
        )

        # Gazebo simulator
        gz_gui_arg = context.launch_configurations.get("gazebo_gui", "true")
        gz_args = (
            f" -r -v 4 " if _is_truthy(gz_gui_arg) else f" -s -r -v 4 "
        )
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ),
            launch_arguments={
                "gz_args": [gz_args, world_file],
            }.items(),
        )

        # Spawn robot
        gz_spawn = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-string", robot_description_content,
                "-name", "ur",
                "-allow_renaming", "true",
            ],
        )

        # Bridges
        clock_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ],
        )
        garmo_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="garmo_bridge",
            output="screen",
            arguments=[
                "/garmo_laser/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            ],
        )
        robin_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="robin_bridge",
            output="screen",
            arguments=[
                "/robin/weld_active@std_msgs/msg/Bool]gz.msgs.Boolean",
                "/robin/weld_current@std_msgs/msg/Float64]gz.msgs.Double",
                "/robin/weld_wfs@std_msgs/msg/Float64]gz.msgs.Double",
                "/robin/weld_voltage@std_msgs/msg/Float64]gz.msgs.Double",
                "/robin/weld_travel_speed@std_msgs/msg/Float64]gz.msgs.Double",
                "/robin/weld_bead_id@std_msgs/msg/String]gz.msgs.StringMsg",
                "/robin/clear_beads@std_msgs/msg/Bool]gz.msgs.Boolean",
            ],
        )

        # Sim controller spawners — need Python references for gating
        sim_jsb_spawner = Node(
            package="controller_manager",
            executable="spawner",
            name="sim_jsb_spawner",
            arguments=[
                "joint_state_broadcaster",
                "-c", "/controller_manager",
            ],
        )
        sim_traj_spawner = Node(
            package="controller_manager",
            executable="spawner",
            name="sim_traj_spawner",
            arguments=[
                "joint_trajectory_controller",
                "-c", "/controller_manager",
            ],
        )
        sim_vel_spawner = Node(
            package="controller_manager",
            executable="spawner",
            name="sim_vel_spawner",
            arguments=[
                "forward_velocity_controller",
                "-c", "/controller_manager",
                "--inactive",
            ],
        )

        # Fake OPC UA bridge
        fake_opcua = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [sim_share, "launch", "fake_opcua_bridge.launch.xml"]
                )
            ),
        )

        gate_process = sim_traj_spawner
        hw_nodes = [
            gz_resource_path, gz_plugin_path,
            sim_rsp, gz_sim, gz_spawn,
            clock_bridge, garmo_bridge, robin_bridge,
            sim_jsb_spawner, sim_traj_spawner, sim_vel_spawner,
            fake_opcua,
        ]

    # ════════════════════════════════════════════════════════════════
    # INDEPENDENT NODES — start immediately (no controller dependency)
    # ════════════════════════════════════════════════════════════════

    tcp_manager_node = Node(
        package="robin_core", executable="tcp_manager_node",
        name="tcp_manager_node", output="screen",
        parameters=[{
            "default_ctwd": 0.015,
            "default_mode": "welding",
            "ctwd_file": ctwd_file,
            "use_sim_time": use_sim,
        }],
    )

    progression_node = Node(
        package="robin_core", executable="progression_node",
        name="progression_node", output="screen",
        parameters=[progression_yaml, {"use_sim_time": use_sim}],
    )

    weld_data_node = Node(
        package="robin_core", executable="weld_data_node",
        name="weld_data_node", output="screen",
        parameters=[weld_data_yaml, {"use_sim_time": use_sim}],
    )

    weld_profile_processor = Node(
        package="robin_core", executable="process_data_node",
        name="weld_profile_processor", output="screen",
        parameters=[{"use_sim_time": use_sim}],
    )

    telemetry_aggregator = Node(
        package="robin_core", executable="telemetry_aggregator_node",
        name="telemetry_aggregator", output="screen",
        parameters=[{"publish_rate": 10.0, "use_sim_time": use_sim}],
    )

    plate_markers = Node(
        package="robin_core", executable="plate_markers_node",
        name="plate_markers", output="screen",
        parameters=[{
            "plates_config": plates_config,
            "beads_config": beads_config,
            "use_sim_time": use_sim,
        }],
    )

    welding_coordinator = Node(
        package="robin_hardware_fronius", executable="welding_coordinator",
        name="welding_coordinator", output="screen",
        parameters=[{"use_sim_time": use_sim}],
    )

    foxglove_bridge = Node(
        package="foxglove_bridge", executable="foxglove_bridge",
        name="foxglove_bridge", output="log",
        arguments=["--ros-args", "--log-level", "fatal"],
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",
            "send_buffer_limit": 10000000,
        }],
        condition=IfCondition(launch_foxglove),
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2",
        name="rviz2", output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim}],
        condition=IfCondition(launch_rviz),
    )

    operator_panel = Node(
        package="rqt_gui", executable="rqt_gui",
        name="robin_operator_panel", output="screen",
        arguments=["--perspective-file", rqt_perspective],
    )

    independent = [
        tcp_manager_node,
        progression_node,
        weld_data_node,
        weld_profile_processor,
        telemetry_aggregator,
        plate_markers,
        welding_coordinator,
        foxglove_bridge,
        rviz_node,
        operator_panel,
    ]

    # ════════════════════════════════════════════════════════════════
    # STAGE 2 (GATED): MoveIt + planner + experiment + skills
    # Fires when the trajectory controller spawner exits successfully.
    # ════════════════════════════════════════════════════════════════

    # Build Stage 2 entities now; they're captured in the closure below
    moveit_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robin_core"), "launch",
                 "moveit_planner.launch.py"]
            )
        ),
        launch_arguments={"use_sim": use_sim}.items(),
    )

    experiment_node = Node(
        package="robin_core", executable="experiment_node",
        name="robin_experiment", output="screen",
        parameters=[{
            "default_bead_pitch": 0.020,
            "default_margin_x": 0.015,
            "default_margin_y": 0.015,
            "default_experiment_spacing_x": 0.030,
            "default_experiment_spacing_y": 0.030,
            "default_experiment_bead_length": 0.100,
            "inter_bead_clearance_height": 0.250,
            "base_frame": "base_link",
            "execute_bead_timeout": 120.0,
            "beads_config": beads_config,
            "use_sim_time": use_sim,
        }],
    )

    # Sensor (real vs sim)
    if not is_sim:
        garmo_sensor = Node(
            package="robin_hardware_garmo", executable="sensor_node",
            name="garmo_sensor_node", output="screen",
            parameters=[garmo_config_yaml, {"sensor_ip": sensor_ip}],
        )
    else:
        garmo_sensor = Node(
            package="robin_simulation", executable="sim_garmo_sensor",
            name="garmo_sensor_node", output="screen",
            parameters=[{"use_sim_time": True}],
        )

    sim_clock_monitor = Node(
        package="robin_core", executable="sim_clock_monitor_node",
        name="sim_clock_monitor", output="screen",
        parameters=[{"use_sim_time": True}],
    ) if is_sim else None

    lifecycle_manager_sensor = Node(
        package="robin_bringup", executable="lifecycle_manager",
        name="lifecycle_manager_sensor", output="screen",
        parameters=[{
            "node_names": ["garmo_sensor_node"],
            "configure_timeout": 15.0,
            "max_retries": 5,
            "retry_delay": 3.0,
            "activate": False,
            "monitor_period": 5.0,
            "use_sim_time": use_sim,
        }],
    )

    intent_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("welding_demo"), "launch",
                 "welding_robin_demo.launch.py"]
            )
        ),
        launch_arguments={
            "use_simulation": "false",
            "use_sim_time": use_sim,
        }.items(),
    )

    # ════════════════════════════════════════════════════════════════
    # STAGE 2 (on active spawner exit): inactive controller spawner
    # Runs alone before the heavy Stage 3 nodes to keep the DDS
    # participant count low (~20 vs ~30+).  Loads ~8 controllers
    # in ~200ms then exits, triggering Stage 3.
    # ════════════════════════════════════════════════════════════════

    stage3_entities = [
        LogInfo(msg="All controllers loaded — launching MoveIt + application nodes"),
        moveit_planner,
        experiment_node,
        garmo_sensor,
        lifecycle_manager_sensor,
        intent_pipeline,
    ]
    if sim_clock_monitor is not None:
        stage3_entities.append(sim_clock_monitor)

    if not is_sim:
        def _on_inactive_spawner_done(event, context):
            if event.returncode != 0:
                return [
                    LogInfo(
                        msg=f"WARNING: Inactive controller spawning failed "
                        f"(exit code {event.returncode}) — "
                        f"forward_velocity_controller may be unavailable"
                    ),
                ] + stage3_entities
            return stage3_entities

        stage3_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=inactive_spawner,
                on_exit=_on_inactive_spawner_done,
            )
        )

        def _on_active_spawner_done(event, context):
            if event.returncode != 0:
                return [
                    LogInfo(
                        msg=f"FATAL: Controller spawning failed "
                        f"(exit code {event.returncode})"
                    ),
                    Shutdown(reason="Controller spawn failed"),
                ]
            return [
                LogInfo(msg="Active controllers ready — loading inactive controllers"),
                inactive_spawner,
                stage3_handler,
            ]
    else:
        def _on_active_spawner_done(event, context):
            if event.returncode != 0:
                return [
                    LogInfo(
                        msg=f"FATAL: Controller spawning failed "
                        f"(exit code {event.returncode})"
                    ),
                    Shutdown(reason="Controller spawn failed"),
                ]
            return [
                LogInfo(msg="Sim controllers ready — launching MoveIt + application nodes"),
            ] + stage3_entities

    stage2_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gate_process,
            on_exit=_on_active_spawner_done,
        )
    )

    return hw_nodes + independent + [stage2_handler]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.101"),
        DeclareLaunchArgument("sensor_ip", default_value="192.168.1.212"),
        DeclareLaunchArgument(
            "beads_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robin_bringup"), "config", "beads.json"]
            ),
        ),
        DeclareLaunchArgument(
            "plates_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robin_bringup"), "config", "plates.json"]
            ),
        ),
        DeclareLaunchArgument("ctwd_file", default_value=""),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("launch_foxglove", default_value="true"),
        DeclareLaunchArgument(
            "controller_spawner_timeout", default_value="30",
        ),
    ]

    return LaunchDescription(
        args + [OpaqueFunction(function=_build_staged_launch)]
    )
