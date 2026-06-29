#!/usr/bin/env python3
"""
welding_robin_sim.launch.py — lightweight no-hardware welding demo (no Gazebo/MoveIt).

Brings up just enough to drive the dashboard intent buttons and stream synthetic
telemetry ROS 2 -> DDS -> FIWARE:

  * robot_state_publisher with the full ROBIN description (UR10e + Fronius weld
    torch + Garmo laser profilometer + welding table + robot platform) so the
    whole cell renders in Lichtblick
  * foxglove_bridge (ws://localhost:8765) for the 3D view
  * the intent pipeline in mock mode: welding_robin_demo.launch.py use_simulation:=true
    (home/seam/recommendation/manual skills + welding_http_bridge + welding_supervisor)

The seam skill (mock) animates the arm via /joint_states_manual and publishes
ProcessTelemetry on /robin/telemetry; the home skill (mock) owns /joint_states. The
torch, profilometer, table and platform are fixed links in the URDF, so they follow
the arm (torch/profilometer) or stay put (table/platform) with no extra publishers.
No controllers, MoveIt, Gazebo, fake_opcua, or GPU are required (sim defaults to false
on the tooling macros, so no Gazebo plugins/sensors are loaded).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_share          = FindPackageShare('ur_robot_driver')
    description_share  = FindPackageShare('robin_description')
    demo_share         = FindPackageShare('welding_demo')

    # Full ROBIN cell description: UR10e + Fronius torch + Garmo profilometer +
    # welding table + platform. ur_fronius_garmo.urdf.xacro (robin_description) includes
    # ur.urdf.xacro and robin_tools.xacro, which pull the fronius / garmo / scene macros —
    # all self-contained in robin_description (urdf + meshes, plus the external UR
    # ur_description / ur_robot_driver xacros). The lite build selects just
    # `robin_description` (plus the welding_* + robin_interfaces packages); there is no
    # hardware/Gazebo code left to leave out.
    description_file = PathJoinSubstitution(
        [description_share, 'urdf', 'ur_fronius_garmo.urdf.xacro']
    )

    # Robot State Publisher via the upstream ur_rsp.launch.py (same as robin_main).
    # robot_ip is a dummy — rsp only builds/publishes the description, it does not
    # connect to a robot.
    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([ur_share, 'launch', 'ur_rsp.launch.py'])
        ),
        launch_arguments={
            'robot_ip': '0.0.0.0',
            'ur_type': 'ur10e',
            'description_file': description_file,
            'headless_mode': 'true',
        }.items(),
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,
        }],
    )

    # Intent pipeline (skills + http bridge + supervisor) in mock mode.
    intent_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([demo_share, 'launch', 'welding_robin_demo.launch.py'])
        ),
        launch_arguments={
            'use_simulation': 'true',
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        rsp,
        foxglove_bridge,
        intent_pipeline,
    ])
