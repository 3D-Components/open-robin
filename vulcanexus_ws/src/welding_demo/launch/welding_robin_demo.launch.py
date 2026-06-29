#!/usr/bin/env python3
"""
welding_robin_demo.launch.py — master launch for the ROBIN welding HRI PoC.

Starts the full intent pipeline including the ROBIN dashboard skills
and the HTTP bridge so the React dashboard can fire intents.

Launch arguments:
  use_simulation  (default: true)  — true = internal mock mode (sleep timers, no
                                     real ROS2 commands); false = send real ROS2
                                     action goals to whatever hardware is present
                                     (Gazebo or real prototype).
  use_sim_time    (default: false) — set to true when running under Gazebo so all
                                     nodes synchronise to the /clock topic.

Start order:
  t=0s   → 2 original skill nodes  (home, seam)
           3 new skill nodes        (recommendation, manual, http_bridge)
           welding_http_bridge      (HTTP server on port 8766 for React UI)
  t=2s   → welding_supervisor      (waits for all 5 action servers to appear)

Intent flow:
  React dashboard button click
    └─► POST http://localhost:8766/intent  (welding_http_bridge)
          └─► /intents topic
                └─► welding_supervisor
                      ├─► START_PROCESS             → welding_seam_skill
                      ├─► REQUEST_AI_RECOMMENDATION → welding_recommendation_skill
                      └─► MANUAL_ADJUST             → welding_manual_skill

To watch the intent stream:
    ros2 topic echo /intents

To test manually without the dashboard:
    curl -s -X POST http://localhost:8766/intent \\
         -H 'Content-Type: application/json' \\
         -d '{"intent": "START_PROCESS", "data": {"seam_id": "seam_01"}}' | jq
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='true',
        description=(
            'true = internal mock mode (sleep timers, no real ROS2 commands); '
            'false = send real ROS2 action goals to hardware (Gazebo or prototype)'
        ),
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock topic for simulation time (set true under Gazebo)',
    )

    use_sim = LaunchConfiguration('use_simulation')
    sim_time = LaunchConfiguration('use_sim_time')

    # ── Original motion / weld skill servers ──────────────────────────────
    home_skill = Node(
        package='welding_home_skill',
        executable='welding_home_skill_node',
        name='welding_home_skill',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_simulation': use_sim, 'use_sim_time': sim_time}],
    )

    seam_skill = Node(
        package='welding_seam_skill',
        executable='welding_seam_skill_node',
        name='welding_seam_skill',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_simulation': use_sim, 'use_sim_time': sim_time}],
    )

    # ── ROBIN dashboard skill servers ─────────────────────────────────────
    recommendation_skill = Node(
        package='welding_recommendation_skill',
        executable='welding_recommendation_skill_node',
        name='welding_recommendation_skill',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_simulation': use_sim, 'use_sim_time': sim_time}],
    )

    manual_skill = Node(
        package='welding_manual_skill',
        executable='welding_manual_skill_node',
        name='welding_manual_skill',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_simulation': use_sim, 'use_sim_time': sim_time}],
    )

    # ── HTTP bridge (React dashboard → /intents) ──────────────────────────
    http_bridge = Node(
        package='welding_http_bridge',
        executable='welding_http_bridge_node',
        name='welding_http_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': sim_time}],
    )

    # ── Mission controller (delay 2 s so all skill action servers are up) ─
    supervisor = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='welding_supervisor',
                executable='welding_supervisor_node',
                name='welding_supervisor',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_simulation': use_sim, 'use_sim_time': sim_time}],
            ),
        ],
    )

    return LaunchDescription([
        # Arguments
        use_simulation_arg,
        use_sim_time_arg,
        # Original skills
        home_skill,
        seam_skill,
        # ROBIN dashboard skills
        recommendation_skill,
        manual_skill,
        # HTTP bridge for React UI
        http_bridge,
        # Intent router (delayed)
        supervisor,
    ])
