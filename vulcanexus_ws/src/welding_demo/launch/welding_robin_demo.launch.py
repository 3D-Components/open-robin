#!/usr/bin/env python3
"""
welding_robin_demo.launch.py — master launch for the ROBIN welding HRI PoC.

Starts the full intent pipeline including the 3 new ROBIN dashboard skills
and the HTTP bridge so the React dashboard can fire intents.

Start order:
  t=0s   → 3 original skill nodes  (home, zone, seam)
           3 new skill nodes        (recommendation, manual, finetune)
           welding_http_bridge      (HTTP server on port 8766 for React UI)
  t=2s   → welding_supervisor      (waits for all 6 action servers to appear)

Intent flow:
  React dashboard button click
    └─► POST http://localhost:8766/intent  (welding_http_bridge)
          └─► /intents topic
                └─► welding_supervisor
                      ├─► START_PROCESS             → welding_seam_skill
                      ├─► REQUEST_AI_RECOMMENDATION → welding_recommendation_skill
                      ├─► MANUAL_ADJUST             → welding_manual_skill
                      └─► FINE_TUNE_MODEL           → welding_finetune_skill

To watch the intent stream:
    ros2 topic echo /intents

To test manually without the dashboard:
    curl -s -X POST http://localhost:8766/intent \\
         -H 'Content-Type: application/json' \\
         -d '{"intent": "START_PROCESS", "data": {"seam_id": "seam_01"}}' | jq
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # ── Original motion / weld skill servers ──────────────────────────────
    home_skill = Node(
        package='welding_home_skill',
        executable='welding_home_skill_node',
        name='welding_home_skill',
        output='screen',
        emulate_tty=True,
    )

    zone_skill = Node(
        package='welding_zone_skill',
        executable='welding_zone_skill_node',
        name='welding_zone_skill',
        output='screen',
        emulate_tty=True,
    )

    seam_skill = Node(
        package='welding_seam_skill',
        executable='welding_seam_skill_node',
        name='welding_seam_skill',
        output='screen',
        emulate_tty=True,
    )

    # ── ROBIN dashboard skill servers ─────────────────────────────────────
    recommendation_skill = Node(
        package='welding_recommendation_skill',
        executable='welding_recommendation_skill_node',
        name='welding_recommendation_skill',
        output='screen',
        emulate_tty=True,
    )

    manual_skill = Node(
        package='welding_manual_skill',
        executable='welding_manual_skill_node',
        name='welding_manual_skill',
        output='screen',
        emulate_tty=True,
    )

    finetune_skill = Node(
        package='welding_finetune_skill',
        executable='welding_finetune_skill_node',
        name='welding_finetune_skill',
        output='screen',
        emulate_tty=True,
    )

    # ── HTTP bridge (React dashboard → /intents) ──────────────────────────
    http_bridge = Node(
        package='welding_http_bridge',
        executable='welding_http_bridge_node',
        name='welding_http_bridge',
        output='screen',
        emulate_tty=True,
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
            ),
        ],
    )

    return LaunchDescription([
        # Original skills
        home_skill,
        zone_skill,
        seam_skill,
        # New ROBIN dashboard skills
        recommendation_skill,
        manual_skill,
        finetune_skill,
        # HTTP bridge for React UI
        http_bridge,
        # Intent router (delayed)
        supervisor,
    ])
