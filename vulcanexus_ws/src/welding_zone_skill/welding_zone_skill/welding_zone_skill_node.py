#!/usr/bin/env python3
"""
WeldingZoneSkillNode: mock lifecycle action server for MOVE_TO_ZONE.

ros4hri concept: maps to navigation_skills/action/NavigateToWaypoint.
Mock behaviour: moves through APPROACHING → ALIGNING → ARRIVED phases over 4 s.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from welding_msgs.action import MoveToZone


class WeldingZoneSkillNode(LifecycleNode):
    """Mock skill: moves robot TCP to a named weld zone (simulated, 4 s)."""

    ACTION_NAME    = 'welding_zone_skill/execute'
    PHASES         = ['APPROACHING', 'ALIGNING', 'ARRIVED']
    PHASE_DURATION = 4.0 / 3

    def __init__(self):
        super().__init__('welding_zone_skill')
        self._action_server: ActionServer | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_zone_skill: configuring')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_zone_skill: activating — advertising {self.ACTION_NAME}'
        )
        self._action_server = ActionServer(
            self,
            MoveToZone,
            self.ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_zone_skill: deactivating')
        if self._action_server:
            self._action_server.destroy()
            self._action_server = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f'MOVE_TO_ZONE goal received: zone={goal_request.zone_id!r} '
            f'speed={goal_request.approach_speed}'
        )
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._current_goal_handle and self._current_goal_handle.is_active:
                self.get_logger().warning('Preempting previous zone goal')
                self._current_goal_handle.abort()
            self._current_goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('MOVE_TO_ZONE: cancel requested')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> MoveToZone.Result:
        zone = goal_handle.request.zone_id
        self.get_logger().info(f'MOVE_TO_ZONE: executing → Zone {zone}')
        feedback = MoveToZone.Feedback()
        result   = MoveToZone.Result()

        for i, phase in enumerate(self.PHASES):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal cancelled by operator'
                return result

            feedback.progress_pct = (i + 1) / len(self.PHASES) * 100.0
            feedback.phase        = phase
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'MOVE_TO_ZONE: {phase} → {feedback.progress_pct:.0f}%'
            )
            time.sleep(self.PHASE_DURATION)

        goal_handle.succeed()
        result.success    = True
        result.message    = f'Arrived at weld zone {zone}'
        result.final_zone = zone
        self.get_logger().info(f'MOVE_TO_ZONE: complete → Zone {zone}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WeldingZoneSkillNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    node.trigger_configure()
    node.trigger_activate()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
