#!/usr/bin/env python3
"""
WeldingHomeSkillNode: mock lifecycle action server for MOVE_TO_HOME.

ros4hri concept: maps to motion_skills/action/ExecuteJointTrajectory.
Mock behaviour: publishes joint-by-joint progress over 3 s, then returns success.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from welding_msgs.action import MoveToHome


class WeldingHomeSkillNode(LifecycleNode):
    """Mock skill: moves robot to home position (simulated, 3 s)."""

    ACTION_NAME     = 'welding_home_skill/execute'
    MOCK_DURATION_S = 3.0
    JOINTS          = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    def __init__(self):
        super().__init__('welding_home_skill')
        self._action_server: ActionServer | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_home_skill: configuring')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_home_skill: activating — advertising {self.ACTION_NAME}'
        )
        self._action_server = ActionServer(
            self,
            MoveToHome,
            self.ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_home_skill: deactivating')
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
            f'MOVE_TO_HOME goal received: fast_speed={goal_request.use_fast_speed}'
        )
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._current_goal_handle and self._current_goal_handle.is_active:
                self.get_logger().warning('Preempting previous home goal')
                self._current_goal_handle.abort()
            self._current_goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('MOVE_TO_HOME: cancel requested')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> MoveToHome.Result:
        self.get_logger().info('MOVE_TO_HOME: executing ...')
        feedback = MoveToHome.Feedback()
        result   = MoveToHome.Result()
        step_duration = self.MOCK_DURATION_S / len(self.JOINTS)

        for i, joint in enumerate(self.JOINTS):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal cancelled by operator'
                return result

            feedback.progress_pct  = (i + 1) / len(self.JOINTS) * 100.0
            feedback.current_joint = joint
            goal_handle.publish_feedback(feedback)
            self.get_logger().debug(
                f'MOVE_TO_HOME: {joint} → {feedback.progress_pct:.0f}%'
            )
            time.sleep(step_duration)

        goal_handle.succeed()
        result.success = True
        result.message = 'Robot moved to home position successfully'
        self.get_logger().info('MOVE_TO_HOME: complete')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WeldingHomeSkillNode()

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
