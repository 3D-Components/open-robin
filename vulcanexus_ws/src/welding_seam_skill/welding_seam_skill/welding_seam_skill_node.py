#!/usr/bin/env python3
"""
WeldingSeamSkillNode: mock lifecycle action server for EXECUTE_SEAM.

ros4hri concept: maps to motion_skills/action/ExecuteCartesianTrajectory.
Mock behaviour: progresses through IGNITING → WELDING (×3) → FINISHING
phases over 6 s while publishing weld speed feedback.
Also handles the START_PROCESS intent which the supervisor routes here.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from welding_msgs.action import ExecuteSeam


class WeldingSeamSkillNode(LifecycleNode):
    """Mock skill: executes weld seam Cartesian trajectory (simulated, 6 s)."""

    ACTION_NAME         = 'welding_seam_skill/execute'
    PHASES              = ['IGNITING', 'WELDING', 'WELDING', 'WELDING', 'FINISHING']
    PHASE_DURATION      = 6.0 / 5
    MOCK_SEAM_LENGTH_MM = 150.0

    def __init__(self):
        super().__init__('welding_seam_skill')
        self._action_server: ActionServer | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_seam_skill: configuring')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_seam_skill: activating — advertising {self.ACTION_NAME}'
        )
        self._action_server = ActionServer(
            self,
            ExecuteSeam,
            self.ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_seam_skill: deactivating')
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
            f'EXECUTE_SEAM goal received: seam={goal_request.seam_id!r} '
            f'speed={goal_request.weld_speed} mm/s '
            f'wire_feed={goal_request.wire_feed_rate} m/min'
        )
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._current_goal_handle and self._current_goal_handle.is_active:
                self.get_logger().warning('Preempting previous seam goal')
                self._current_goal_handle.abort()
            self._current_goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('EXECUTE_SEAM: cancel requested')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> ExecuteSeam.Result:
        seam_id    = goal_handle.request.seam_id
        weld_speed = goal_handle.request.weld_speed
        self.get_logger().info(
            f'EXECUTE_SEAM: starting seam {seam_id!r} at {weld_speed} mm/s'
        )

        feedback = ExecuteSeam.Feedback()
        result   = ExecuteSeam.Result()
        welded_length    = 0.0
        length_per_phase = self.MOCK_SEAM_LENGTH_MM / len(self.PHASES)

        for i, phase in enumerate(self.PHASES):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success        = False
                result.message        = 'Weld cancelled — ESTOP or operator request'
                result.seam_length_mm = welded_length
                self.get_logger().warning(
                    f'EXECUTE_SEAM: cancelled at {welded_length:.1f} mm (phase: {phase})'
                )
                return result

            welded_length += length_per_phase
            feedback.progress_pct  = (i + 1) / len(self.PHASES) * 100.0
            feedback.phase         = phase
            feedback.current_speed = weld_speed if phase == 'WELDING' else 0.0
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'EXECUTE_SEAM: {phase} [{i+1}/{len(self.PHASES)}] '
                f'→ {feedback.progress_pct:.0f}% | {welded_length:.1f} mm welded'
            )
            time.sleep(self.PHASE_DURATION)

        goal_handle.succeed()
        result.success        = True
        result.message        = f'Seam {seam_id} welded successfully'
        result.seam_length_mm = self.MOCK_SEAM_LENGTH_MM
        self.get_logger().info(
            f'EXECUTE_SEAM: complete — {self.MOCK_SEAM_LENGTH_MM} mm welded'
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WeldingSeamSkillNode()

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
