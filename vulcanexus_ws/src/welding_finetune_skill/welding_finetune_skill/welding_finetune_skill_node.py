#!/usr/bin/env python3
"""
WeldingFineTuneSkillNode: mock lifecycle action server for FINE_TUNE_MODEL.

Triggered by the "Fine-tune" button in the ROBIN dashboard.
ros4hri concept: maps to ml_skills/action/FineTuneModel.

Mock behaviour: progresses through COLLECTING → UPLOADING → QUEUED phases
over 4 s, then returns the number of samples submitted (mock: 42).

In production, replace the stub with a call to the ROBIN ML pipeline to
collect the current session telemetry and submit it for fine-tuning.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from welding_msgs.action import FineTuneModel


class WeldingFineTuneSkillNode(LifecycleNode):
    """Mock skill: collects session data and queues it for model fine-tuning (4 s)."""

    ACTION_NAME    = 'welding_finetune_skill/execute'
    # COLLECTING → UPLOADING → QUEUED = 3 phases × 1.33 s ≈ 4 s total
    PHASES         = ['COLLECTING', 'UPLOADING', 'QUEUED']
    PHASE_DURATION = 4.0 / len(PHASES)
    MOCK_SAMPLES   = 42   # stub value; production: count actual telemetry rows

    def __init__(self):
        super().__init__('welding_finetune_skill')
        self._action_server: ActionServer | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None

    # ── Lifecycle transitions ──────────────────────────────────────────────

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_finetune_skill: configuring')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_finetune_skill: activating — advertising {self.ACTION_NAME}'
        )
        self._action_server = ActionServer(
            self,
            FineTuneModel,
            self.ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_finetune_skill: deactivating')
        if self._action_server:
            self._action_server.destroy()
            self._action_server = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── Action server callbacks ────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f'FINE_TUNE_MODEL goal received: '
            f'process={goal_request.process_id!r} '
            f'dataset_tag={goal_request.dataset_tag!r}'
        )
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._current_goal_handle and self._current_goal_handle.is_active:
                self.get_logger().warning('Preempting previous fine-tune goal')
                self._current_goal_handle.abort()
            self._current_goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('FINE_TUNE_MODEL: cancel requested')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> FineTuneModel.Result:
        process_id  = goal_handle.request.process_id
        dataset_tag = goal_handle.request.dataset_tag
        self.get_logger().info(
            f'FINE_TUNE_MODEL: collecting data for process {process_id!r} '
            f'tag={dataset_tag!r}'
        )

        feedback = FineTuneModel.Feedback()
        result   = FineTuneModel.Result()

        for i, phase in enumerate(self.PHASES):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success           = False
                result.message           = 'Fine-tune submission cancelled'
                result.samples_submitted = 0
                return result

            feedback.progress_pct = (i + 1) / len(self.PHASES) * 100.0
            feedback.phase        = phase
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'FINE_TUNE_MODEL: {phase} [{i+1}/{len(self.PHASES)}] '
                f'→ {feedback.progress_pct:.0f}%'
            )
            time.sleep(self.PHASE_DURATION)

        goal_handle.succeed()
        result.success           = True
        result.message           = (
            f'{self.MOCK_SAMPLES} samples from process {process_id} '
            f'queued for fine-tuning (tag: {dataset_tag})'
        )
        result.samples_submitted = self.MOCK_SAMPLES
        self.get_logger().info(
            f'FINE_TUNE_MODEL: complete — {self.MOCK_SAMPLES} samples queued'
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WeldingFineTuneSkillNode()

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
