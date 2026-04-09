#!/usr/bin/env python3
"""
WeldingRecommendationSkillNode: mock lifecycle action server for REQUEST_AI_RECOMMENDATION.

Triggered by the "Ask for a new AI recommendation" button in the ROBIN dashboard.
ros4hri concept: maps to ai_skills/action/RequestRecommendation.

Mock behaviour: progresses through FETCHING → PROCESSING → COMPLETE phases over
3 s while publishing progress feedback, then returns a hardcoded recommendation JSON.

In production, replace the stub response with a call to the ROBIN AI API
(POST /ai-recommendation on the robin backend).
"""
from __future__ import annotations

import json
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from welding_msgs.action import RequestAIRecommendation


class WeldingRecommendationSkillNode(LifecycleNode):
    """Mock skill: fetches an AI welding parameter recommendation (3 s)."""

    ACTION_NAME    = 'welding_recommendation_skill/execute'
    # FETCHING → PROCESSING → COMPLETE = 3 phases × 1 s = 3 s total
    PHASES         = ['FETCHING', 'PROCESSING', 'COMPLETE']
    PHASE_DURATION = 3.0 / len(PHASES)

    # Stub recommendation returned in mock mode.
    # Production: call robin backend POST /ai-recommendation and parse the response.
    MOCK_RECOMMENDATION = {
        'weld_speed': 4.8,
        'wire_feed': 4.2,
        'current': 210.0,
        'voltage': 24.5,
        'confidence': 0.87,
        'source': 'mock_model_v1',
    }

    def __init__(self):
        super().__init__('welding_recommendation_skill')
        self._action_server: ActionServer | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None

    # ── Lifecycle transitions ──────────────────────────────────────────────

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_recommendation_skill: configuring')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_recommendation_skill: activating — advertising {self.ACTION_NAME}'
        )
        self._action_server = ActionServer(
            self,
            RequestAIRecommendation,
            self.ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_recommendation_skill: deactivating')
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
            f'REQUEST_AI_RECOMMENDATION goal received: '
            f'process={goal_request.process_id!r} mode={goal_request.mode!r}'
        )
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._current_goal_handle and self._current_goal_handle.is_active:
                self.get_logger().warning('Preempting previous recommendation goal')
                self._current_goal_handle.abort()
            self._current_goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('REQUEST_AI_RECOMMENDATION: cancel requested')
        return CancelResponse.ACCEPT

    def _execute_callback(
        self, goal_handle
    ) -> RequestAIRecommendation.Result:
        process_id = goal_handle.request.process_id
        mode = goal_handle.request.mode
        self.get_logger().info(
            f'REQUEST_AI_RECOMMENDATION: fetching for process {process_id!r} mode={mode!r}'
        )

        feedback = RequestAIRecommendation.Feedback()
        result   = RequestAIRecommendation.Result()

        for i, phase in enumerate(self.PHASES):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success             = False
                result.message             = 'Recommendation request cancelled'
                result.recommendation_json = '{}'
                return result

            feedback.progress_pct = (i + 1) / len(self.PHASES) * 100.0
            feedback.phase        = phase
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'REQUEST_AI_RECOMMENDATION: {phase} [{i+1}/{len(self.PHASES)}] '
                f'→ {feedback.progress_pct:.0f}%'
            )
            time.sleep(self.PHASE_DURATION)

        goal_handle.succeed()
        result.success             = True
        result.message             = f'Recommendation ready for process {process_id}'
        result.recommendation_json = json.dumps(self.MOCK_RECOMMENDATION)
        self.get_logger().info(
            f'REQUEST_AI_RECOMMENDATION: complete — {result.recommendation_json}'
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WeldingRecommendationSkillNode()

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
