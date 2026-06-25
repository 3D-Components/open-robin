#!/usr/bin/env python3
"""
WeldingRecommendationSkillNode: lifecycle action server for REQUEST_AI_RECOMMENDATION.

Triggered by the "Ask for a new AI recommendation" button in the ROBIN dashboard.
ros4hri concept: maps to ai_skills/action/RequestRecommendation.

Behaviour: calls the ROBIN process-intelligence API (``POST /ai-recommendation`` on
the Alert Engine) and returns the recommendation it produces. The skill first fetches
the inputs that endpoint needs for the requested mode:

  * geometry_driven  — GET /process/{id}/target  -> target_geometry
  * parameter_driven — GET /process/{id}         -> inputParams

It then POSTs to /ai-recommendation and returns the ``recommendation`` object as
JSON in the action result. Feedback advances through FETCHING -> PROCESSING -> COMPLETE.

The Alert Engine base URL is read from the ``ROBIN_API_URL`` environment variable
(default ``http://localhost:8001``); the welding container runs on the host network,
so the dashboard's published port is reachable directly.
"""
from __future__ import annotations

import json
import os
import threading
import urllib.error
import urllib.request

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from welding_msgs.action import RequestAIRecommendation


class WeldingRecommendationSkillNode(LifecycleNode):
    """Skill: requests an AI welding parameter recommendation from the ROBIN API."""

    ACTION_NAME      = 'welding_recommendation_skill/execute'
    DEFAULT_API_URL  = 'http://localhost:8001'
    GET_TIMEOUT_S    = 5.0
    POST_TIMEOUT_S   = 20.0

    def __init__(self):
        super().__init__('welding_recommendation_skill')
        self._action_server: ActionServer | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None
        self._api_base_url = os.environ.get('ROBIN_API_URL', self.DEFAULT_API_URL)

    # ── Lifecycle transitions ──────────────────────────────────────────────

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_recommendation_skill: configuring (api={self._api_base_url})'
        )
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

    # ── HTTP helpers ───────────────────────────────────────────────────────

    def _http_get_json(self, path: str) -> dict:
        url = f'{self._api_base_url}{path}'
        req = urllib.request.Request(url, method='GET')
        with urllib.request.urlopen(req, timeout=self.GET_TIMEOUT_S) as resp:
            return json.loads(resp.read().decode('utf-8'))

    def _http_post_json(self, path: str, body: dict) -> dict:
        url = f'{self._api_base_url}{path}'
        data = json.dumps(body).encode('utf-8')
        req = urllib.request.Request(
            url, data=data, method='POST',
            headers={'Content-Type': 'application/json'},
        )
        with urllib.request.urlopen(req, timeout=self.POST_TIMEOUT_S) as resp:
            return json.loads(resp.read().decode('utf-8'))

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
        process_id = goal_handle.request.process_id or 'ros_bridge'
        mode = goal_handle.request.mode or 'geometry_driven'
        self.get_logger().info(
            f'REQUEST_AI_RECOMMENDATION: fetching for process {process_id!r} mode={mode!r}'
        )

        feedback = RequestAIRecommendation.Feedback()
        result   = RequestAIRecommendation.Result()

        def fail(message: str) -> RequestAIRecommendation.Result:
            result.success             = False
            result.message             = message
            result.recommendation_json = '{}'
            return result

        # ── Phase 1: FETCHING — gather the inputs /ai-recommendation needs ──
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return fail('Recommendation request cancelled')
        feedback.progress_pct = 100.0 / 3.0
        feedback.phase        = 'FETCHING'
        goal_handle.publish_feedback(feedback)

        request_body: dict = {'process_id': process_id, 'mode': mode}
        try:
            if mode == 'geometry_driven':
                target = self._http_get_json(f'/process/{process_id}/target')
                target_geometry = (
                    target.get('target_geometry') if isinstance(target, dict) else None
                )
                if not target_geometry:
                    goal_handle.abort()
                    return fail(f'No geometry target set for process {process_id}')
                request_body['target_geometry'] = target_geometry
            elif mode == 'parameter_driven':
                entity = self._http_get_json(f'/process/{process_id}')
                input_params = None
                if isinstance(entity, dict) and isinstance(entity.get('inputParams'), dict):
                    input_params = entity['inputParams'].get('value')
                if not input_params:
                    goal_handle.abort()
                    return fail(
                        f'No input parameters available for process {process_id}')
                request_body['input_params'] = input_params
            else:
                goal_handle.abort()
                return fail(f'Unsupported mode: {mode}')
        except (urllib.error.URLError, OSError, ValueError) as exc:
            self.get_logger().error(f'REQUEST_AI_RECOMMENDATION: fetch failed: {exc}')
            goal_handle.abort()
            return fail(f'Failed to fetch process state: {exc}')

        # ── Phase 2: PROCESSING — call the ROBIN AI API ────────────────────
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return fail('Recommendation request cancelled')
        feedback.progress_pct = 200.0 / 3.0
        feedback.phase        = 'PROCESSING'
        goal_handle.publish_feedback(feedback)

        try:
            response = self._http_post_json('/ai-recommendation', request_body)
        except (urllib.error.URLError, OSError, ValueError) as exc:
            self.get_logger().error(f'REQUEST_AI_RECOMMENDATION: API call failed: {exc}')
            goal_handle.abort()
            return fail(f'AI recommendation request failed: {exc}')

        if not isinstance(response, dict) or response.get('status') != 'success':
            err = response.get('error') if isinstance(response, dict) else 'unknown error'
            self.get_logger().error(f'REQUEST_AI_RECOMMENDATION: API error: {err}')
            goal_handle.abort()
            return fail(f'AI recommendation error: {err}')

        recommendation = response.get('recommendation', {})

        # ── Phase 3: COMPLETE ──────────────────────────────────────────────
        feedback.progress_pct = 100.0
        feedback.phase        = 'COMPLETE'
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.success             = True
        result.message             = f'Recommendation ready for process {process_id}'
        result.recommendation_json = json.dumps(recommendation)
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
