#!/usr/bin/env python3
"""
WeldingManualSkillNode: mock lifecycle action server for MANUAL_ADJUST.

Triggered by the "Manual adjust" button in the ROBIN dashboard.
ros4hri concept: maps to process_skills/action/ManualParameterAdjust.

Mock behaviour: progresses through VALIDATING → APPLYING → CONFIRMING phases
over 2 s, then returns the applied value (clamped to a safe range).

In production, replace the stub with a call to the UR/process controller
service that actually writes the parameter to the hardware.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from welding_msgs.action import ManualAdjust


# Safe operating ranges per parameter (unit as declared in the action goal)
SAFE_RANGES: dict[str, tuple[float, float]] = {
    'weld_speed':  (1.0,  20.0),   # mm/s
    'wire_feed':   (1.0,  15.0),   # m/min
    'current':     (50.0, 400.0),  # A
    'voltage':     (10.0,  50.0),  # V
}


class WeldingManualSkillNode(LifecycleNode):
    """Mock skill: validates and applies a manual process parameter adjustment (2 s)."""

    ACTION_NAME    = 'welding_manual_skill/execute'
    # VALIDATING → APPLYING → CONFIRMING = 3 phases × 0.67 s ≈ 2 s total
    PHASES         = ['VALIDATING', 'APPLYING', 'CONFIRMING']
    PHASE_DURATION = 2.0 / len(PHASES)

    def __init__(self):
        super().__init__('welding_manual_skill')
        self._action_server: ActionServer | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None

    # ── Lifecycle transitions ──────────────────────────────────────────────

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_manual_skill: configuring')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_manual_skill: activating — advertising {self.ACTION_NAME}'
        )
        self._action_server = ActionServer(
            self,
            ManualAdjust,
            self.ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_manual_skill: deactivating')
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
            f'MANUAL_ADJUST goal received: '
            f'param={goal_request.parameter_name!r} '
            f'value={goal_request.new_value} {goal_request.unit}'
        )
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._current_goal_handle and self._current_goal_handle.is_active:
                self.get_logger().warning('Preempting previous manual adjust goal')
                self._current_goal_handle.abort()
            self._current_goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('MANUAL_ADJUST: cancel requested')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> ManualAdjust.Result:
        param_name = goal_handle.request.parameter_name
        new_value  = goal_handle.request.new_value
        unit       = goal_handle.request.unit
        self.get_logger().info(
            f'MANUAL_ADJUST: adjusting {param_name!r} → {new_value} {unit}'
        )

        # Clamp to safe range if known
        lo, hi = SAFE_RANGES.get(param_name, (float('-inf'), float('inf')))
        applied = max(lo, min(hi, new_value))
        if applied != new_value:
            self.get_logger().warning(
                f'MANUAL_ADJUST: {param_name} {new_value} clamped to {applied} '
                f'(safe range [{lo}, {hi}] {unit})'
            )

        feedback = ManualAdjust.Feedback()
        result   = ManualAdjust.Result()

        for i, phase in enumerate(self.PHASES):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success       = False
                result.message       = 'Manual adjust cancelled'
                result.applied_value = new_value
                return result

            feedback.progress_pct = (i + 1) / len(self.PHASES) * 100.0
            feedback.phase        = phase
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'MANUAL_ADJUST: {phase} [{i+1}/{len(self.PHASES)}] '
                f'→ {feedback.progress_pct:.0f}%'
            )
            time.sleep(self.PHASE_DURATION)

        goal_handle.succeed()
        result.success       = True
        result.message       = f'{param_name} set to {applied} {unit}'
        result.applied_value = applied
        self.get_logger().info(
            f'MANUAL_ADJUST: complete — {param_name} = {applied} {unit}'
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WeldingManualSkillNode()

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
