#!/usr/bin/env python3
"""
WeldingManualSkillNode: lifecycle action server for MANUAL_ADJUST.

Triggered by the "Manual adjust" button in the ROBIN dashboard.
ros4hri concept: maps to process_skills/action/ManualParameterAdjust.

Modes (controlled by ROS2 parameter 'use_simulation', default True):
  use_simulation=True  — mock behaviour: VALIDATING → APPLYING → CONFIRMING over 2 s.
  use_simulation=False — real hardware: calls Fronius SetFloat32 services
                         (/fronius/set_current, /fronius/set_voltage,
                          /fronius/set_wire_speed) via WeldingCoordinator (OPC UA bridge).

Parameter name → Fronius service mapping:
  'current'    → /fronius/set_current    (Amperes)
  'voltage'    → /fronius/set_voltage    (Volts)
  'wire_speed' → /fronius/set_wire_speed (m/min)
  'weld_speed' → motion parameter, not a Fronius service (logs warning, no-op)
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from robin_interfaces.srv import SetFloat32
from welding_msgs.action import ManualAdjust

# Safe operating ranges per parameter (unit as declared in the action goal)
SAFE_RANGES: dict[str, tuple[float, float]] = {
    'weld_speed':  (1.0,  20.0),   # mm/s
    'wire_feed':   (1.0,  15.0),   # m/min
    'current':     (50.0, 400.0),  # A
    'voltage':     (10.0,  50.0),  # V
    'wire_speed':  (1.0,  20.0),   # m/min
}

# Maps parameter_name → (service_topic, description)
# None means "no Fronius service for this parameter"
FRONIUS_SERVICE_MAP: dict[str, tuple[str, str] | None] = {
    'current':    ('/fronius/set_current',    'welding current [A]'),
    'voltage':    ('/fronius/set_voltage',    'welding voltage [V]'),
    'wire_speed': ('/fronius/set_wire_speed', 'wire feed speed [m/min]'),
    'weld_speed': None,  # motion parameter — controlled by robot planner, not Fronius
}


class WeldingManualSkillNode(LifecycleNode):
    """
    Skill: validates and applies a manual process parameter adjustment.

    In simulation mode (default): runs a 2-second mock loop.
    In hardware mode: calls the appropriate Fronius SetFloat32 service.
    """

    ACTION_NAME    = 'welding_manual_skill/execute'
    PHASES         = ['VALIDATING', 'APPLYING', 'CONFIRMING']
    PHASE_DURATION = 2.0 / len(PHASES)

    def __init__(self):
        super().__init__('welding_manual_skill')
        self._action_server: ActionServer | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None
        self._use_simulation: bool = True
        # Service clients created on-demand in hardware mode
        self._service_clients: dict[str, rclpy.client.Client] = {}

    # ── Lifecycle transitions ─────────────────────────────────────────────────

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.declare_parameter('use_simulation', True)
        self._use_simulation = (
            self.get_parameter('use_simulation').get_parameter_value().bool_value
        )
        mode = 'SIMULATION' if self._use_simulation else 'HARDWARE'
        self.get_logger().info(f'welding_manual_skill: configuring [{mode} mode]')

        if not self._use_simulation:
            cb = ReentrantCallbackGroup()
            for param_name, mapping in FRONIUS_SERVICE_MAP.items():
                if mapping is not None:
                    topic, description = mapping
                    self._service_clients[param_name] = self.create_client(
                        SetFloat32, topic, callback_group=cb
                    )
                    self.get_logger().info(
                        f'welding_manual_skill: created client for {topic} ({description})'
                    )
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
        self._service_clients.clear()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── Action server callbacks ───────────────────────────────────────────────

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
        if self._use_simulation:
            return self._execute_mock(goal_handle)
        return self._execute_hardware(goal_handle)

    # ── Mock (simulation) execution ───────────────────────────────────────────

    def _execute_mock(self, goal_handle) -> ManualAdjust.Result:
        param_name = goal_handle.request.parameter_name
        new_value  = goal_handle.request.new_value
        unit       = goal_handle.request.unit
        self.get_logger().info(
            f'MANUAL_ADJUST [sim]: adjusting {param_name!r} → {new_value} {unit}'
        )

        lo, hi = SAFE_RANGES.get(param_name, (float('-inf'), float('inf')))
        applied = max(lo, min(hi, new_value))
        if applied != new_value:
            self.get_logger().warning(
                f'MANUAL_ADJUST [sim]: {param_name} {new_value} clamped to {applied} '
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
                f'MANUAL_ADJUST [sim]: {phase} [{i+1}/{len(self.PHASES)}] '
                f'→ {feedback.progress_pct:.0f}%'
            )
            time.sleep(self.PHASE_DURATION)

        goal_handle.succeed()
        result.success       = True
        result.message       = f'{param_name} set to {applied} {unit} (simulation)'
        result.applied_value = applied
        self.get_logger().info(
            f'MANUAL_ADJUST [sim]: complete — {param_name} = {applied} {unit}'
        )
        return result

    # ── Hardware execution (via Fronius SetFloat32 services) ──────────────────

    def _execute_hardware(self, goal_handle) -> ManualAdjust.Result:
        param_name = goal_handle.request.parameter_name
        new_value  = goal_handle.request.new_value
        unit       = goal_handle.request.unit

        result = ManualAdjust.Result()

        # Clamp to safe range
        lo, hi = SAFE_RANGES.get(param_name, (float('-inf'), float('inf')))
        applied = max(lo, min(hi, new_value))
        if applied != new_value:
            self.get_logger().warning(
                f'MANUAL_ADJUST [hw]: {param_name} {new_value} clamped to {applied} '
                f'(safe range [{lo}, {hi}] {unit})'
            )

        # Publish initial feedback
        feedback = ManualAdjust.Feedback()
        feedback.phase        = 'VALIDATING'
        feedback.progress_pct = 33.0
        goal_handle.publish_feedback(feedback)

        # Check if this parameter maps to a Fronius service
        mapping = FRONIUS_SERVICE_MAP.get(param_name)
        if mapping is None:
            if param_name == 'weld_speed':
                self.get_logger().warning(
                    'MANUAL_ADJUST [hw]: weld_speed is a motion parameter — '
                    'it cannot be set via Fronius services. '
                    'Use a trajectory replanning intent instead.'
                )
            else:
                self.get_logger().warning(
                    f'MANUAL_ADJUST [hw]: unknown parameter {param_name!r} — no Fronius service mapped'
                )
            # Succeed with a warning (parameter unknown but not a fatal error)
            goal_handle.succeed()
            result.success       = True
            result.message       = (
                f'{param_name} is not a Fronius-controlled parameter; no hardware change made'
            )
            result.applied_value = applied
            return result

        client = self._service_clients.get(param_name)
        if client is None:
            self.get_logger().error(
                f'MANUAL_ADJUST [hw]: no service client for {param_name!r}'
            )
            goal_handle.abort()
            result.success = False
            result.message = f'Internal error: no service client for {param_name!r}'
            result.applied_value = new_value
            return result

        # Wait for service
        feedback.phase        = 'APPLYING'
        feedback.progress_pct = 66.0
        goal_handle.publish_feedback(feedback)

        topic, description = mapping
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'MANUAL_ADJUST [hw]: service {topic} not available — '
                f'is robin_hardware_fronius running?'
            )
            goal_handle.abort()
            result.success       = False
            result.message       = f'Fronius service {topic} unavailable'
            result.applied_value = new_value
            return result

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success       = False
            result.message       = 'Manual adjust cancelled before service call'
            result.applied_value = new_value
            return result

        # Call Fronius service synchronously via threading.Event
        svc_request = SetFloat32.Request()
        svc_request.data = float(applied)

        done_event   = threading.Event()
        svc_response: dict = {}

        def on_response(future) -> None:
            try:
                svc_response['result'] = future.result()
            except Exception as exc:
                svc_response['error'] = str(exc)
            done_event.set()

        call_future = client.call_async(svc_request)
        call_future.add_done_callback(on_response)
        done_event.wait(timeout=10.0)

        # Confirm
        feedback.phase        = 'CONFIRMING'
        feedback.progress_pct = 100.0
        goal_handle.publish_feedback(feedback)

        if 'error' in svc_response:
            goal_handle.abort()
            result.success       = False
            result.message       = f'Service call failed: {svc_response["error"]}'
            result.applied_value = new_value
            self.get_logger().error(
                f'MANUAL_ADJUST [hw]: {topic} call failed — {svc_response["error"]}'
            )
            return result

        svc_result = svc_response.get('result')
        if svc_result and svc_result.success:
            goal_handle.succeed()
            result.success       = True
            result.message       = f'{param_name} ({description}) set to {applied} {unit}'
            result.applied_value = applied
            self.get_logger().info(
                f'MANUAL_ADJUST [hw]: complete — {description} = {applied} {unit}'
            )
        else:
            msg = svc_result.message if svc_result else 'No response received'
            goal_handle.abort()
            result.success       = False
            result.message       = f'Fronius service reported failure: {msg}'
            result.applied_value = new_value
            self.get_logger().error(
                f'MANUAL_ADJUST [hw]: {topic} returned failure — {msg}'
            )

        return result


def main(args=None):
    rclpy.init(args=args)
    node = WeldingManualSkillNode()

    executor = MultiThreadedExecutor(num_threads=4)
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
