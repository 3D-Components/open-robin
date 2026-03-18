#!/usr/bin/env python3
"""
WeldingSeamSkillNode: lifecycle action server for EXECUTE_SEAM / START_PROCESS.

ros4hri concept: maps to motion_skills/action/ExecuteCartesianTrajectory.

Modes (controlled by ROS2 parameter 'use_simulation', default True):
  use_simulation=True  — mock behaviour: IGNITING → WELDING×3 → FINISHING over 6 s.
  use_simulation=False — real hardware: delegates to /weld_experiment action on
                         RobinPlanner (robin_moveit_control), which sequences
                         MoveItPy PTP/LIN motions + WeldingCoordinator arc start/stop.

Seam coordinates are loaded from the 'seam_registry' parameter (YAML dict keyed by
seam_id).  A built-in default for 'seam_01' is provided so the system runs out of
the box; replace with actual robot-frame coordinates for production.
"""
from __future__ import annotations

import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from robin_interfaces.action import WeldExperiment
from robin_interfaces.msg import WeldBead
from welding_msgs.action import ExecuteSeam

# ── Default seam registry ────────────────────────────────────────────────────
# Coordinates are in the robot base frame (metres).
# Override via ROS2 parameter 'seam_registry' (YAML/JSON string) or by calling
# the node with a config file that sets the parameter.
DEFAULT_SEAM_REGISTRY: dict[str, dict[str, tuple]] = {
    'seam_01': {
        'start': (0.45, -0.20, 0.05),
        'end':   (0.45,  0.20, 0.05),
    },
}

# Welding arc defaults used when the goal doesn't carry electrical parameters
DEFAULT_CURRENT_A  = 220.0   # Amperes
DEFAULT_VOLTAGE_V  =  26.0   # Volts
DEFAULT_STICKOUT_M =   0.015  # 15 mm CTWD


class WeldingSeamSkillNode(LifecycleNode):
    """
    Skill: executes a weld seam Cartesian trajectory.

    In simulation mode (default): runs a 6-second mock loop.
    In hardware mode: delegates to /weld_experiment on RobinPlanner.
    """

    ACTION_NAME         = 'welding_seam_skill/execute'
    PLANNER_ACTION_NAME = 'weld_experiment'

    # Mock-mode constants
    PHASES              = ['IGNITING', 'WELDING', 'WELDING', 'WELDING', 'FINISHING']
    PHASE_DURATION      = 6.0 / 5
    MOCK_SEAM_LENGTH_MM = 150.0

    def __init__(self):
        super().__init__('welding_seam_skill')
        self._action_server: ActionServer | None = None
        self._planner_client: ActionClient | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None
        self._use_simulation: bool = True
        self._seam_registry: dict = dict(DEFAULT_SEAM_REGISTRY)

    # ── Lifecycle transitions ─────────────────────────────────────────────────

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.declare_parameter('use_simulation', True)
        self._use_simulation = (
            self.get_parameter('use_simulation').get_parameter_value().bool_value
        )
        mode = 'SIMULATION' if self._use_simulation else 'HARDWARE'
        self.get_logger().info(f'welding_seam_skill: configuring [{mode} mode]')

        if not self._use_simulation:
            self._planner_client = ActionClient(
                self,
                WeldExperiment,
                self.PLANNER_ACTION_NAME,
                callback_group=ReentrantCallbackGroup(),
            )
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
        if self._planner_client:
            self._planner_client.destroy()
            self._planner_client = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── Action server callbacks ───────────────────────────────────────────────

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
        if self._use_simulation:
            return self._execute_mock(goal_handle)
        return self._execute_hardware(goal_handle)

    # ── Mock (simulation) execution ───────────────────────────────────────────

    def _execute_mock(self, goal_handle) -> ExecuteSeam.Result:
        seam_id    = goal_handle.request.seam_id
        weld_speed = goal_handle.request.weld_speed
        self.get_logger().info(
            f'EXECUTE_SEAM [sim]: starting seam {seam_id!r} at {weld_speed} mm/s'
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
                    f'EXECUTE_SEAM [sim]: cancelled at {welded_length:.1f} mm (phase: {phase})'
                )
                return result

            welded_length += length_per_phase
            feedback.progress_pct  = (i + 1) / len(self.PHASES) * 100.0
            feedback.phase         = phase
            feedback.current_speed = weld_speed if phase == 'WELDING' else 0.0
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'EXECUTE_SEAM [sim]: {phase} [{i+1}/{len(self.PHASES)}] '
                f'→ {feedback.progress_pct:.0f}% | {welded_length:.1f} mm welded'
            )
            time.sleep(self.PHASE_DURATION)

        goal_handle.succeed()
        result.success        = True
        result.message        = f'Seam {seam_id} welded successfully (simulation)'
        result.seam_length_mm = self.MOCK_SEAM_LENGTH_MM
        self.get_logger().info(
            f'EXECUTE_SEAM [sim]: complete — {self.MOCK_SEAM_LENGTH_MM} mm welded'
        )
        return result

    # ── Hardware execution (via RobinPlanner /weld_experiment) ────────────────

    def _execute_hardware(self, goal_handle) -> ExecuteSeam.Result:
        seam_id       = goal_handle.request.seam_id
        weld_speed    = goal_handle.request.weld_speed       # mm/s
        wire_feed     = goal_handle.request.wire_feed_rate   # m/min

        result = ExecuteSeam.Result()

        # 1. Wait for planner action server
        if not self._planner_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'EXECUTE_SEAM [hw]: /weld_experiment server not available — '
                'is robin_moveit_control running?'
            )
            goal_handle.abort()
            result.success = False
            result.message = 'Planner action server unavailable'
            return result

        # 2. Build WeldBead from seam registry + goal parameters
        bead = self._build_weld_bead(seam_id, weld_speed, wire_feed)

        # 3. Build WeldExperiment goal
        planner_goal = WeldExperiment.Goal()
        planner_goal.weld_beads = [bead]
        planner_goal.dry_run    = False

        self.get_logger().info(
            f'EXECUTE_SEAM [hw]: delegating seam {seam_id!r} to /weld_experiment '
            f'(start={bead.start_point}, end={bead.end_point}, '
            f'speed={bead.target_speed:.3f} m/s, '
            f'current={bead.target_current:.0f} A, voltage={bead.target_voltage:.0f} V)'
        )

        # 4. Send goal and wait synchronously using threading.Event
        done_event    = threading.Event()
        planner_state: dict[str, Any] = {}

        feedback_ref: dict[str, Any] = {}

        def on_feedback(feedback_msg) -> None:
            fb = feedback_msg.feedback
            feedback_ref['last'] = fb
            skill_fb = ExecuteSeam.Feedback()
            skill_fb.phase         = fb.status
            skill_fb.progress_pct  = fb.progress_percentage
            skill_fb.current_speed = weld_speed
            goal_handle.publish_feedback(skill_fb)
            self.get_logger().debug(
                f'EXECUTE_SEAM [hw]: planner feedback → {fb.status} {fb.progress_percentage:.0f}%'
            )

        def on_goal_response(future) -> None:
            gh = future.result()
            if not gh.accepted:
                self.get_logger().warning('EXECUTE_SEAM [hw]: planner REJECTED goal')
                planner_state['success'] = False
                planner_state['message'] = 'Planner rejected the WeldExperiment goal'
                done_event.set()
                return
            planner_state['goal_handle'] = gh
            res_future = gh.get_result_async()
            res_future.add_done_callback(on_result)

        def on_result(future) -> None:
            r = future.result().result
            planner_state['success']    = r.success
            planner_state['completion'] = r.completion_percentage
            done_event.set()

        send_future = self._planner_client.send_goal_async(
            planner_goal, feedback_callback=on_feedback
        )
        send_future.add_done_callback(on_goal_response)

        # Wait for planner to finish; periodically check for ESTOP cancel
        while not done_event.wait(timeout=0.5):
            if goal_handle.is_cancel_requested:
                planner_gh = planner_state.get('goal_handle')
                if planner_gh:
                    self.get_logger().info(
                        'EXECUTE_SEAM [hw]: cancel requested — cancelling planner goal'
                    )
                    planner_gh.cancel_goal_async()
                done_event.wait(timeout=5.0)
                goal_handle.canceled()
                result.success = False
                result.message = 'Weld cancelled — ESTOP or operator request'
                return result

        # 5. Map planner result back to ExecuteSeam result
        success    = planner_state.get('success', False)
        completion = planner_state.get('completion', 0.0)

        if success:
            goal_handle.succeed()
            result.success        = True
            result.message        = f'Seam {seam_id} welded successfully'
            result.seam_length_mm = completion  # planner reports completion %
            self.get_logger().info(
                f'EXECUTE_SEAM [hw]: complete — {completion:.0f}% of seam welded'
            )
        else:
            goal_handle.abort()
            result.success = False
            result.message = planner_state.get('message', 'Planner reported failure')
            self.get_logger().error(f'EXECUTE_SEAM [hw]: FAILED — {result.message}')

        return result

    # ── Seam registry helper ──────────────────────────────────────────────────

    def _build_weld_bead(
        self, seam_id: str, weld_speed_mm_s: float, wire_feed_m_min: float
    ) -> WeldBead:
        """Construct a WeldBead from seam registry coordinates and goal parameters."""
        entry = self._seam_registry.get(seam_id, self._seam_registry.get('seam_01'))
        if entry is None:
            self.get_logger().warning(
                f'Seam {seam_id!r} not in registry and no seam_01 fallback — using zeros'
            )
            start = (0.0, 0.0, 0.0)
            end   = (0.0, 0.1, 0.0)
        else:
            start = entry['start']
            end   = entry['end']
            if seam_id not in self._seam_registry:
                self.get_logger().warning(
                    f'Seam {seam_id!r} not in registry — using seam_01 coordinates'
                )

        bead = WeldBead()
        bead.bead_id       = seam_id
        bead.start_point   = Point(x=float(start[0]), y=float(start[1]), z=float(start[2]))
        bead.end_point     = Point(x=float(end[0]),   y=float(end[1]),   z=float(end[2]))
        bead.target_speed  = weld_speed_mm_s / 1000.0   # mm/s → m/s
        bead.target_current    = DEFAULT_CURRENT_A
        bead.target_voltage    = DEFAULT_VOLTAGE_V
        bead.wire_feed_speed   = wire_feed_m_min
        bead.stickout          = DEFAULT_STICKOUT_M
        return bead


def main(args=None):
    rclpy.init(args=args)
    node = WeldingSeamSkillNode()

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
