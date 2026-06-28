#!/usr/bin/env python3
"""
WeldingSupervisorNode: intent-to-skill mission controller.

Subscribes to /intents, parses the JSON payload and dispatches each intent
to the corresponding skill action server. Implements the same MissionController
pattern as ErgoBot_AI's llm_supervisor/mission_controller.py, but with
welding-specific skills replacing the MOVE_TO / SAY skill set.

Architecture:
  /intents (Intent msg)
       │
       ▼
  intent_callback()
       ├─► MOVE_TO_HOME             ──► welding_home_skill/execute      (action)
       ├─► EXECUTE_SEAM             ──► welding_seam_skill/execute      (action)
       ├─► START_PROCESS            ──► welding_seam_skill/execute      (reuses seam)
       ├─► PAUSE_PROCESS            ──► robin_planner weld/pause(true)  (freeze in place)
       ├─► RESUME_PROCESS           ──► robin_planner weld/pause(false) (continue bead)
       ├─► STOP_PROCESS             ──► cancel all + welding_home_skill/execute
       ├─► ESTOP (Abort)            ──► cancel all + move home + publish /weld_errors
       │
       │   ── ROBIN dashboard button intents ──────────────────────────────────
       ├─► REQUEST_AI_RECOMMENDATION──► welding_recommendation_skill/execute
       ├─► MANUAL_ADJUST            ──► welding_manual_skill/execute
       └─► LAUNCH_NEW_DOE           ──► publish on /doe/launch (String, JSON)

Threading: ReentrantCallbackGroup + MultiThreadedExecutor(4 threads) allows
action client callbacks (goal response, result, feedback) to fire concurrently
with the intent subscriber without deadlocking.
"""
import json
import os
import subprocess
import threading

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from welding_msgs.action import (
    ExecuteSeam,
    ManualAdjust,
    MoveToHome,
    RequestAIRecommendation,
)
from welding_msgs.msg import Intent


# ── Dashboard → ROS parameter contract ────────────────────────────────────────
# The dashboard sends ML model-input keys (robin-dashboard/src/config/aiInputFeatures.ts)
# in a different vocabulary/units than the skill action goals. The supervisor is the
# single place that turns the generic Intent.data dict into typed skill goals, so the
# translation lives here. The dashboard stays model-driven; the skills keep their
# canonical fields. Legacy keys (weld_speed/wire_feed/current/voltage) still resolve.

# Dashboard parameter key -> (ExecuteSeam motion-goal field, factor to the goal's unit).
#   ExecuteSeam.wire_feed_rate is m/min; ExecuteSeam.weld_speed is mm/s.
SEAM_PARAM_MAP = {
    'wire_feed_speed_mpm_model_input': ('wire_feed_rate', 1.0),     # m/min -> m/min
    'travel_speed_mps_model_input':    ('weld_speed',     1000.0),  # m/s   -> mm/s
    # legacy / backward-compat keys
    'wire_feed':   ('wire_feed_rate', 1.0),     # m/min
    'wire_speed':  ('wire_feed_rate', 1.0),     # m/min
    'weld_speed':  ('weld_speed',     1.0),     # mm/s
    'travel_speed': ('weld_speed',    1000.0),  # m/s -> mm/s
}

# Dashboard parameter key -> (manual-skill / Fronius parameter name, unit) for hardware
# adjusts. The model-input wire feed also drives the Fronius wire speed; current/voltage
# pass through. Motion / informational params (weld_speed, travel_speed,
# arc_length_correction_mm_model_input) are intentionally absent so they are skipped
# rather than sent to the manual skill as unrecognized parameters.
MANUAL_PARAM_MAP = {
    'wire_feed_speed_mpm_model_input': ('wire_speed', 'm/min'),
    'current':    ('current', 'A'),
    'voltage':    ('voltage', 'V'),
    'wire_speed': ('wire_speed', 'm/min'),
}


class WeldingSupervisorNode(Node):
    """Routes Intent messages to welding skill action servers."""

    # Action server names — must match what skill nodes advertise
    HOME_ACTION           = 'welding_home_skill/execute'
    SEAM_ACTION           = 'welding_seam_skill/execute'
    RECOMMENDATION_ACTION = 'welding_recommendation_skill/execute'
    MANUAL_ACTION         = 'welding_manual_skill/execute'

    # Human-readable label for an action's terminal goal status. Used for logging:
    # the goal status is authoritative, unlike the result payload (see _on_result).
    _STATUS_NAMES = {
        GoalStatus.STATUS_SUCCEEDED: 'SUCCESS',
        GoalStatus.STATUS_CANCELED:  'CANCELED',
        GoalStatus.STATUS_ABORTED:   'ABORTED',
    }

    def __init__(self):
        super().__init__('welding_supervisor')

        # Simulation / headless mode (default True, matching the skill nodes). In
        # sim mode LAUNCH_NEW_DOE skips the rqt OperatorPanel GUI (not built in the
        # lite workspace and unusable headless) and only publishes the notification.
        self.declare_parameter('use_simulation', True)
        self._use_simulation = (
            self.get_parameter('use_simulation').get_parameter_value().bool_value
        )

        # ReentrantCallbackGroup lets action callbacks fire while intent
        # callback is still on the stack (needed for concurrent skill execution)
        cb = ReentrantCallbackGroup()

        # Intent subscriber
        self._intent_sub = self.create_subscription(
            Intent,
            '/intents',
            self._intent_callback,
            10,
            callback_group=cb,
        )

        # Skill action clients — original motion/weld skills
        self._home_client = ActionClient(
            self, MoveToHome, self.HOME_ACTION, callback_group=cb
        )
        self._seam_client = ActionClient(
            self, ExecuteSeam, self.SEAM_ACTION, callback_group=cb
        )

        # Skill action clients — ROBIN dashboard button skills
        self._recommendation_client = ActionClient(
            self, RequestAIRecommendation, self.RECOMMENDATION_ACTION, callback_group=cb
        )
        self._manual_client = ActionClient(
            self, ManualAdjust, self.MANUAL_ACTION, callback_group=cb
        )

        # Publisher for DOE GUI launch notifications
        self._doe_launch_pub = self.create_publisher(String, '/doe/launch', 10)

        # Operator pause/resume freezes the in-progress bead in place (robin_planner
        # weld/pause service: data=True pauses, data=False resumes).
        self._pause_client = self.create_client(
            SetBool, 'weld/pause', callback_group=cb
        )
        # Abort surfaces an operator-visible error on /weld_errors (JSON string).
        self._weld_errors_pub = self.create_publisher(String, '/weld_errors', 10)

        # Track active goal handles so ESTOP/STOP can cancel them all. Mutated from
        # several action callbacks (goal response, result) and the intent thread
        # under a MultiThreadedExecutor, so guard every access with this lock.
        self._goals_lock = threading.Lock()
        self._active_goal_handles: list = []
        # Subset: only seam/start goals, so PAUSE can cancel only those
        self._active_seam_goal_handles: list = []
        # Track the OperatorPanel subprocess to avoid duplicate windows
        self._doe_proc: subprocess.Popen | None = None

        self.get_logger().info(
            'WeldingSupervisorNode ready — listening on /intents'
        )

    # ── Intent router ──────────────────────────────────────────────────────

    def _intent_callback(self, msg: Intent) -> None:
        self.get_logger().info(
            f'Received intent: {msg.intent!r} | modality: {msg.modality} | '
            f'data: {msg.data}'
        )

        data: dict = {}
        if msg.data:
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                self.get_logger().warning(
                    f'Cannot parse intent data as JSON: {msg.data!r}'
                )

        if msg.intent == Intent.MOVE_TO_HOME:
            self._dispatch_move_home(data)
        elif msg.intent == Intent.EXECUTE_SEAM:
            self._dispatch_execute_seam(data)
        elif msg.intent == Intent.ESTOP:
            self._handle_estop(data)
        # ── ROBIN dashboard button intents ─────────────────────────────────
        elif msg.intent == Intent.START_PROCESS:
            self._dispatch_start_process(data)
        elif msg.intent == Intent.REQUEST_AI_RECOMMENDATION:
            self._dispatch_recommendation(data)
        elif msg.intent == Intent.MANUAL_ADJUST:
            self._dispatch_manual_adjust(data)
        elif msg.intent == Intent.LAUNCH_NEW_DOE:
            self._dispatch_launch_doe(data)
        elif msg.intent == Intent.PAUSE_PROCESS:
            self._handle_pause(data)
        elif msg.intent == Intent.RESUME_PROCESS:
            self._handle_resume(data)
        elif msg.intent == Intent.STOP_PROCESS:
            self._handle_stop(data)
        else:
            self.get_logger().warning(f'Unknown intent type: {msg.intent!r}')

    # ── Dispatchers — original intents ─────────────────────────────────────

    def _dispatch_move_home(self, data: dict) -> None:
        goal = MoveToHome.Goal()
        goal.use_fast_speed = bool(data.get('fast', False))
        self._send_goal(self._home_client, goal, Intent.MOVE_TO_HOME)

    def _dispatch_execute_seam(self, data: dict) -> None:
        goal = ExecuteSeam.Goal()
        goal.seam_id        = str(data.get('seam_id', 'seam_01'))
        goal.weld_speed     = 5.0   # mm/s default
        goal.wire_feed_rate = 4.0   # m/min default
        self._apply_seam_params(data, goal)  # override from dashboard UI params
        self._send_goal(self._seam_client, goal, Intent.EXECUTE_SEAM)

    def _handle_estop(self, data: dict) -> None:
        """ABORT / ESTOP: cancel all active goals, move home, and emit an error."""
        reason = data.get('reason', 'unknown')
        self.get_logger().warning(
            f'ESTOP received (reason: {reason}) — cancelling all goals, '
            'moving home, and publishing an error'
        )
        # Cancel all active goals and return to a safe home position.
        self._cancel_all_and_home(Intent.ESTOP)
        # Surface an operator-visible error message.
        err = String()
        err.data = json.dumps({
            'error': 'PROCESS_ABORTED',
            'reason': reason,
            'message': 'Operator aborted the process; robot returned to home.',
        })
        self._weld_errors_pub.publish(err)

    def _handle_pause(self, data: dict) -> None:
        """PAUSE_PROCESS: freeze the bead in place (arm holds, arc + telemetry off).

        The seam goal stays active so RESUME can continue the remaining segments;
        the robot does NOT move to home.
        """
        self.get_logger().info('PAUSE_PROCESS: freezing bead in place')
        self._set_pause(True)

    def _handle_resume(self, data: dict) -> None:
        """RESUME_PROCESS: continue the frozen bead from the current waypoint."""
        self.get_logger().info('RESUME_PROCESS: continuing the bead')
        self._set_pause(False)

    def _set_pause(self, pause: bool) -> None:
        """Call robin_planner's weld/pause service (non-blocking)."""
        label = 'PAUSE' if pause else 'RESUME'
        if not self._pause_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f'{label}: weld/pause service unavailable — is robin_planner running?'
            )
            return
        req = SetBool.Request()
        req.data = pause
        future = self._pause_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'{label}: weld/pause acknowledged')
        )

    def _cancel_all_and_home(self, label: str) -> None:
        """Cancel every active goal (stop welding) and send the robot to home.

        We do NOT resume a paused bead first: the seam loop checks is_cancel_requested
        before its pause-continue, so a frozen bead observes the cancel within ~0.1 s.
        Resuming first would briefly re-start the weld ("resume then stop"), which is
        wrong for STOP / LAUNCH_NEW_DOE / ABORT — they must stop directly. Any leftover
        pause state is cleared by the next START (_execute_mock clears _pause_event).
        """
        with self._goals_lock:
            handles = list(self._active_goal_handles)
            self._active_goal_handles.clear()
            self._active_seam_goal_handles.clear()
        for gh in handles:
            try:
                gh.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(f'  {label}: cancel failed: {exc}')
        goal = MoveToHome.Goal()
        goal.use_fast_speed = False
        self._send_goal(self._home_client, goal, label)

    def _handle_stop(self, data: dict) -> None:
        """STOP_PROCESS: cancel all active goals, then move robot to home (orderly shutdown)."""
        reason = data.get('reason', 'operator_stop')
        self.get_logger().info(
            f'STOP_PROCESS (reason: {reason}): cancelling all goals and moving home'
        )
        self._cancel_all_and_home(Intent.STOP_PROCESS)

    # ── Dispatchers — ROBIN dashboard button intents ───────────────────────

    def _dispatch_start_process(self, data: dict) -> None:
        """START_PROCESS reuses welding_seam_skill (start = execute the weld)."""
        goal = ExecuteSeam.Goal()
        goal.seam_id        = str(data.get('seam_id', 'seam_01'))
        goal.weld_speed     = 5.0   # mm/s default
        goal.wire_feed_rate = 4.0   # m/min default
        self._apply_seam_params(data, goal)  # override from dashboard UI params
        self._send_goal(self._seam_client, goal, Intent.START_PROCESS)
        # Apply welding parameters to hardware (Fronius) before arc starts.
        for adj_goal in self._collect_manual_goals(data):
            self._send_goal(
                self._manual_client, adj_goal,
                f'{Intent.START_PROCESS}→MANUAL_ADJUST',
            )

    def _dispatch_recommendation(self, data: dict) -> None:
        goal = RequestAIRecommendation.Goal()
        goal.process_id = str(data.get('process_id', ''))
        goal.mode       = str(data.get('mode', 'geometry_driven'))
        self._send_goal(
            self._recommendation_client, goal, Intent.REQUEST_AI_RECOMMENDATION
        )
        # Apply the recommended parameters to hardware via manual_skill (translated).
        for adj_goal in self._collect_manual_goals(data):
            self._send_goal(
                self._manual_client, adj_goal,
                f'{Intent.REQUEST_AI_RECOMMENDATION}→MANUAL_ADJUST',
            )

    def _dispatch_manual_adjust(self, data: dict) -> None:
        if data.get('parameters'):
            # Multi-parameter format: translate dashboard keys to recognized
            # Fronius params (motion/informational params are skipped).
            for goal in self._collect_manual_goals(data):
                self._send_goal(self._manual_client, goal, Intent.MANUAL_ADJUST)
        else:
            # Legacy single-parameter format
            goal = ManualAdjust.Goal()
            goal.parameter_name = str(data.get('parameter_name', 'weld_speed'))
            goal.new_value      = float(data.get('new_value', 5.0))
            goal.unit           = str(data.get('unit', 'mm/s'))
            self._send_goal(self._manual_client, goal, Intent.MANUAL_ADJUST)

    def _dispatch_launch_doe(self, data: dict) -> None:
        """Publish a notification, stop any active weld, send the robot home, and
        (on hardware) spawn the robin_rqt OperatorPanel GUI.

        Launching a new DOE supersedes the current run, so the robot must stop
        welding and return to a safe home position before the new experiment.
        """
        msg = String()
        msg.data = json.dumps(data)
        self._doe_launch_pub.publish(msg)
        self.get_logger().info(
            f'LAUNCH_NEW_DOE: notification published on /doe/launch | data: {json.dumps(data)}'
        )

        # A new DOE supersedes the running process: stop welding and home the robot.
        self.get_logger().info('LAUNCH_NEW_DOE: stopping weld and moving home')
        self._cancel_all_and_home(Intent.LAUNCH_NEW_DOE)

        # In simulation / headless mode the robin_rqt OperatorPanel GUI is neither
        # built (lite workspace) nor renderable (no DISPLAY), so skip spawning it —
        # the /doe/launch notification above is the observable effect of the intent.
        if self._use_simulation:
            self.get_logger().info(
                'LAUNCH_NEW_DOE: skipping rqt OperatorPanel (sim/headless mode)'
            )
            return

        # Guard: don't open a second window if the GUI is already running
        if self._doe_proc is not None and self._doe_proc.poll() is None:
            self.get_logger().warning(
                'LAUNCH_NEW_DOE: OperatorPanel already running — skipping launch'
            )
            return

        # Build environment: inherit everything, ensure DISPLAY is set
        env = os.environ.copy()
        if 'DISPLAY' not in env or not env['DISPLAY']:
            env['DISPLAY'] = ':0'

        self._doe_proc = subprocess.Popen(
            ['ros2', 'launch', 'robin_rqt', 'operator_panel.launch.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
        )
        self.get_logger().info(
            f'LAUNCH_NEW_DOE: OperatorPanel spawned (pid {self._doe_proc.pid})'
        )

        # Log subprocess output in a background thread so it appears in the supervisor log
        def _log_output(proc, logger):
            for line in proc.stdout:
                logger.info(f'[operator_panel] {line.decode(errors="replace").rstrip()}')
            rc = proc.wait()
            if rc != 0:
                logger.warning(f'LAUNCH_NEW_DOE: OperatorPanel exited with code {rc}')

        threading.Thread(
            target=_log_output,
            args=(self._doe_proc, self.get_logger()),
            daemon=True,
        ).start()

    # ── Dashboard parameter translation ────────────────────────────────────

    def _resolve_seam_field(self, name: str, unit: str):
        """Map a parameter name/unit onto an ExecuteSeam motion field + factor.

        Looks up SEAM_PARAM_MAP first; if the key is unknown (e.g. a profile
        renamed it), falls back to the unit so renamed keys still resolve.
        Returns (field, factor) or (None, 0.0) if it is not a motion parameter.
        """
        if name in SEAM_PARAM_MAP:
            return SEAM_PARAM_MAP[name]
        u = unit.lower()
        if u in ('m/s', 'mps'):
            return ('weld_speed', 1000.0)      # m/s -> mm/s
        if u in ('mm/s', 'mmps'):
            return ('weld_speed', 1.0)
        if u in ('m/min', 'mpm'):
            return ('wire_feed_rate', 1.0)
        return (None, 0.0)

    def _apply_seam_params(self, data: dict, goal) -> None:
        """Override ExecuteSeam.Goal motion fields from dashboard data.

        Reads both the flat keys (START_PROCESS / EXECUTE_SEAM) and the
        parameters[] list (manual / recommendation form). Fields the dashboard
        does not provide keep the goal defaults set by the caller.
        """
        # Flat keys carry no unit; the static SEAM_PARAM_MAP covers them.
        for key, value in data.items():
            if key in SEAM_PARAM_MAP and isinstance(value, (int, float)):
                field, factor = SEAM_PARAM_MAP[key]
                setattr(goal, field, float(value) * factor)
        # parameters[] entries carry an explicit unit -> allow the unit fallback.
        for p in data.get('parameters', []) or []:
            value = p.get('new_value')
            if not isinstance(value, (int, float)):
                continue
            field, factor = self._resolve_seam_field(
                str(p.get('parameter_name', '')), str(p.get('unit', ''))
            )
            if field:
                setattr(goal, field, float(value) * factor)

    def _resolve_manual_param(self, name: str, unit: str):
        """Map a parameter name onto a manual-skill / Fronius (name, unit), or None.

        Returns None for motion / informational parameters (weld_speed,
        travel_speed, arc_length_correction_*) so they are skipped instead of
        being sent to the manual skill as unrecognized parameters.
        """
        if name in MANUAL_PARAM_MAP:
            return MANUAL_PARAM_MAP[name]
        return None

    def _make_manual_goal(self, name: str, value: float, unit: str) -> ManualAdjust.Goal:
        goal = ManualAdjust.Goal()
        goal.parameter_name = name
        goal.new_value      = float(value)
        goal.unit           = unit
        return goal

    def _collect_manual_goals(self, data: dict) -> list:
        """Build the list of hardware ManualAdjust goals implied by `data`.

        Sources: flat current/voltage keys (legacy START_PROCESS form) and the
        parameters[] list (dashboard manual-adjust / recommendation form),
        translating each name to what the manual skill recognizes.
        """
        goals: list = []
        for legacy, unit in (('current', 'A'), ('voltage', 'V')):
            value = data.get(legacy)
            if isinstance(value, (int, float)):
                goals.append(self._make_manual_goal(legacy, value, unit))
        for p in data.get('parameters', []) or []:
            value = p.get('new_value')
            if not isinstance(value, (int, float)):
                continue
            mapped = self._resolve_manual_param(
                str(p.get('parameter_name', '')), str(p.get('unit', ''))
            )
            if mapped:
                goals.append(self._make_manual_goal(mapped[0], float(value), mapped[1]))
        return goals

    # ── Generic goal helper ────────────────────────────────────────────────

    def _send_goal(self, client: ActionClient, goal, label: str) -> None:
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'{label}: action server not available — is the skill running?'
            )
            return

        future = client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._on_feedback(fb, label),
        )
        future.add_done_callback(
            lambda f: self._on_goal_response(f, label)
        )
        self.get_logger().info(f'{label}: goal sent')

    def _on_goal_response(self, future, label: str) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f'{label}: goal REJECTED by skill server')
            return

        self.get_logger().info(f'{label}: goal ACCEPTED')
        with self._goals_lock:
            self._active_goal_handles.append(goal_handle)
            if label in (Intent.START_PROCESS, Intent.EXECUTE_SEAM):
                self._active_seam_goal_handles.append(goal_handle)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_result(f, label, goal_handle)
        )

    def _on_result(self, future, label: str, goal_handle) -> None:
        with self._goals_lock:
            try:
                self._active_goal_handles.remove(goal_handle)
            except ValueError:
                pass  # already removed by ESTOP/STOP
            try:
                self._active_seam_goal_handles.remove(goal_handle)
            except ValueError:
                pass  # not a seam goal or already removed

        # Judge the outcome from the authoritative goal STATUS, not the result
        # payload: the lite sim action servers reach the terminal status correctly
        # (SUCCEEDED/CANCELED/ABORTED) but deliver a default-valued result payload
        # (success=False, message=''), so reading result.success mislabels a
        # successful home/stop as FAILED.
        response = future.result()
        status = self._STATUS_NAMES.get(
            response.status, f'status={response.status}'
        )
        message = getattr(response.result, 'message', '') or ''
        self.get_logger().info(
            f'{label}: result → {status} | "{message}"'
        )

    def _on_feedback(self, feedback_msg, label: str) -> None:
        fb = feedback_msg.feedback
        self.get_logger().debug(f'{label}: feedback → {fb}')


def main(args=None):
    rclpy.init(args=args)
    node = WeldingSupervisorNode()

    # MultiThreadedExecutor is required for ReentrantCallbackGroup to work
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
