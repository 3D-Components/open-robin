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
       ├─► ESTOP                    ──► cancel_goal_async() on all handles
       │
       │   ── ROBIN dashboard button intents ──────────────────────────────────
       ├─► START_PROCESS            ──► welding_seam_skill/execute      (reuses seam)
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
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from welding_msgs.action import (
    ExecuteSeam,
    ManualAdjust,
    MoveToHome,
    RequestAIRecommendation,
)
from welding_msgs.msg import Intent


class WeldingSupervisorNode(Node):
    """Routes Intent messages to welding skill action servers."""

    # Action server names — must match what skill nodes advertise
    HOME_ACTION           = 'welding_home_skill/execute'
    SEAM_ACTION           = 'welding_seam_skill/execute'
    RECOMMENDATION_ACTION = 'welding_recommendation_skill/execute'
    MANUAL_ACTION         = 'welding_manual_skill/execute'

    def __init__(self):
        super().__init__('welding_supervisor')

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

        # Track active goal handles so ESTOP/STOP can cancel them all
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
        goal.weld_speed     = float(data.get('weld_speed', 5.0))
        goal.wire_feed_rate = float(data.get('wire_feed', 4.0))
        self._send_goal(self._seam_client, goal, Intent.EXECUTE_SEAM)

    def _handle_estop(self, data: dict) -> None:
        reason = data.get('reason', 'unknown')
        self.get_logger().warning(
            f'ESTOP received (reason: {reason}) — cancelling all active goals'
        )
        for goal_handle in list(self._active_goal_handles):
            try:
                goal_handle.cancel_goal_async()
                self.get_logger().info(f'  Cancelled goal: {goal_handle}')
            except Exception as exc:
                self.get_logger().error(f'  Cancel failed: {exc}')
        self._active_goal_handles.clear()

    def _handle_pause(self, data: dict) -> None:
        """PAUSE_PROCESS: cancel active seam goal, then move robot to home."""
        self.get_logger().info('PAUSE_PROCESS: cancelling seam and moving to home')
        for gh in list(self._active_seam_goal_handles):
            try:
                gh.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(f'  Pause cancel failed: {exc}')
        self._active_seam_goal_handles.clear()
        goal = MoveToHome.Goal()
        goal.use_fast_speed = False
        self._send_goal(self._home_client, goal, Intent.PAUSE_PROCESS)

    def _handle_resume(self, data: dict) -> None:
        """RESUME_PROCESS: move robot to home so operator can re-trigger the seam."""
        self.get_logger().info('RESUME_PROCESS: moving to home')
        goal = MoveToHome.Goal()
        goal.use_fast_speed = False
        self._send_goal(self._home_client, goal, Intent.RESUME_PROCESS)

    def _handle_stop(self, data: dict) -> None:
        """STOP_PROCESS: cancel all active goals, then move robot to home (orderly shutdown)."""
        reason = data.get('reason', 'operator_stop')
        self.get_logger().info(
            f'STOP_PROCESS (reason: {reason}): cancelling all goals and moving home'
        )
        for gh in list(self._active_goal_handles):
            try:
                gh.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(f'  Stop cancel failed: {exc}')
        self._active_goal_handles.clear()
        self._active_seam_goal_handles.clear()
        goal = MoveToHome.Goal()
        goal.use_fast_speed = False
        self._send_goal(self._home_client, goal, Intent.STOP_PROCESS)

    # ── Dispatchers — ROBIN dashboard button intents ───────────────────────

    def _dispatch_start_process(self, data: dict) -> None:
        """START_PROCESS reuses welding_seam_skill (start = execute the weld)."""
        goal = ExecuteSeam.Goal()
        goal.seam_id = str(data.get('seam_id', 'seam_01'))

        # New parameter names from the dashboard (preferred); fall back to legacy keys.
        travel_speed_mps = data.get('travel_speed_mps_model_input')
        if travel_speed_mps is not None:
            goal.weld_speed = float(travel_speed_mps) * 1000.0  # m/s → mm/s
        else:
            goal.weld_speed = float(data.get('weld_speed', 5.0))

        wire_feed_mpm = data.get('wire_feed_speed_mpm_model_input')
        if wire_feed_mpm is not None:
            goal.wire_feed_rate = float(wire_feed_mpm)  # m/min (same unit as action)
        else:
            goal.wire_feed_rate = float(data.get('wire_feed', 4.0))

        self._send_goal(self._seam_client, goal, Intent.START_PROCESS)

        # Arc length correction → hardware adjust
        arc_length_mm = data.get('arc_length_correction_mm_model_input')
        if arc_length_mm is not None:
            adj_goal = ManualAdjust.Goal()
            adj_goal.parameter_name = 'arc_length_correction'
            adj_goal.new_value      = float(arc_length_mm)
            adj_goal.unit           = 'mm'
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
        # Apply the recommended parameters to hardware via manual_skill
        params = data.get('parameters')
        if params:
            for p in params:
                adj_goal = ManualAdjust.Goal()
                adj_goal.parameter_name = str(p.get('parameter_name', ''))
                adj_goal.new_value      = float(p.get('new_value', 0.0))
                adj_goal.unit           = str(p.get('unit', ''))
                self._send_goal(
                    self._manual_client, adj_goal,
                    f'{Intent.REQUEST_AI_RECOMMENDATION}→MANUAL_ADJUST',
                )

    def _dispatch_manual_adjust(self, data: dict) -> None:
        params = data.get('parameters')
        if params:
            # Multi-parameter format: dispatch one ManualAdjust goal per entry
            for p in params:
                goal = ManualAdjust.Goal()
                goal.parameter_name = str(p.get('parameter_name', ''))
                goal.new_value      = float(p.get('new_value', 0.0))
                goal.unit           = str(p.get('unit', ''))
                self._send_goal(self._manual_client, goal, Intent.MANUAL_ADJUST)
        else:
            # Legacy single-parameter format
            goal = ManualAdjust.Goal()
            goal.parameter_name = str(data.get('parameter_name', 'weld_speed'))
            goal.new_value      = float(data.get('new_value', 5.0))
            goal.unit           = str(data.get('unit', 'mm/s'))
            self._send_goal(self._manual_client, goal, Intent.MANUAL_ADJUST)

    def _dispatch_launch_doe(self, data: dict) -> None:
        """Publish a notification and spawn the robin_rqt OperatorPanel GUI."""
        msg = String()
        msg.data = json.dumps(data)
        self._doe_launch_pub.publish(msg)
        self.get_logger().info(
            f'LAUNCH_NEW_DOE: notification published on /doe/launch | data: {json.dumps(data)}'
        )

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
        self._active_goal_handles.append(goal_handle)
        if label in (Intent.START_PROCESS, Intent.EXECUTE_SEAM):
            self._active_seam_goal_handles.append(goal_handle)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_result(f, label, goal_handle)
        )

    def _on_result(self, future, label: str, goal_handle) -> None:
        try:
            self._active_goal_handles.remove(goal_handle)
        except ValueError:
            pass  # already removed by ESTOP/STOP
        try:
            self._active_seam_goal_handles.remove(goal_handle)
        except ValueError:
            pass  # not a seam goal or already removed

        result = future.result().result
        status = 'SUCCESS' if result.success else 'FAILED'
        self.get_logger().info(
            f'{label}: result → {status} | "{result.message}"'
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
