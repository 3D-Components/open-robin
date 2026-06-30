#!/usr/bin/env python3
"""
WeldingSeamSkillNode: lifecycle action server for EXECUTE_SEAM / START_PROCESS.

ros4hri concept: maps to motion_skills/action/ExecuteCartesianTrajectory.

Modes (controlled by ROS2 parameter 'use_simulation', default True):
  use_simulation=True  — lite mock: sweeps the arm along the seam by publishing
                         joint positions to /joint_states_manual (the home skill
                         relays them to /joint_states), and publishes synthetic
                         ProcessTelemetry on /robin/telemetry (ROS 2 -> DDS ->
                         FIWARE). Hosts the weld/pause service so the supervisor
                         can freeze/resume the weld. No Gazebo/MoveIt.
  use_simulation=False — Gazebo/hardware: delegates to /execute_bead on RobinPlanner
                         (MoveItPy LIN motions + WeldingCoordinator arc start/stop).

Seam coordinates are loaded from the 'seam_registry' parameter (YAML dict keyed by
seam_id).  A built-in default for 'seam_01' is provided so the system runs out of
the box; replace with actual robot-frame coordinates for production.
"""
from __future__ import annotations

import math
import random
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from robin_interfaces.msg import ProcessTelemetry
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

    In simulation mode (default): runs a lite mock that animates the arm and
    streams synthetic telemetry.  In hardware mode: delegates to /execute_bead.
    """

    ACTION_NAME         = 'welding_seam_skill/execute'
    PLANNER_ACTION_NAME = 'execute_bead'

    # UR joint names — must match robot_state_publisher / URDF / home skill
    UR_JOINTS = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
    ]

    # Base welding pose (matches the home skill's HOME_RADIANS) plus a sweep of
    # the shoulder-pan joint to trace the bead so the torch visibly moves.
    WELD_POSE         = [2.763658, -1.64229, 1.7227086, -2.143056, -1.384997, -0.331205]
    SWEEP_JOINT_IDX   = 0      # shoulder_pan_joint
    SWEEP_RANGE_RAD   = 0.50   # total sweep across the seam

    # Lite-mock timing
    WELD_DURATION_S   = 30.0   # one full bead at progress 0 -> 1
    RATE_HZ           = 20.0   # joint update rate (smooth motion)
    TELEMETRY_HZ      = 1.0    # ProcessTelemetry publish rate

    # Target geometry the synthetic telemetry centres on (mm), with a deviation
    # window in the middle of the bead so the dashboard shows a deviation.
    TARGET_HEIGHT_MM  = 3.0
    TARGET_WIDTH_MM   = 7.0

    def __init__(self):
        super().__init__('welding_seam_skill')
        self._action_server: ActionServer | None = None
        self._planner_client: ActionClient | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None
        self._use_simulation: bool = True
        self._seam_registry: dict = dict(DEFAULT_SEAM_REGISTRY)

        # Lite-mode publishers / service
        self._jsm_pub = None          # -> /joint_states_manual (home skill relays)
        self._telemetry_pub = None    # -> /robin/telemetry (DDS -> FIWARE)
        self._pause_service = None
        self._pause_event = threading.Event()

    # ── Lifecycle transitions ─────────────────────────────────────────────────

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.declare_parameter('use_simulation', True)
        self._use_simulation = (
            self.get_parameter('use_simulation').get_parameter_value().bool_value
        )
        mode = 'SIMULATION (lite)' if self._use_simulation else 'HARDWARE/Gazebo'
        self.get_logger().info(f'welding_seam_skill: configuring [{mode}]')

        if not self._use_simulation:
            from robin_interfaces.action import ExecuteBead  # hardware-only dep
            self._planner_client = ActionClient(
                self,
                ExecuteBead,
                self.PLANNER_ACTION_NAME,
                callback_group=ReentrantCallbackGroup(),
            )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_seam_skill: activating — advertising {self.ACTION_NAME}'
        )
        if self._use_simulation:
            self._jsm_pub = self.create_publisher(JointState, '/joint_states_manual', 10)
            self._telemetry_pub = self.create_publisher(ProcessTelemetry, '/robin/telemetry', 10)
            # Same service name the supervisor calls; in Gazebo mode robin_planner
            # hosts it instead.  data=True freezes the bead, data=False resumes.
            self._pause_service = self.create_service(
                SetBool, 'weld/pause', self._pause_service_cb,
                callback_group=ReentrantCallbackGroup(),
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

    # ── Pause/resume service ──────────────────────────────────────────────────

    def _pause_service_cb(self, request, response):
        if request.data:
            self._pause_event.set()
            self.get_logger().info('weld/pause: PAUSE — freezing bead in place')
            response.message = 'paused'
        else:
            self._pause_event.clear()
            self.get_logger().info('weld/pause: RESUME — continuing bead')
            response.message = 'resumed'
        response.success = True
        return response

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
        wire_feed  = goal_handle.request.wire_feed_rate or 4.0
        self.get_logger().info(
            f'EXECUTE_SEAM [sim]: welding seam {seam_id!r} '
            f'(animating arm + streaming telemetry)'
        )

        # A new bead must never inherit a stale pause.
        self._pause_event.clear()

        feedback = ExecuteSeam.Feedback()
        result   = ExecuteSeam.Result()

        total_steps      = int(self.WELD_DURATION_S * self.RATE_HZ)
        step_delay_s     = 1.0 / self.RATE_HZ
        telem_interval   = max(1, int(self.RATE_HZ / self.TELEMETRY_HZ))
        step             = 0

        while step <= total_steps:
            # Preempted by a newer goal
            if not goal_handle.is_active:
                result.success = False
                result.message = 'Seam preempted'
                return result
            # Operator cancel (STOP / ABORT)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success        = False
                result.message        = 'Weld cancelled — ESTOP or operator request'
                result.seam_length_mm = (step / total_steps) * 150.0
                self.get_logger().warning(
                    f'EXECUTE_SEAM [sim]: cancelled at {result.seam_length_mm:.0f} mm'
                )
                return result
            # Operator pause: freeze in place (stop advancing + stop telemetry)
            if self._pause_event.is_set():
                time.sleep(0.1)
                continue

            progress = step / total_steps

            # Animate the arm: sweep the shoulder-pan joint across the seam.
            self._publish_joint_sweep(progress)

            # Stream synthetic telemetry to FIWARE at TELEMETRY_HZ.
            if step % telem_interval == 0:
                self._publish_telemetry(seam_id, progress, wire_feed)

            feedback.progress_pct  = progress * 100.0
            feedback.phase         = 'WELDING' if 0.05 < progress < 0.95 else 'IGNITING'
            feedback.current_speed = weld_speed
            goal_handle.publish_feedback(feedback)

            step += 1
            time.sleep(step_delay_s)

        goal_handle.succeed()
        result.success        = True
        result.message        = f'Seam {seam_id} welded successfully (simulation)'
        result.seam_length_mm = 150.0
        self.get_logger().info('EXECUTE_SEAM [sim]: complete')
        return result

    def _publish_joint_sweep(self, progress: float) -> None:
        """Publish a joint pose that sweeps the torch along the seam."""
        positions = list(self.WELD_POSE)
        positions[self.SWEEP_JOINT_IDX] = (
            self.WELD_POSE[self.SWEEP_JOINT_IDX]
            - self.SWEEP_RANGE_RAD / 2.0
            + progress * self.SWEEP_RANGE_RAD
        )
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.UR_JOINTS)
        js.position = positions
        if self._jsm_pub is not None:
            self._jsm_pub.publish(js)

    def _publish_telemetry(self, seam_id: str, progress: float, wire_feed: float) -> None:
        """Publish synthetic ProcessTelemetry on /robin/telemetry (DDS -> FIWARE)."""
        # Inject a deviation window in the middle of the bead so the dashboard
        # shows a height drop / width bump relative to the target geometry.
        in_window = 0.4 <= progress <= 0.6
        dev = 1.0 if in_window else 0.0
        # NOTE: ProcessTelemetry has no header field; the DDS temporal trigger
        # falls back to the insert timestamp for observedAt.
        msg = ProcessTelemetry()
        msg.bead_id = str(seam_id)
        msg.progression = float(progress)
        msg.height = float(self.TARGET_HEIGHT_MM - 0.6 * dev + random.uniform(-0.05, 0.05))
        msg.width  = float(self.TARGET_WIDTH_MM + 0.8 * dev + random.uniform(-0.08, 0.08))
        msg.speed   = float(wire_feed)
        msg.current = float(DEFAULT_CURRENT_A + random.uniform(-3.0, 3.0))
        msg.voltage = float(DEFAULT_VOLTAGE_V + random.uniform(-0.4, 0.4))
        msg.cross_sectional_area = 0.0
        if self._telemetry_pub is not None:
            self._telemetry_pub.publish(msg)

    # ── Hardware execution (via RobinPlanner /execute_bead) ──────────────────

    def _execute_hardware(self, goal_handle) -> ExecuteSeam.Result:
        from robin_interfaces.action import ExecuteBead  # hardware-only dep
        seam_id    = goal_handle.request.seam_id
        weld_speed = goal_handle.request.weld_speed      # mm/s
        wire_feed  = goal_handle.request.wire_feed_rate  # m/min

        result = ExecuteSeam.Result()

        # 1. Wait for planner action server
        if not self._planner_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'EXECUTE_SEAM [hw]: /{self.PLANNER_ACTION_NAME} server not available — '
                'is robin_planner_node running?'
            )
            goal_handle.abort()
            result.success = False
            result.message = 'Planner action server unavailable'
            return result

        # 2. Build ExecuteBead goal from seam registry + goal parameters
        planner_goal = self._build_execute_bead_goal(
            ExecuteBead, seam_id, weld_speed, wire_feed
        )

        self.get_logger().info(
            f'EXECUTE_SEAM [hw]: delegating seam {seam_id!r} to /{self.PLANNER_ACTION_NAME} '
            f'(path_pts={len(planner_goal.path)}, '
            f'length={planner_goal.total_length:.3f} m, '
            f'speed={planner_goal.target_speed:.3f} m/s, '
            f'wire_feed={planner_goal.wire_feed_speed:.1f} m/min)'
        )

        # 3. Send goal and wait synchronously using threading.Event
        done_event    = threading.Event()
        planner_state: dict[str, Any] = {}

        def on_feedback(feedback_msg) -> None:
            fb = feedback_msg.feedback
            skill_fb = ExecuteSeam.Feedback()
            skill_fb.phase         = fb.step
            skill_fb.progress_pct  = fb.step_progress * 100.0
            skill_fb.current_speed = weld_speed
            goal_handle.publish_feedback(skill_fb)
            self.get_logger().debug(
                f'EXECUTE_SEAM [hw]: planner feedback → {fb.step} '
                f'{fb.step_progress * 100.0:.0f}%'
            )

        def on_goal_response(future) -> None:
            gh = future.result()
            if not gh.accepted:
                self.get_logger().warning('EXECUTE_SEAM [hw]: planner REJECTED goal')
                planner_state['success'] = False
                planner_state['message'] = 'Planner rejected the ExecuteBead goal'
                done_event.set()
                return
            planner_state['goal_handle'] = gh
            res_future = gh.get_result_async()
            res_future.add_done_callback(on_result)

        def on_result(future) -> None:
            r = future.result().result
            planner_state['success'] = r.success
            planner_state['message'] = r.message
            done_event.set()

        send_future = self._planner_client.send_goal_async(
            planner_goal, feedback_callback=on_feedback
        )
        send_future.add_done_callback(on_goal_response)

        # Wait for planner to finish; periodically check for cancel
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

        # 4. Map planner result back to ExecuteSeam result
        success = planner_state.get('success', False)
        msg     = planner_state.get('message', '')

        if success:
            goal_handle.succeed()
            result.success        = True
            result.message        = msg or f'Seam {seam_id} welded successfully'
            result.seam_length_mm = planner_goal.total_length * 1000.0  # m → mm
            self.get_logger().info(f'EXECUTE_SEAM [hw]: complete — {result.message}')
        else:
            goal_handle.abort()
            result.success = False
            result.message = msg or 'Planner reported failure'
            self.get_logger().error(f'EXECUTE_SEAM [hw]: FAILED — {result.message}')

        return result

    # ── Seam registry helper ──────────────────────────────────────────────────

    def _build_execute_bead_goal(
        self,
        ExecuteBead,
        seam_id: str,
        weld_speed_mm_s: float,
        wire_feed_m_min: float,
    ):
        """Build an ExecuteBead.Goal from seam registry coordinates and goal parameters."""
        entry = self._seam_registry.get(seam_id) or self._seam_registry.get('seam_01')
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
                    f'Seam {seam_id!r} not in registry — falling back to seam_01 coordinates'
                )

        start_pt = Point(x=float(start[0]), y=float(start[1]), z=float(start[2]))
        end_pt   = Point(x=float(end[0]),   y=float(end[1]),   z=float(end[2]))
        length_m = math.sqrt(
            (end[0] - start[0]) ** 2 +
            (end[1] - start[1]) ** 2 +
            (end[2] - start[2]) ** 2
        )

        goal = ExecuteBead.Goal()
        goal.bead_id                 = seam_id
        goal.plate_id                = ''
        goal.path                    = [start_pt, end_pt]
        goal.total_length            = length_m
        goal.target_speed            = weld_speed_mm_s / 1000.0   # mm/s → m/s
        goal.wire_feed_speed         = wire_feed_m_min
        goal.arc_length_correction_mm = 0.0
        goal.dry_run                 = False
        return goal


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
