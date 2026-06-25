#!/usr/bin/env python3
"""
RobinPlanner — slim MoveItPy node for single-bead motion execution.

Hosts the ExecuteBead action server: the experiment node sends per-bead
goals, and this node executes approach → calibrate → weld → scan → retract.

Calibration services are hosted by CalibrationManager (robin_core.calibration).

Composed helpers:
  - robin_core.motion          — MoveIt Pilz planning wrapper
  - robin_core.welding_client  — welding start/stop + data publishing
  - robin_core.tcp_utils       — TCP mode switching + pose offset
  - robin_core.bead_layout     — bead geometry helpers
  - robin_core.utils           — shared helpers
  - robin_core.calibration.manager          — calibration service host
"""

import time
import traceback
import threading

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State

from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Bool, String
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import PointCloud2

import tf2_ros

from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from robin_interfaces.action import ExecuteBead
from robin_interfaces.msg import ActiveBead
from robin_interfaces.srv import StartWeld, SetTcpMode, SetCtwd
from robin_interfaces.srv import SetFloat32 as SetFloat32Srv
from welding_msgs.action import MoveToHome

from robin_core.bead_layout import (
    PhysicalBead,
    compute_path_length,
    compute_weld_orientation,
    compute_weld_velocity_scaling,
)
from robin_core.motion import MotionPlanner
from robin_core.welding_client import WeldingClient
from robin_core.utils.tcp_utils import TcpHelper, LATCHED_QOS
from robin_core.utils import wait_for_future
from robin_core.ctwd_model import compute_target_ctwd_m

from robin_core.calibration.manager import CalibrationManager
from robin_core.utils.time_utils import RosTimeHealth


class RobinPlanner(Node):
    """ROS2 node for single-bead motion execution via ExecuteBead action."""

    def __init__(self):
        super().__init__(
            "robin_planner",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # -- Declare parameters (loaded from robin_planner_params.yaml) -----
        self._declare("controller_timeout", 30.0)
        self._declare("approach_height", 0.15)
        self._declare("default_velocity_scaling", 0.3)
        self._declare("default_acceleration_scaling", 0.3)
        self._declare("max_cartesian_velocity", 0.25)
        self._declare("default_ctwd", 0.015)
        self._declare("welding_service_timeout", 5.0)
        self._declare("parameter_service_timeout", 8.0)
        self._declare("end_effector_link", "wire_tip")
        self._declare("base_frame", "base_link")
        self._declare("controller_name", "scaled_joint_trajectory_controller")
        self._declare("inter_bead_clearance_height", 0.150)
        self._declare("scan_speed", 0.020)
        self._declare("scan_wire_slack", 0.005)
        self._declare("scan_wire_min_stickout", 0.010)
        self._declare("scan_on_fail", "continue")
        self._declare("scan_control_laser", True)
        self._declare("voltage_recomm_timeout", 2.0)
        self._declare("posture_lock_enabled", True)
        self._declare(
            "posture_lock_joint_names",
            ["shoulder_lift_joint", "elbow_joint", "wrist_1_joint"],
        )
        self._declare("posture_lock_tolerances_rad", [0.75, 0.75, 0.75])

        # -- Execution state -------------------------------------------------
        self._is_executing = False
        self._is_welding = False
        self._is_homing = False
        self._terminate_event = threading.Event()
        self._last_bead_calibrated_ctwd_m: float | None = None
        self._last_bead_applied_ctwd_m: float | None = None
        self._last_execute_error: str | None = None
        self._voltage_recomm_v: float | None = None
        self._voltage_recomm_stamp_ns: int = 0
        self._current_recomm_a: float | None = None
        self._last_pointcloud_monotonic: float | None = None

        # -- Read parameters into attributes --------------------------------
        for name in (
            "controller_timeout", "approach_height",
            "default_velocity_scaling", "default_acceleration_scaling",
            "max_cartesian_velocity", "default_ctwd",
            "welding_service_timeout", "parameter_service_timeout",
            "end_effector_link", "base_frame", "controller_name",
            "inter_bead_clearance_height",
            "scan_speed", "scan_wire_slack", "scan_wire_min_stickout",
            "scan_control_laser", "voltage_recomm_timeout",
            "posture_lock_enabled",
        ):
            setattr(self, name, self.get_parameter(name).value)

        self.scan_on_fail = str(self.get_parameter("scan_on_fail").value).lower()
        self.posture_lock_joint_names = list(
            self.get_parameter("posture_lock_joint_names").value
        )
        self.posture_lock_tolerances_rad = [
            float(value)
            for value in self.get_parameter("posture_lock_tolerances_rad").value
        ]

        if self.scan_on_fail not in ("continue", "abort"):
            self.get_logger().warn("Invalid scan_on_fail value; using 'continue'")
            self.scan_on_fail = "continue"
        self._cb_group = ReentrantCallbackGroup()
        self._clock_health = RosTimeHealth(
            self,
            name='robin_planner',
            stall_after_s=1.0,
            warn_interval_s=2.0,
        )
        self.planner = MotionPlanner()

        # -- Create ROS2 interfaces for TcpHelper --
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        tcp_clients = {
            'tcp_set_mode': self.create_client(
                SetTcpMode, 'tcp/set_mode',
                callback_group=self._cb_group),
            'tcp_set_ctwd': self.create_client(
                SetCtwd, 'tcp/set_ctwd',
                callback_group=self._cb_group),
        }
        self.tcp = TcpHelper(
            self, clients=tcp_clients, tf_buffer=self._tf_buffer,
            default_ctwd=self.default_ctwd)
        self.create_subscription(
            String, 'tcp/active_frame',
            self.tcp.frame_cb, LATCHED_QOS,
            callback_group=self._cb_group)
        self.create_subscription(
            Float32, 'tcp/ctwd',
            self.tcp.ctwd_cb, LATCHED_QOS,
            callback_group=self._cb_group)

        # -- Create ROS2 interfaces for WeldingClient --
        welding_clients = {
            'welding_start': self.create_client(
                StartWeld, 'welding/start',
                callback_group=self._cb_group),
            'welding_set_params': self.create_client(
                StartWeld, 'welding/set_params',
                callback_group=self._cb_group),
            'welding_stop': self.create_client(
                Trigger, 'welding/stop',
                callback_group=self._cb_group),
            'welding_wire_retract': self.create_client(
                SetFloat32Srv, 'welding/wire_retract',
                callback_group=self._cb_group),
            'welding_simulation': self.create_client(
                SetBool, 'wago/in/welding_simulation',
                callback_group=self._cb_group),
        }
        self._wire_feed_touch_client = self.create_client(
            Trigger, 'welding/wire_feed_until_touch',
            callback_group=self._cb_group)
        welding_publishers = {
            'active_bead': self.create_publisher(
                ActiveBead, 'robin/data/active_bead', LATCHED_QOS
            ),
            'is_welding': self.create_publisher(Bool, 'robin/data/is_welding', LATCHED_QOS),
            'is_scanning': self.create_publisher(Bool, 'robin/data/is_scanning', LATCHED_QOS),
        }
        self.welding = WeldingClient(
            self, clients=welding_clients, publishers=welding_publishers,
            welding_service_timeout=self.welding_service_timeout,
            parameter_service_timeout=self.parameter_service_timeout,
            base_frame=self.base_frame)

        self._sensor_change_state = self.create_client(
            ChangeState, 'garmo_sensor_node/change_state',
            callback_group=self._cb_group)
        self._sensor_get_state = self.create_client(
            GetState, 'garmo_sensor_node/get_state',
            callback_group=self._cb_group)
        self._list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers',
            callback_group=self._cb_group)
        self._cancel_goal_service = None

        self.create_subscription(
            Float32, "fronius/voltage_recommvalue",
            self._voltage_recomm_cb, 10, callback_group=self._cb_group)
        self.create_subscription(
            Float32, "fronius/current_recommvalue",
            self._current_recomm_cb, 10, callback_group=self._cb_group)
        self.create_subscription(
            PointCloud2, "robin/pointcloud",
            self._pointcloud_cb, qos_profile_sensor_data,
            callback_group=self._cb_group,
        )

        self.get_logger().info("RobinPlanner created — call start() after executor is spinning")

    def start(self):
        """Heavy initialization that requires the executor to be spinning.

        Must be called after the node is added to an executor and the
        executor is spinning in a background thread, because service
        calls (wait_for_controller, MoveIt init) need the executor to
        process responses.
        """
        if not self.wait_for_controller():
            self.get_logger().fatal("Controller not available — cannot start")
            raise RuntimeError("No trajectory controller available")

        self._cancel_goal_service = self.create_client(
            CancelGoal,
            f"/{self.controller_name}/follow_joint_trajectory/_action/cancel_goal",
            callback_group=self._cb_group,
        )

        max_moveit_retries = 3
        for attempt in range(1, max_moveit_retries + 1):
            if self.planner.initialize(self.get_logger(), node=self):
                break
            self.get_logger().warn(
                f"MoveIt init failed (attempt {attempt}/{max_moveit_retries}), "
                "retrying in 5 s...")
            time.sleep(5.0)
        else:
            self.get_logger().fatal(
                f"MoveIt failed to initialize after {max_moveit_retries} attempts")
            raise RuntimeError("MoveIt initialization failed")

        self.planner.configure_posture_lock(
            enabled=bool(self.posture_lock_enabled),
            joint_names=self.posture_lock_joint_names,
            tolerances_rad=self.posture_lock_tolerances_rad,
        )

        self.calibration = CalibrationManager(self, self._cb_group, self.controller_name)
        self.calibration.set_dependencies(
            self.planner, self.tcp, lambda: self._is_executing)
        self.calibration.create_services()

        self._action_server = ActionServer(
            self, ExecuteBead, "execute_bead",
            execute_callback=self._execute_bead_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group)

        self._move_home_action_server = ActionServer(
            self, MoveToHome, "/move_home",
            execute_callback=self._move_home_execute_callback,
            goal_callback=self._move_home_goal_callback,
            cancel_callback=self._move_home_cancel_callback,
            callback_group=self._cb_group)

        self.get_logger().info("RobinPlanner ready: MoveIt + action servers initialized")

    def _declare(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)

    def destroy_node(self):
        if hasattr(self, '_is_welding') and self._is_welding:
            self.welding.emergency_stop(True)
            self._is_welding = False
        if hasattr(self, 'planner'):
            self.planner.shutdown()
        super().destroy_node()

    # =========================================================================
    # Action server callbacks
    # =========================================================================
    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received ExecuteBead goal: bead={goal_request.bead_id} "
            f"plate={goal_request.plate_id}")
        if self._is_executing:
            self.get_logger().warn("Rejecting goal: another bead in progress")
            return GoalResponse.REJECT
        if self._is_homing:
            self.get_logger().warn("Rejecting goal: robot is moving home")
            return GoalResponse.REJECT
        if not self.planner.is_ready:
            self.get_logger().warn("Rejecting goal: MoveIt not initialized")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _move_home_goal_callback(self, goal_request):
        self.get_logger().info("Received /move_home goal")
        if self._is_homing:
            self.get_logger().warn("Rejecting /move_home: already homing")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _move_home_cancel_callback(self, goal_handle):
        self.get_logger().info("/move_home: cancel requested")
        return CancelResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel for ExecuteBead")
        self._terminate_event.set()
        # Always call stop to abort any in-progress wire operations on the
        # coordinator, even if the arc hasn't started yet.
        self.welding.stop()
        self.welding.emergency_stop(self._is_welding)
        self._is_welding = False
        self.calibration.request_abort()
        return CancelResponse.ACCEPT

    def _move_home_execute_callback(self, goal_handle) -> MoveToHome.Result:
        """Execute /move_home action: preempt any active bead execution, then move home."""
        log = self.get_logger()
        result = MoveToHome.Result()
        self._is_homing = True

        try:
            # ── 1. Preempt any in-progress bead execution ─────────────────
            if self._is_executing:
                log.warn("/move_home: preempting in-progress bead execution")

                # Signal bead callback to abort at next checkpoint
                self._terminate_event.set()
                self.calibration.request_abort()

                # Stop the welding arc immediately
                if self._is_welding:
                    log.warn("/move_home: stopping active weld")
                    self.welding.stop()
                    self.welding.emergency_stop(self._is_welding)
                    self._is_welding = False

                # Cancel the in-flight trajectory on the controller so
                # robot.execute() returns promptly instead of blocking
                # until the full trajectory completes.
                self._cancel_active_trajectories()

                # Wait for the bead callback to finish cleanup (up to 5 s).
                # Using time.sleep — this runs on an executor thread, not an
                # asyncio event loop.
                for _ in range(50):
                    if not self._is_executing:
                        break
                    time.sleep(0.1)

                if self._is_executing:
                    log.error(
                        "/move_home: bead execution did not exit within 5 s — "
                        "proceeding with home motion anyway")

            # ── 2. Check for cancellation ─────────────────────────────────
            if not goal_handle.is_active:
                result.success = False
                result.message = "Goal cancelled"
                return result

            # ── 3. Move to the SRDF "home" group state ────────────────────
            self._terminate_event.clear()
            log.info("/move_home: moving to home position")
            success = self.planner.move_to_named_target(
                "home",
                self.end_effector_link,
                log,
                default_velocity_scaling=0.3,
                default_acceleration_scaling=0.3,
            )

            if success:
                log.info("/move_home: successfully reached home")
                result.success = True
                result.message = "Robot moved to home"
                goal_handle.succeed()
            else:
                log.error("/move_home: planning/execution to home failed")
                result.success = False
                result.message = "Failed to move to home"
                goal_handle.abort()

        except Exception as e:
            log.error(f"/move_home: exception: {e}")
            result.success = False
            result.message = f"Exception: {e}"
            goal_handle.abort()
        finally:
            self._is_homing = False

        return result

    # =========================================================================
    # Trajectory cancellation for preemption
    # =========================================================================
    def _cancel_active_trajectories(self):
        """Cancel all in-flight goals on the FollowJointTrajectory controller.

        Sends a CancelGoal request with a zero UUID, which means "cancel all
        goals".  This unblocks any ongoing ``robot.execute()`` call in the bead
        pipeline so the bead callback can observe ``_terminate_event`` and exit.
        """
        log = self.get_logger()
        client = self._cancel_goal_service
        if client is None:
            log.warn("Cancel goal service not initialized — cannot cancel motion")
            return
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                log.warn("Cancel goal service not available — cannot cancel motion")
                return

        log.warn("Cancelling all active trajectory goals on controller")
        future = client.call_async(CancelGoal.Request())
        result = wait_for_future(self, future, timeout_sec=2.0)
        if result is None:
            log.warn("Trajectory cancel request timed out")
        else:
            log.info(
                f"Trajectory cancel acknowledged "
                f"({len(result.goals_canceling)} goal(s) canceling)")

    def _voltage_recomm_cb(self, msg: Float32):
        self._voltage_recomm_v = float(msg.data)
        self._voltage_recomm_stamp_ns = self.get_clock().now().nanoseconds

    def _current_recomm_cb(self, msg: Float32):
        self._current_recomm_a = float(msg.data)

    def _pointcloud_cb(self, _msg: PointCloud2):
        self._last_pointcloud_monotonic = time.monotonic()

    def _publish_idle_runtime_state(self):
        """Clear latched pass-state topics so late joiners never inherit stale state."""
        if not hasattr(self, "welding"):
            return
        self.welding.publish_welding_state(False)
        self.welding.publish_scanning_state(False)
        self.welding.publish_idle_bead()

    def _wait_for_fresh_voltage_recomm(self, min_stamp_ns: int,
                                       timeout_s: float) -> float | None:
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            self._clock_health.warn_if_unhealthy(
                active=True,
                context='waiting for a fresh Fronius recommendation during execution',
            )
            if self._terminate_event.is_set():
                return None
            if (self._voltage_recomm_v is not None
                    and self._voltage_recomm_stamp_ns >= min_stamp_ns):
                return float(self._voltage_recomm_v)
            time.sleep(0.02)
        return None

    async def _execute_bead_callback(self, goal_handle):
        self._is_executing = True
        self._terminate_event.clear()
        self._last_execute_error = None
        self.calibration.clear_abort()
        req = goal_handle.request

        result = ExecuteBead.Result()
        feedback = ExecuteBead.Feedback()
        simulation_mode_enabled = False
        success = None

        try:
            # Simulation mode
            if req.dry_run:
                self.get_logger().info("Dry-run: enabling PLC welding simulation mode")
                if not self.welding.set_simulation_mode(True):
                    result.message = "Failed to enable welding simulation mode"
                    success = False
                simulation_mode_enabled = True
            else:
                if not self.welding.set_simulation_mode(False):
                    result.message = "Failed to disable welding simulation mode"
                    success = False

            if success is not False:
                # Build PhysicalBead from goal fields
                bead = PhysicalBead(
                    bead_id=req.bead_id,
                    plate_id=req.plate_id,
                    path=list(req.path),
                    total_length=req.total_length if req.total_length > 0 else compute_path_length(req.path),
                    weld_speed=req.target_speed,
                    wire_feed_speed=req.wire_feed_speed,
                )

                runtime = {
                    "arc_length_correction_mm": req.arc_length_correction_mm,
                }

                # Publish progress
                feedback.step = "starting"
                feedback.step_progress = 0.0
                goal_handle.publish_feedback(feedback)

                success = self._execute_weld_bead(
                    bead, dry_run=req.dry_run, runtime=runtime,
                    goal_handle=goal_handle, feedback=feedback)

        except Exception as e:
            self.get_logger().error(f"ExecuteBead failed: {e!r}")
            self.get_logger().error(traceback.format_exc())
            result.message = f"ExecuteBead exception: {e!r}"

        # --- Cleanup (runs for every code path) ---
        try:
            if simulation_mode_enabled:
                if not self.welding.set_simulation_mode(False):
                    self.get_logger().error(
                        "Cleanup: failed to disable welding simulation mode")
        except Exception as cleanup_err:
            self.get_logger().error(
                f"Cleanup: simulation mode reset failed: {cleanup_err}")

        # --- Set terminal state AFTER cleanup to avoid rclpy result race ---
        # goal_handle.succeed() triggers notify_goal_done which can dispatch
        # a default Result on another executor thread before this callback
        # returns.  By deferring the state transition to here, cleanup is
        # already done and the return follows immediately.
        if self._terminate_event.is_set():
            if not self._is_homing:
                self._attempt_termination_lift(0.100)
            goal_handle.canceled()
            result.success = False
            if not result.message:
                result.message = "Canceled by experiment node"
        elif success:
            result.success = True
            result.message = f"Bead {req.bead_id} completed"
            result.calibrated_ctwd = float(
                self._last_bead_calibrated_ctwd_m or 0.0)
            result.applied_ctwd = float(
                self._last_bead_applied_ctwd_m or 0.0)
            goal_handle.succeed()
        else:
            result.success = False
            if not result.message:
                result.message = self._last_execute_error or f"Bead {req.bead_id} failed"
            goal_handle.abort()

        self._terminate_event.clear()
        self._is_executing = False
        self._publish_idle_runtime_state()
        return result

    # =========================================================================
    # Bead execution
    # =========================================================================
    def _execute_weld_bead(self, bead: PhysicalBead, dry_run: bool = False,
                           runtime: dict | None = None,
                           goal_handle=None, feedback=None) -> bool:
        """Execute a single weld bead: approach -> calibrate -> weld -> scan -> retract."""
        log = self.get_logger()
        tag = "[DRY RUN] " if dry_run else ""
        runtime = runtime or {}
        arc_length_correction_mm = float(runtime.get("arc_length_correction_mm", 0.0))
        self._last_bead_calibrated_ctwd_m = None
        self._last_bead_applied_ctwd_m = None

        path = bead.path
        if len(path) < 2:
            log.error(f"Bead {bead.bead_id}: path has < 2 waypoints")
            return False

        log.info(
            f"{tag}Bead {bead.bead_id}: "
            f"({path[0].x:.3f},{path[0].y:.3f},{path[0].z:.3f}) -> "
            f"({path[-1].x:.3f},{path[-1].y:.3f},{path[-1].z:.3f}) "
            f"[{len(path)} waypoints, {bead.total_length*1000:.1f}mm]")

        weld_vel_scaling = compute_weld_velocity_scaling(
            bead.weld_speed, self.max_cartesian_velocity, log)

        weld_orientation = compute_weld_orientation(
            path[0], path[-1], log)
        clock = self.get_clock()
        resolve = self.tcp.resolve_target

        def _pose(x, y, z, orient=weld_orientation):
            return MotionPlanner.create_pose(
                x, y, z, self.base_frame, clock, orientation=orient)

        def _move(pose, step_label: str = "", **kw):
            return self.planner.move_to_pose(
                pose, self.end_effector_link, log, resolve,
                default_velocity_scaling=self.default_velocity_scaling,
                default_acceleration_scaling=self.default_acceleration_scaling,
                move_label=f"bead={bead.bead_id} {step_label}" if step_label else "",
                **kw)

        def _publish_step(step: str, progress: float):
            if feedback is not None and goal_handle is not None:
                feedback.step = step
                feedback.step_progress = progress
                goal_handle.publish_feedback(feedback)

        def _check_terminate() -> bool:
            return self._terminate_event.is_set()

        # 1. Approach via inter-bead clearance waypoint
        max_z = max(p.z for p in path)
        clearance_z = max_z + self.inter_bead_clearance_height
        _publish_step("approach_clearance", 0.0)
        if _check_terminate():
            return False
        if not _move(_pose(path[0].x, path[0].y, clearance_z),
                     step_label="approach_clearance"):
            return False

        # 2. Preload weld params and wait for fresh Fronius recommended voltage
        _publish_step("prepare_params", 0.05)
        if _check_terminate():
            return False

        preload_start_ns = self.get_clock().now().nanoseconds
        if not self.welding.set_params(bead, arc_length_correction_mm):
            self._last_execute_error = "Failed to preload welding parameters"
            return False

        voltage_recomm_v = self._wait_for_fresh_voltage_recomm(
            preload_start_ns,
            self.voltage_recomm_timeout,
        )
        if voltage_recomm_v is None:
            self._last_execute_error = (
                "Timed out waiting for fresh /fronius/voltage_recommvalue "
                "after parameter preload"
            )
            log.error(self._last_execute_error)
            return False

        target_ctwd_m = compute_target_ctwd_m(voltage_recomm_v)
        clamped_arc_mm = max(-10.0, min(10.0, arc_length_correction_mm))
        if clamped_arc_mm != arc_length_correction_mm:
            log.warn(
                "Arc length correction clamped from "
                f"{arc_length_correction_mm:.2f}mm to {clamped_arc_mm:.2f}mm"
            )
        log.info(
            "CTWD target from Fronius voltage recommendation: "
            f"V_recomm={voltage_recomm_v:.2f}V, "
            f"target_ctwd={target_ctwd_m * 1000.0:.2f}mm, "
            f"arc_corr={clamped_arc_mm:.2f}mm (welder only)"
        )

        # 3. Apply modeled CTWD from Fronius voltage recommendation
        _publish_step("apply_ctwd", 0.1)
        if _check_terminate():
            return False

        log.info(
            f"Applying modeled target CTWD={target_ctwd_m * 1000.0:.2f}mm")
        if not self.tcp.set_ctwd(target_ctwd_m, calibrated=False):
            self._last_execute_error = "Failed to apply modeled CTWD"
            return False
        self._last_bead_applied_ctwd_m = target_ctwd_m
        self._last_bead_calibrated_ctwd_m = None

        # Compensate MoveIt target Z for CTWD change.
        # MoveIt uses the URDF model's fixed wire_tip offset (default_ctwd)
        # for IK, but the actual CTWD may differ.  Shift the target Z so
        # the physical wire tip lands at the intended plate surface.
        ctwd_z_offset = target_ctwd_m - self.default_ctwd
        if abs(ctwd_z_offset) > 1e-6:
            log.info(
                f"CTWD Z-offset compensation: {ctwd_z_offset * 1000.0:+.2f}mm "
                f"(applied={target_ctwd_m * 1000.0:.2f}mm, "
                f"URDF={self.default_ctwd * 1000.0:.2f}mm)")

        # 4. Move to weld start
        _publish_step("move_to_start", 0.2)
        if _check_terminate():
            return False
        if not _move(_pose(path[0].x, path[0].y, path[0].z + ctwd_z_offset),
                     step_label="move_weld_start"):
            self._last_execute_error = "Failed to move to weld start pose"
            return False

        # 5. Start welding
        _publish_step("welding", 0.3)
        if _check_terminate():
            return False

        self.welding.publish_active_bead(
            bead, bead.total_length,
            voltage_recommvalue=voltage_recomm_v,
            current_recommvalue=self._current_recomm_a or 0.0,
            arc_length_correction_mm=clamped_arc_mm)
        if not self.welding.start(bead, clamped_arc_mm):
            log.error("Failed to start welding!")
            self._last_execute_error = "Failed to start welding"
            return False
        self._is_welding = True
        self.welding.publish_welding_state(True)

        # 6. Weld motion — LIN through each path segment
        weld_ok = True
        for seg_idx in range(1, len(path)):
            if _check_terminate():
                weld_ok = False
                break
            wp = path[seg_idx]
            seg_orient = compute_weld_orientation(path[seg_idx - 1], wp, None)
            log.info(
                f"{tag}Weld LIN segment {seg_idx}/{len(path)-1} "
                f"at {bead.weld_speed:.4f} m/s...")
            if not _move(
                _pose(wp.x, wp.y, wp.z + ctwd_z_offset, orient=seg_orient),
                step_label=f"lin_weld_seg{seg_idx}",
                velocity_scaling=weld_vel_scaling,
                planner_id=MotionPlanner.PLANNER_LIN,
            ):
                self._last_execute_error = f"Failed during weld segment {seg_idx}"
                weld_ok = False
                break

        # 7. Stop welding
        if not self.welding.stop():
            log.warn("Failed to stop welding cleanly")
        self._is_welding = False
        self.welding.publish_welding_state(False)

        # 7b. Post-weld wire retract — pull wire back so ~5-8mm sticks out
        #     from the contact tip (enough clearance for scan, but not fully
        #     retracted to avoid the molten ball catching).
        #     retract = CTWD + slack - min_stickout
        if not dry_run and not _check_terminate():
            post_weld_retract = target_ctwd_m + self.scan_wire_slack - self.scan_wire_min_stickout
            post_weld_retract = max(0.005, post_weld_retract)  # at least 5mm
            log.info(f"Post-weld wire retract: {post_weld_retract * 1000.0:.1f}mm "
                     f"(CTWD={target_ctwd_m * 1000.0:.1f}mm)")
            if not self.welding.wire_retract(post_weld_retract):
                log.warn("Post-weld wire retract failed")

        if not weld_ok:
            return False

        # 8. Retract to clearance
        _publish_step("retract", 0.7)
        if _check_terminate():
            return False
        if not _move(_pose(path[-1].x, path[-1].y, clearance_z),
                     step_label="retract_clearance"):
            self._last_execute_error = "Failed to retract to clearance"
            return False

        # 9. Scan pass
        _publish_step("scan_pass", 0.8)
        if _check_terminate():
            return False
        scan_ok = self._execute_scan_pass(
            bead,
            scan_speed=self.scan_speed,
            dry_run=dry_run)

        if scan_ok:
            _publish_step("complete", 1.0)
        else:
            self._last_execute_error = "Scan pass failed"
        return scan_ok

    # =========================================================================
    # Scan pass
    # =========================================================================
    def _execute_scan_pass(self, bead: PhysicalBead, scan_speed: float,
                           dry_run: bool) -> bool:
        log = self.get_logger()
        tag = "[DRY RUN] " if dry_run else ""

        # Wire retract is done in step 7b (post-weld) before we get here.

        if not self.tcp.set_mode("scanning"):
            log.error("Failed to switch TCP to scanning mode")
            return False

        laser_enabled = False
        if self.scan_control_laser:
            if not self._set_laser_scan_active(True):
                log.error("Failed to activate laser before scan pass")
                return False
            laser_enabled = True
            wait_started_at = time.monotonic()
        else:
            wait_started_at = 0.0

        scan_success = False
        try:
            if laser_enabled and not self._wait_for_scan_pipeline_ready(wait_started_at):
                return False

            if self._terminate_event.is_set():
                return False

            scan_vel_scaling = compute_weld_velocity_scaling(
                scan_speed, self.max_cartesian_velocity, log)
            clock = self.get_clock()
            resolve = self.tcp.resolve_target

            def _scan_move(pose, step_label: str = "", **kw):
                return self.planner.move_to_pose(
                    pose, self.end_effector_link, log, resolve,
                    default_velocity_scaling=self.default_velocity_scaling,
                    default_acceleration_scaling=self.default_acceleration_scaling,
                    move_label=f"bead={bead.bead_id} scan {step_label}" if step_label else "",
                    **kw)

            log.info(
                f"{tag}Scan pass for bead {bead.bead_id} at "
                f"{scan_speed:.4f} m/s")

            self.welding.publish_active_bead(bead, bead.total_length)
            self.welding.publish_scanning_state(True)

            path = bead.path
            scan_orientation = compute_weld_orientation(
                path[0], path[-1], log)

            def _scan_pose(x, y, z):
                return MotionPlanner.create_pose(
                    x, y, z, self.base_frame, clock, orientation=scan_orientation)

            if self._terminate_event.is_set():
                pass  # fall through to scan_success check
            elif not _scan_move(_scan_pose(
                path[0].x, path[0].y,
                path[0].z + self.approach_height,
            ), step_label="approach"):
                log.warn("Scan approach failed")
            elif not _scan_move(
                _scan_pose(path[0].x, path[0].y, path[0].z),
                step_label="move_start"):
                log.warn("Scan start failed")
            else:
                # LIN through each path segment
                scan_lin_ok = True
                for seg_idx in range(1, len(path)):
                    if self._terminate_event.is_set():
                        scan_lin_ok = False
                        break
                    wp = path[seg_idx]
                    if not _scan_move(
                        _scan_pose(wp.x, wp.y, wp.z),
                        velocity_scaling=scan_vel_scaling,
                        planner_id=MotionPlanner.PLANNER_LIN,
                        step_label=f"lin_pass_seg{seg_idx}"):
                        log.warn(f"Scan LIN segment {seg_idx} failed")
                        scan_lin_ok = False
                        break

                if scan_lin_ok:
                    max_z = max(p.z for p in path)
                    clearance_z = max_z + self.inter_bead_clearance_height
                    if not _scan_move(_scan_pose(
                        path[-1].x, path[-1].y, clearance_z,
                    ), step_label="retract_clearance"):
                        log.warn("Scan retract failed")
                    else:
                        scan_success = True

            if not scan_success:
                if dry_run or self.scan_on_fail == "continue":
                    log.warn(
                        f"{tag}Scan pass IK failed for bead {bead.bead_id}; "
                        "continuing by policy")
                    scan_success = True
        finally:
            self.welding.publish_scanning_state(False)
            if laser_enabled and not self._set_laser_scan_active(False):
                log.error("Failed to deactivate laser after scan pass")
            if not self.tcp.set_mode("welding"):
                log.error("Failed to restore TCP welding mode after scan pass")
                scan_success = False

        return scan_success

    # =========================================================================
    # Laser control
    # =========================================================================
    def _set_laser_scan_active(self, enable: bool) -> bool:
        """Enable/disable Garmo laser via lifecycle transitions."""
        log = self.get_logger()
        action = "activate" if enable else "deactivate"

        if not self._sensor_change_state.wait_for_service(timeout_sec=2.0):
            log.error("Sensor lifecycle service not available")
            return False

        if enable:
            transitions = self._get_sensor_activate_transitions()
        else:
            transitions = [Transition.TRANSITION_DEACTIVATE]

        for tid in transitions:
            req = ChangeState.Request()
            req.transition.id = tid
            future = self._sensor_change_state.call_async(req)
            result = wait_for_future(self, future, timeout_sec=5.0)
            if result is None or not result.success:
                log.error(f"Sensor {action} failed at transition {tid}")
                return False

        log.info(f"Laser {action}d for scanning pass")
        return True

    def _get_sensor_state_id(self) -> int | None:
        """Return the current lifecycle state id for the scan sensor."""
        if not self._sensor_get_state.wait_for_service(timeout_sec=2.0):
            return None

        future = self._sensor_get_state.call_async(GetState.Request())
        result = wait_for_future(self, future, timeout_sec=2.0)
        if result is None:
            return None
        return int(result.current_state.id)

    def _wait_for_scan_pipeline_ready(self, activated_at: float) -> bool:
        """Wait for the sensor to report ACTIVE and publish fresh pointcloud data."""
        log = self.get_logger()
        deadline = time.monotonic() + 8.0
        sensor_active = False
        pointcloud_ready = False

        while time.monotonic() < deadline:
            self._clock_health.warn_if_unhealthy(
                active=True,
                context='waiting for scan pipeline readiness',
            )
            state_id = self._get_sensor_state_id()
            sensor_active = state_id == State.PRIMARY_STATE_ACTIVE
            pointcloud_ready = (
                self._last_pointcloud_monotonic is not None
                and self._last_pointcloud_monotonic >= activated_at
            )
            if sensor_active and pointcloud_ready:
                log.info("Scan pipeline ready: sensor active and pointcloud streaming")
                return True
            time.sleep(0.05)

        if not sensor_active:
            log.error("Scan pipeline not ready: sensor never reached ACTIVE")
        elif not pointcloud_ready:
            clock_issue = self._clock_health.issue(active=True)
            if clock_issue is None:
                log.error("Scan pipeline not ready: no fresh /robin/pointcloud received")
            else:
                log.error(
                    "Scan pipeline not ready: no fresh /robin/pointcloud received "
                    f'while {clock_issue.lower()}'
                )
        return False

    def _get_sensor_activate_transitions(self) -> list:
        """Return the transitions needed to reach active from current state."""
        if not self._sensor_get_state.wait_for_service(timeout_sec=2.0):
            return [Transition.TRANSITION_ACTIVATE]

        future = self._sensor_get_state.call_async(GetState.Request())
        result = wait_for_future(self, future, timeout_sec=2.0)
        if result is None:
            # State unknown — assume already configured (lifecycle_manager does
            # that on startup).  Worst case ACTIVATE fails and we retry with
            # CONFIGURE+ACTIVATE.
            return [Transition.TRANSITION_ACTIVATE]

        sid = result.current_state.id
        if sid == State.PRIMARY_STATE_UNCONFIGURED:
            return [Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]
        elif sid == State.PRIMARY_STATE_INACTIVE:
            return [Transition.TRANSITION_ACTIVATE]
        elif sid == State.PRIMARY_STATE_ACTIVE:
            return []
        return [Transition.TRANSITION_ACTIVATE]

    # =========================================================================
    # Termination lift
    # =========================================================================
    def _attempt_termination_lift(self, dz_m: float) -> bool:
        """Best-effort upward retreat after operator termination."""
        log = self.get_logger()
        log.warn(f"Attempting termination retreat: +{dz_m * 1000.0:.0f}mm in Z")
        try:
            return self.planner.move_relative(
                0.0, 0.0, float(dz_m),
                self.end_effector_link, self.base_frame, log,
                resolve_tcp_fn=self.tcp.resolve_target,
                default_velocity_scaling=min(0.2, self.default_velocity_scaling),
                default_acceleration_scaling=min(0.2, self.default_acceleration_scaling))
        except Exception as e:
            log.error(f"Termination retreat failed: {e}")
            return False

    # =========================================================================
    # Startup
    # =========================================================================
    def _list_trajectory_controller_states(self) -> dict[str, str]:
        """Return known trajectory controller states from controller_manager."""
        if not self._list_controllers_client.wait_for_service(timeout_sec=1.0):
            return {}

        future = self._list_controllers_client.call_async(ListControllers.Request())
        response = wait_for_future(self, future, timeout_sec=2.0)
        if response is None:
            return {}

        states = {}
        for controller in response.controller:
            if "trajectory" not in controller.name:
                continue
            states[controller.name] = controller.state
        return states

    @staticmethod
    def _ordered_controller_candidates(
        requested_controller: str,
        controller_states: dict[str, str],
    ) -> list[str]:
        ordered: list[str] = []

        def add(name: str):
            if name and name not in ordered:
                ordered.append(name)

        add(requested_controller)

        for name in sorted(controller_states):
            if controller_states[name] == "active":
                add(name)

        add("scaled_joint_trajectory_controller")
        add("joint_trajectory_controller")

        for name in sorted(controller_states):
            add(name)

        return ordered

    def wait_for_controller(self) -> bool:
        """Block until a trajectory controller's action server is reachable.

        Two-phase check per candidate:
          1. controller_manager reports the controller as "active"
          2. The FollowJointTrajectory action server is discoverable

        Uses a single persistent ActionClient per candidate (kept alive
        until the node is destroyed) to avoid the race condition where
        destroying a temporary client while the executor is spinning
        causes an RCLError / segfault.

        Returns True if found within ``controller_timeout``, False otherwise.
        """
        log = self.get_logger()
        requested_controller = self.controller_name
        log.info(
            f"Waiting for trajectory controller (requested: "
            f"{requested_controller})...")

        deadline = time.monotonic() + self.controller_timeout
        last_wait_log = 0.0
        # Cache probe clients so we never destroy them while the
        # executor is spinning — they live until node shutdown.
        probe_clients: dict[str, ActionClient] = {}

        while time.monotonic() < deadline:
            controller_states = self._list_trajectory_controller_states()
            candidates = self._ordered_controller_candidates(
                requested_controller, controller_states)

            for candidate in candidates:
                state = controller_states.get(candidate, "unknown")
                if state != "active":
                    continue

                # Phase 2: verify the action server is actually reachable
                if candidate not in probe_clients:
                    probe_clients[candidate] = ActionClient(
                        self,
                        FollowJointTrajectory,
                        f"/{candidate}/follow_joint_trajectory",
                    )
                if not probe_clients[candidate].wait_for_server(
                    timeout_sec=1.0
                ):
                    continue

                if candidate != requested_controller:
                    log.warn(
                        f"Requested controller '{requested_controller}' is not "
                        f"ready; using '{candidate}' instead (state={state})")
                self.controller_name = candidate
                log.info(f"{self.controller_name} is ready!")
                return True

            now = time.monotonic()
            if now - last_wait_log >= 5.0:
                if controller_states:
                    summary = ", ".join(
                        f"{name}={state}"
                        for name, state in sorted(controller_states.items())
                    )
                    log.info(
                        f"Still waiting for trajectory controller; "
                        f"controller_manager reports: {summary}")
                else:
                    log.info(
                        "Still waiting for trajectory controller; "
                        "controller_manager has not reported any usable "
                        "trajectory controllers yet")
                last_wait_log = now

            time.sleep(0.5)

        log.error(
            f"No trajectory controller became available within "
            f"{self.controller_timeout:.0f}s "
            f"(requested: {requested_controller})")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = RobinPlanner()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # Spin in background so service calls work during node.start()
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.start()  # blocks until controller + MoveIt are ready
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
