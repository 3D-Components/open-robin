#!/usr/bin/env python3
"""
RobinPlanner — slim MoveItPy node for single-bead motion execution.

Hosts the ExecuteBead action server: the experiment node sends per-bead
goals, and this node executes approach → calibrate → weld → scan → retract.

Calibration services are hosted by CalibrationManager (robin_calibration).

Composed helpers:
  - robin_core_planner.motion          — MoveIt Pilz planning wrapper
  - robin_core_planner.welding_client  — welding start/stop + data publishing
  - robin_core_planner.tcp_utils       — TCP mode switching + pose offset
  - robin_core_planner.bead_layout     — bead geometry helpers
  - robin_core_planner.utils           — shared helpers
  - robin_calibration.manager          — calibration service host
"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from robin_interfaces.action import ExecuteBead
from robin_interfaces.srv import CalibrateStickout, SensorCommand

from robin_core_planner.bead_layout import (
    PhysicalBead,
    compute_weld_orientation,
    compute_weld_velocity_scaling,
)
from robin_core_planner.motion import MotionPlanner
from robin_core_planner.welding_client import WeldingClient
from robin_core_planner.tcp_utils import TcpHelper
from robin_core_planner.utils import wait_for_future

from robin_calibration.manager import CalibrationManager


class RobinPlanner(Node):
    """ROS2 node for single-bead motion execution via ExecuteBead action."""

    def __init__(self):
        super().__init__("robin_planner")

        # -- Declare parameters (loaded from robin_planner_params.yaml) -----
        self.declare_parameter("controller_timeout", 30.0)
        self.declare_parameter("approach_height", 0.05)
        self.declare_parameter("default_velocity_scaling", 0.3)
        self.declare_parameter("default_acceleration_scaling", 0.3)
        self.declare_parameter("max_cartesian_velocity", 0.25)
        self.declare_parameter("default_stickout", 0.015)
        self.declare_parameter("welding_service_timeout", 5.0)
        self.declare_parameter("parameter_service_timeout", 2.0)
        self.declare_parameter("end_effector_link", "wire_tip")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter("default_bead_pitch", 0.030)
        self.declare_parameter("pre_weld_calibration", False)
        self.declare_parameter("pre_weld_calibration_retries", 1)
        self.declare_parameter("pre_weld_calibration_on_fail", "continue")
        self.declare_parameter("stickout_calibration_retract", 0.030)
        self.declare_parameter("inter_bead_clearance_height", 0.250)
        self.declare_parameter("scan_wire_retract", 0.040)
        self.declare_parameter("scan_wire_slack_compensation", 0.015)
        self.declare_parameter("scan_on_fail", "continue")
        self.declare_parameter("scan_control_laser", True)
        self.declare_parameter("scan_laser_fps", 42)

        # -- Read parameter values -------------------------------------------
        self.controller_timeout = self.get_parameter("controller_timeout").value
        self.approach_height = self.get_parameter("approach_height").value
        self.default_velocity_scaling = self.get_parameter("default_velocity_scaling").value
        self.default_acceleration_scaling = self.get_parameter("default_acceleration_scaling").value
        self.max_cartesian_velocity = self.get_parameter("max_cartesian_velocity").value
        self.default_stickout = self.get_parameter("default_stickout").value
        self.welding_service_timeout = self.get_parameter("welding_service_timeout").value
        self.parameter_service_timeout = self.get_parameter("parameter_service_timeout").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.base_frame = self.get_parameter("base_frame").value
        self.controller_name = self.get_parameter("controller_name").value
        self.default_bead_pitch = self.get_parameter("default_bead_pitch").value
        self.pre_weld_calibration = self.get_parameter("pre_weld_calibration").value
        self.pre_weld_calibration_retries = int(
            self.get_parameter("pre_weld_calibration_retries").value)
        self.pre_weld_calibration_on_fail = str(
            self.get_parameter("pre_weld_calibration_on_fail").value).lower()
        self.stickout_cal_retract = self.get_parameter("stickout_calibration_retract").value
        self.inter_bead_clearance_height = float(
            self.get_parameter("inter_bead_clearance_height").value)
        self.scan_wire_retract = float(self.get_parameter("scan_wire_retract").value)
        self.scan_wire_slack_compensation = float(
            self.get_parameter("scan_wire_slack_compensation").value)
        self.scan_on_fail = str(self.get_parameter("scan_on_fail").value).lower()
        self.scan_control_laser = bool(self.get_parameter("scan_control_laser").value)
        self.scan_laser_fps = int(self.get_parameter("scan_laser_fps").value)

        if self.scan_on_fail not in ("continue", "abort"):
            self.get_logger().warn("Invalid scan_on_fail value; using 'continue'")
            self.scan_on_fail = "continue"
        if self.pre_weld_calibration_on_fail not in ("continue", "abort"):
            self.get_logger().warn("Invalid pre_weld_calibration_on_fail; using 'continue'")
            self.pre_weld_calibration_on_fail = "continue"

        self.get_logger().info("RobinPlanner node initializing…")
        self.get_logger().info(f"  end_effector_link: {self.end_effector_link}")
        self.get_logger().info(f"  approach_height: {self.approach_height}m")
        self.get_logger().info(f"  default_stickout: {self.default_stickout*1000:.1f}mm")
        self.get_logger().info(f"  max_cartesian_velocity: {self.max_cartesian_velocity}m/s")
        self.get_logger().info(f"  pre_weld_calibration: {self.pre_weld_calibration}")

        # -- Compose helper objects ------------------------------------------
        self._cb_group = ReentrantCallbackGroup()

        self.planner = MotionPlanner()
        self.tcp = TcpHelper(self, self.default_stickout)
        self.welding = WeldingClient(
            self, self.welding_service_timeout,
            self.parameter_service_timeout, self.base_frame)

        # Calibration (robin_calibration) — hosts its own services
        self.calibration = CalibrationManager(self, self._cb_group, self.controller_name)
        self.calibration.set_dependencies(
            self.planner, self.tcp, lambda: self._is_executing)

        # Laser sensor command services (Garmo)
        self._sensor_activate_client = self.create_client(
            SensorCommand, "/profilometer_activate")
        self._sensor_deactivate_client = self.create_client(
            SensorCommand, "/profilometer_deactivate")

        # -- Execution state -------------------------------------------------
        self._is_executing = False
        self._is_welding = False
        self._terminate_requested = False
        self._simulation_mode_active = False
        self._last_bead_calibrated_stickout_m: float | None = None
        self._last_bead_applied_stickout_m: float | None = None

        # -- ExecuteBead action server (receives per-bead goals) -------------
        self._action_server = ActionServer(
            self, ExecuteBead, "execute_bead",
            execute_callback=self._execute_bead_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group)

        self.get_logger().info("ExecuteBead action server created on /execute_bead")

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
        if not self.planner.is_ready:
            self.get_logger().warn("Rejecting goal: MoveIt not initialized")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel for ExecuteBead")
        self._terminate_requested = True
        self.welding.emergency_stop(self._is_welding)
        self._is_welding = False
        self.calibration.request_abort()
        return CancelResponse.ACCEPT

    async def _execute_bead_callback(self, goal_handle):
        self._is_executing = True
        self._terminate_requested = False
        self.calibration.clear_abort()
        req = goal_handle.request

        result = ExecuteBead.Result()
        feedback = ExecuteBead.Feedback()
        simulation_mode_enabled = False

        try:
            # Simulation mode
            if self.scan_control_laser:
                self._set_laser_scan_active(False)

            if req.dry_run:
                self.get_logger().info("Dry-run: enabling PLC welding simulation mode")
                if not self.welding.set_simulation_mode(True):
                    goal_handle.abort()
                    result.success = False
                    result.message = "Failed to enable welding simulation mode"
                    return result
                simulation_mode_enabled = True
                self._simulation_mode_active = True
            else:
                if not self.welding.set_simulation_mode(False):
                    goal_handle.abort()
                    result.success = False
                    result.message = "Failed to disable welding simulation mode"
                    return result
                self._simulation_mode_active = False

            # Build PhysicalBead from goal fields
            bead = PhysicalBead(
                bead_id=req.bead_id,
                plate_id=req.plate_id,
                start_point=req.start_point,
                end_point=req.end_point,
                target_speed=req.target_speed,
                primary_parameter=req.primary_parameter,
                primary_value=req.primary_value,
                target_current=req.target_current,
                target_voltage=req.target_voltage,
                wire_feed_speed=req.wire_feed_speed,
            )

            runtime = {
                "stickout": req.requested_stickout,
                "scan_speed": req.scan_speed,
            }

            # Publish progress
            feedback.step = "starting"
            feedback.step_progress = 0.0
            goal_handle.publish_feedback(feedback)

            success = self._execute_weld_bead(
                bead, dry_run=req.dry_run, runtime=runtime,
                goal_handle=goal_handle, feedback=feedback)

            if self._terminate_requested:
                self._attempt_termination_lift(0.100)
                goal_handle.canceled()
                result.success = False
                result.message = "Canceled by experiment node"
                return result

            if success:
                goal_handle.succeed()
                result.success = True
                result.message = f"Bead {req.bead_id} completed"
                result.calibrated_stickout = float(
                    self._last_bead_calibrated_stickout_m or 0.0)
                result.applied_stickout = float(
                    self._last_bead_applied_stickout_m or 0.0)
            else:
                goal_handle.abort()
                result.success = False
                result.message = f"Bead {req.bead_id} failed"

            return result

        except Exception as e:
            self.get_logger().error(f"ExecuteBead failed: {e}")
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            return result
        finally:
            if self.scan_control_laser:
                self._set_laser_scan_active(False)
            if simulation_mode_enabled:
                self.welding.set_simulation_mode(False)
                self._simulation_mode_active = False
            self._terminate_requested = False
            self._is_executing = False

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
        requested_stickout = float(runtime.get("stickout", 0.0))
        scan_speed = float(runtime.get("scan_speed", 0.0))
        self._last_bead_calibrated_stickout_m = None
        self._last_bead_applied_stickout_m = None

        log.info(
            f"{tag}Bead {bead.bead_id}: "
            f"({bead.start_point.x:.3f},{bead.start_point.y:.3f},"
            f"{bead.start_point.z:.3f}) -> "
            f"({bead.end_point.x:.3f},{bead.end_point.y:.3f},"
            f"{bead.end_point.z:.3f})")

        weld_vel_scaling = compute_weld_velocity_scaling(
            bead.target_speed, self.max_cartesian_velocity, log)
        weld_length = math.sqrt(
            (bead.end_point.x - bead.start_point.x)**2 +
            (bead.end_point.y - bead.start_point.y)**2 +
            (bead.end_point.z - bead.start_point.z)**2)

        weld_orientation = compute_weld_orientation(
            bead.start_point, bead.end_point, log)
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
            return self._terminate_requested

        # 1. Approach via inter-bead clearance waypoint
        clearance_z = max(bead.start_point.z, bead.end_point.z) + self.inter_bead_clearance_height
        _publish_step("approach_clearance", 0.0)
        if _check_terminate():
            return False
        if not _move(_pose(bead.start_point.x, bead.start_point.y, clearance_z),
                     step_label="approach_clearance"):
            return False

        # 2. Pre-weld stickout calibration
        _publish_step("calibration", 0.1)
        if _check_terminate():
            return False

        simulation_temporarily_disabled = False
        if dry_run and self._simulation_mode_active:
            log.info("Temporarily disabling welding simulation for calibration")
            if not self.welding.set_simulation_mode(False):
                log.error("Failed to disable welding simulation before calibration")
                return False
            self._simulation_mode_active = False
            simulation_temporarily_disabled = True

        cal_ok, calibrated_stickout_m = self._run_pre_weld_calibration(
            bead, require_success=True)
        if simulation_temporarily_disabled:
            log.info("Re-enabling welding simulation after calibration")
            if not self.welding.set_simulation_mode(True):
                log.error("Failed to re-enable welding simulation")
                return False
            self._simulation_mode_active = True
        if not cal_ok:
            return False
        self._last_bead_calibrated_stickout_m = calibrated_stickout_m

        # 3. Apply requested stickout
        if requested_stickout > 0.0:
            log.info(f"Setting bead stickout to {requested_stickout * 1000:.1f}mm")
            if not self.tcp.set_stickout(requested_stickout):
                log.error("Failed to set per-bead stickout")
                return False
            self._last_bead_applied_stickout_m = requested_stickout
        else:
            self._last_bead_applied_stickout_m = calibrated_stickout_m

        # 4. Move to weld start
        _publish_step("move_to_start", 0.2)
        if _check_terminate():
            return False
        if not _move(_pose(bead.start_point.x, bead.start_point.y, bead.start_point.z),
                     step_label="move_weld_start"):
            return False

        # 5. Start welding
        _publish_step("welding", 0.3)
        if _check_terminate():
            return False
        self.welding.publish_active_bead(bead, weld_length)
        if not self.welding.start(bead):
            log.error("Failed to start welding!")
            return False
        self._is_welding = True
        self.welding.publish_welding_state(True)

        # 6. Weld motion (LIN)
        log.info(f"{tag}Weld LIN at {bead.target_speed:.4f} m/s...")
        weld_ok = _move(
            _pose(bead.end_point.x, bead.end_point.y, bead.end_point.z),
            step_label="lin_weld",
            velocity_scaling=weld_vel_scaling,
            planner_id=MotionPlanner.PLANNER_LIN)

        # 7. Stop welding
        if not self.welding.stop():
            log.warn("Failed to stop welding cleanly")
        self._is_welding = False
        self.welding.publish_welding_state(False)

        if not weld_ok:
            return False

        # 8. Retract to clearance
        _publish_step("retract", 0.7)
        if _check_terminate():
            return False
        if not _move(_pose(bead.end_point.x, bead.end_point.y, clearance_z),
                     step_label="retract_clearance"):
            return False

        # 9. Scan pass
        _publish_step("scan_pass", 0.8)
        if _check_terminate():
            return False
        scan_ok = self._execute_scan_pass(
            bead,
            scan_speed=scan_speed if scan_speed > 0.0 else bead.target_speed,
            dry_run=dry_run)

        if scan_ok:
            _publish_step("complete", 1.0)
        return scan_ok

    # =========================================================================
    # Scan pass
    # =========================================================================
    def _execute_scan_pass(self, bead: PhysicalBead, scan_speed: float,
                           dry_run: bool) -> bool:
        log = self.get_logger()
        tag = "[DRY RUN] " if dry_run else ""

        pre_scan_pull_in = self.scan_wire_retract + self.scan_wire_slack_compensation
        if pre_scan_pull_in < 0.0:
            pre_scan_pull_in = 0.0

        log.info(f"Pre-scan wire pull-in: {pre_scan_pull_in*1000:.1f}mm")
        if not self.welding.wire_retract(pre_scan_pull_in):
            log.error("Failed wire pull-in before scan pass")
            return False

        if not self.tcp.set_mode("scanning"):
            log.error("Failed to switch TCP to scanning mode")
            return False

        laser_enabled = False
        if self.scan_control_laser:
            if not self._set_laser_scan_active(True):
                log.error("Failed to activate laser before scan pass")
                return False
            laser_enabled = True

        scan_success = False
        try:
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

            self.welding.publish_active_bead(
                bead,
                math.sqrt(
                    (bead.end_point.x - bead.start_point.x) ** 2 +
                    (bead.end_point.y - bead.start_point.y) ** 2 +
                    (bead.end_point.z - bead.start_point.z) ** 2))

            scan_orientation = compute_weld_orientation(
                bead.start_point, bead.end_point, log)

            def _scan_pose(x, y, z):
                return MotionPlanner.create_pose(
                    x, y, z, self.base_frame, clock, orientation=scan_orientation)

            if not _scan_move(_scan_pose(
                bead.start_point.x, bead.start_point.y,
                bead.start_point.z + self.approach_height,
            ), step_label="approach"):
                log.warn("Scan approach failed")
            elif not _scan_move(
                _scan_pose(bead.start_point.x, bead.start_point.y, bead.start_point.z),
                step_label="move_start"):
                log.warn("Scan start failed")
            elif not _scan_move(
                _scan_pose(bead.end_point.x, bead.end_point.y, bead.end_point.z),
                velocity_scaling=scan_vel_scaling,
                planner_id=MotionPlanner.PLANNER_LIN,
                step_label="lin_pass"):
                log.warn("Scan LIN pass failed")
            else:
                clearance_z = (max(bead.start_point.z, bead.end_point.z) +
                               self.inter_bead_clearance_height)
                if not _scan_move(_scan_pose(
                    bead.end_point.x, bead.end_point.y, clearance_z,
                ), step_label="retract_clearance"):
                    log.warn("Scan retract failed")
                else:
                    scan_success = True

            if not scan_success:
                if dry_run or self.scan_on_fail == "continue":
                    log.warn(
                        f"{tag}Scan pass IK failed for bead {bead.bead_id}; "
                        "continuing by policy")
                    return True
                return False
        finally:
            if laser_enabled and not self._set_laser_scan_active(False):
                log.error("Failed to deactivate laser after scan pass")
            if not self.tcp.set_mode("welding"):
                log.error("Failed to restore TCP welding mode after scan pass")
                return False

        return scan_success

    # =========================================================================
    # Laser control
    # =========================================================================
    def _set_laser_scan_active(self, enable: bool) -> bool:
        """Enable/disable Garmo laser acquisition for scan pass only."""
        client = (self._sensor_activate_client if enable
                  else self._sensor_deactivate_client)
        action = "activate" if enable else "deactivate"
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Profilometer {action} service not available")
            return False

        req = SensorCommand.Request()
        req.cmd = ""
        req.fps = int(self.scan_laser_fps)

        for attempt in range(1, 3):
            future = client.call_async(req)
            result = wait_for_future(self, future, timeout_sec=5.0)
            if result is not None and result.success:
                self.get_logger().info(f"Laser {action}d for scanning pass")
                return True
            self.get_logger().warn(
                f"Failed to {action} laser (attempt {attempt}/2)")
        return False

    # =========================================================================
    # Pre-weld calibration
    # =========================================================================
    def _run_pre_weld_calibration(self, bead: PhysicalBead,
                                  require_success: bool = False,
                                  ) -> tuple[bool, float | None]:
        """Run pre-weld stickout calibration with retry/failure policy."""
        log = self.get_logger()
        attempts = max(1, 1 + self.pre_weld_calibration_retries)
        last_message = ""

        for attempt in range(1, attempts + 1):
            log.info(
                f"Pre-weld calibration attempt {attempt}/{attempts} at "
                f"bead {bead.bead_id}...")

            cal_req = CalibrateStickout.Request()
            cal_req.x = bead.start_point.x
            cal_req.y = bead.start_point.y
            cal_req.surface_z = bead.start_point.z
            cal_req.nozzle_height = 0.0
            cal_req.retract_length = self.stickout_cal_retract
            cal_resp = self.calibration.calibrate_stickout_impl(cal_req)

            if cal_resp.success:
                log.info(
                    f"Calibration OK: {cal_resp.measured_stickout * 1000:.2f}mm")
                return True, float(cal_resp.measured_stickout)

            last_message = cal_resp.message
            log.warn(
                f"Calibration attempt {attempt}/{attempts} failed: {last_message}")

        if require_success or self.pre_weld_calibration_on_fail == "abort":
            log.error(
                f"Calibration failed after {attempts} attempt(s): "
                f"{last_message}. Aborting bead.")
            return False, None

        log.warn(
            f"Calibration failed after {attempts} attempt(s): "
            f"{last_message}. Continuing by policy.")
        return True, None

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
    def wait_for_controller(self) -> bool:
        self.get_logger().info(f"Waiting for {self.controller_name}...")
        client = ActionClient(
            self, FollowJointTrajectory,
            f"/{self.controller_name}/follow_joint_trajectory")
        start = time.time()
        while not client.wait_for_server(timeout_sec=1.0):
            if time.time() - start > self.controller_timeout:
                self.get_logger().error(
                    f"Timeout waiting for {self.controller_name}")
                return False
            self.get_logger().info(f"Still waiting for {self.controller_name}...")
        self.get_logger().info(f"{self.controller_name} is ready!")
        client.destroy()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = RobinPlanner()

    if not node.wait_for_controller():
        node.get_logger().error("Controller not available, exiting.")
        rclpy.shutdown()
        return

    if not node.planner.initialize(node.get_logger()):
        node.get_logger().error("Failed to initialize MoveIt, exiting.")
        rclpy.shutdown()
        return

    node.get_logger().info(
        "RobinPlanner ready -- waiting for ExecuteBead goals on /execute_bead")

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
