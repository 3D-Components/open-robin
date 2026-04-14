"""Probe and stickout calibration — touch-sensing based surface finding."""

import time
import threading
import math

import rclpy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from controller_manager_msgs.srv import ListControllers, SwitchController

from robin_interfaces.srv import (
    FindSurface, CalibrateStickout, CalibratePlatePlane,
    SetFloat32 as SetFloat32Srv,
)

from robin_calibration.utils import wait_for_future, set_wago_signal_direct
from robin_calibration.plate_geometry import normalize_corner_id, inward_to_world


class ProbeCalibration:
    """Touch-sensing surface probing and stickout calibration.

    Owns the touch-signal subscription, trajectory action client for
    smooth probing, and the service clients for wire feed/retract.
    """

    def __init__(self, node, cb_group, controller_name: str):
        self._node = node
        self._controller_name = controller_name

        # Abort mechanism
        self._abort_event = threading.Event()

        # Touch signal state (latched)
        self._touch_signal = False
        self._touch_signal_latched = False
        self._touch_signal_lock = threading.Lock()
        self._arc_stable_signal = False

        node.create_subscription(
            Bool, "/wago/out/touch_signal", self._touch_signal_cb, 10)
        node.create_subscription(
            Bool, "/wago/out/arc_stable", self._arc_stable_cb, 10)

        # Service clients
        self._touch_probe_client = node.create_client(Trigger, "/welding/touch_probe")
        self._wire_feed_client = node.create_client(Trigger, "/welding/wire_feed_until_touch")
        self._wire_retract_client = node.create_client(SetFloat32Srv, "/welding/wire_retract")
        self._set_stickout_client = node.create_client(SetFloat32Srv, "/tcp/set_stickout")
        self._mark_stickout_calibrated_client = node.create_client(
            Trigger, "/tcp/mark_stickout_calibrated")
        self._switch_ctrl_client = node.create_client(
            SwitchController, "/controller_manager/switch_controller")
        self._list_ctrl_client = node.create_client(
            ListControllers, "/controller_manager/list_controllers")
        self._servo_pause_client = node.create_client(
            SetBool, "/servo_node/pause_servo")

        # Trajectory action client for direct controller access (smooth probe)
        self._traj_action_client = ActionClient(
            node, FollowJointTrajectory,
            f"/{controller_name}/follow_joint_trajectory",
            callback_group=cb_group)
        self._active_traj_goal_handle = None

    # -- abort support -------------------------------------------------------
    def request_abort(self):
        """Request abortion of any in-progress calibration operation."""
        self._abort_event.set()
        self._node.get_logger().warn("Calibration abort requested")
        # Cancel any in-flight trajectory
        if self._active_traj_goal_handle is not None:
            try:
                cancel_future = self._active_traj_goal_handle.cancel_goal_async()
                wait_for_future(self._node, cancel_future, timeout_sec=2.0)
            except Exception:
                pass

    def clear_abort(self):
        """Clear the abort flag before starting a new operation."""
        self._abort_event.clear()

    @property
    def is_abort_requested(self) -> bool:
        return self._abort_event.is_set()

    def _check_abort(self, context: str) -> bool:
        """Check if abort was requested. Returns True if aborted."""
        if self._abort_event.is_set():
            self._node.get_logger().warn(f"Calibration aborted during: {context}")
            return True
        return False

    # -- touch signal callbacks ----------------------------------------------
    def _touch_signal_cb(self, msg: Bool):
        with self._touch_signal_lock:
            self._touch_signal = msg.data
            if msg.data and not self._touch_signal_latched:
                self._touch_signal_latched = True
                self._node.get_logger().info("TOUCH SIGNAL detected (latched)")

    def _arc_stable_cb(self, msg: Bool):
        old = self._arc_stable_signal
        self._arc_stable_signal = msg.data
        if msg.data and not old:
            with self._touch_signal_lock:
                if not self._touch_signal_latched:
                    self._touch_signal_latched = True
                    self._node.get_logger().info(
                        "ARC_STABLE touch signal detected (latched via arc_stable)")

    def clear_touch_latch(self):
        with self._touch_signal_lock:
            self._touch_signal = False
            self._touch_signal_latched = False
        self._arc_stable_signal = False

    def _touch_state_snapshot(self) -> str:
        with self._touch_signal_lock:
            return (
                f"raw={self._touch_signal}, "
                f"latched={self._touch_signal_latched}, "
                f"arc={self._arc_stable_signal}"
            )

    def _enable_touch_sensing_mandatory(self, context: str) -> tuple[bool, str]:
        log = self._node.get_logger()
        if not self._touch_probe_client.wait_for_service(timeout_sec=2.0):
            return False, "Touch probe service not available"

        future = self._touch_probe_client.call_async(Trigger.Request())
        result = wait_for_future(self._node, future, timeout_sec=5.0)
        if result is None:
            return False, "Failed to enable touch sensing (timeout)"
        if not result.success:
            msg = result.message.strip() if result.message else "no details"
            return False, f"Failed to enable touch sensing: {msg}"

        log.info(f"Touch sensing enabled ({context})")
        return True, ""

    def _disable_touch_sensing(self, context: str):
        self._node.get_logger().info(f"Touch sensing disabled ({context})")
        set_wago_signal_direct(self._node, "/wago/in/touch_sensing", False)

    def _prepare_touch_monitoring(self, context: str):
        self.clear_touch_latch()
        self._node.get_logger().info(
            f"Touch monitor reset ({context}): {self._touch_state_snapshot()}")

    def is_touch_detected(self) -> bool:
        with self._touch_signal_lock:
            return self._touch_signal or self._touch_signal_latched

    def is_any_contact_detected(self) -> bool:
        return self.is_touch_detected()

    def _get_active_controller_names(self) -> set[str] | None:
        """Return set of currently active controller names, or None on failure."""
        if not self._list_ctrl_client.wait_for_service(timeout_sec=2.0):
            return None
        future = self._list_ctrl_client.call_async(ListControllers.Request())
        result = wait_for_future(self._node, future, timeout_sec=3.0)
        if result is None:
            return None
        return {c.name for c in result.controller if c.state == "active"}

    def _ensure_trajectory_control_mode(self) -> tuple[bool, str]:
        """Ensure calibration runs with trajectory controller, not jog velocity mode."""
        log = self._node.get_logger()

        # Always pause servo (harmless if already paused / not running)
        if self._servo_pause_client.wait_for_service(timeout_sec=1.0):
            pause_req = SetBool.Request()
            pause_req.data = True
            pause_future = self._servo_pause_client.call_async(pause_req)
            pause_result = wait_for_future(self._node, pause_future, timeout_sec=3.0)
            if pause_result is None or not pause_result.success:
                log.warn("Failed to pause servo before calibration; continuing")
        else:
            log.warn("/servo_node/pause_servo not available; continuing")

        # Query which controllers are currently active
        active = self._get_active_controller_names()
        traj_ctrl = "scaled_joint_trajectory_controller"
        vel_ctrl = "forward_velocity_controller"

        if active is not None and traj_ctrl in active and vel_ctrl not in active:
            log.info(f"{traj_ctrl} already active, skipping controller switch")
            return True, "trajectory control active (already)"

        if not self._switch_ctrl_client.wait_for_service(timeout_sec=2.0):
            return False, "controller_manager switch service not available"

        switch_req = SwitchController.Request()
        switch_req.activate_controllers = [traj_ctrl]
        if active is None or vel_ctrl in active:
            switch_req.deactivate_controllers = [vel_ctrl]
        switch_req.strictness = SwitchController.Request.BEST_EFFORT
        switch_future = self._switch_ctrl_client.call_async(switch_req)
        switch_result = wait_for_future(self._node, switch_future, timeout_sec=5.0)

        if switch_result is not None and switch_result.ok:
            return True, "trajectory control active"

        active_after = self._get_active_controller_names()
        if active_after is not None and traj_ctrl in active_after:
            log.warn(
                f"SwitchController returned ok=False but {traj_ctrl} is active "
                "(likely already-active or missing vel controller) — proceeding"
            )
            return True, "trajectory control active (verified after switch warning)"

        return False, f"failed to switch to {traj_ctrl}"

    # -- probe trajectory execution ------------------------------------------
    def execute_probe_trajectory(self, robot_trajectory, probe_speed: float) -> bool:
        """Execute a probe trajectory with real-time contact monitoring.

        Sends the MoveIt-planned trajectory to the controller, monitors
        touch sensing in a tight loop, and cancels on contact or abort.

        Returns True if contact was detected.
        """
        log = self._node.get_logger()

        if not self._traj_action_client.wait_for_server(timeout_sec=5.0):
            log.error("Trajectory action server not available!")
            return False

        try:
            robot_traj_msg = robot_trajectory.get_robot_trajectory_msg()
            traj_msg = robot_traj_msg.joint_trajectory
            log.info("Extracted JointTrajectory via get_robot_trajectory_msg()")
        except Exception as e:
            log.error(f"Cannot extract JointTrajectory: {e}")
            return False

        n_points = len(traj_msg.points)
        if n_points == 0:
            log.error("Probe trajectory has ZERO points!")
            return False

        last_point = traj_msg.points[-1]
        traj_duration = (last_point.time_from_start.sec
                         + last_point.time_from_start.nanosec * 1e-9)
        log.info(f"Probe trajectory: {n_points} points, duration={traj_duration:.2f}s, "
                 f"joints={len(traj_msg.joint_names)}")
        if traj_duration < 0.1:
            log.warn(f"Trajectory duration very short ({traj_duration:.3f}s)")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj_msg

        log.info(f"Sending probe trajectory to /{self._controller_name}/follow_joint_trajectory…")

        send_goal_future = self._traj_action_client.send_goal_async(goal)
        goal_handle = wait_for_future(self._node, send_goal_future, timeout_sec=5.0)

        if goal_handle is None:
            log.error("Probe trajectory goal send TIMED OUT")
            return False
        if not goal_handle.accepted:
            log.error("Probe trajectory goal REJECTED by controller!")
            return False

        self._active_traj_goal_handle = goal_handle
        log.info(f"Probe trajectory ACCEPTED — monitoring (expected {traj_duration:.1f}s)…")

        result_future = goal_handle.get_result_async()
        contact_detected = False
        poll_interval = 0.01
        loop_count = 0
        diag_interval = 50

        while not result_future.done():
            loop_count += 1
            if loop_count % diag_interval == 0:
                elapsed = loop_count * poll_interval
                log.info(
                    f"Probe monitor [{elapsed:.1f}s]: "
                    f"{self._touch_state_snapshot()}")

            # Check for abort
            if self._check_abort("probe_trajectory"):
                log.warn("Probe trajectory aborted by operator")
                cancel_future = goal_handle.cancel_goal_async()
                wait_for_future(self._node, cancel_future, timeout_sec=2.0)
                self._active_traj_goal_handle = None
                return False

            if self.is_any_contact_detected():
                contact_detected = True
                elapsed = loop_count * poll_interval
                log.info(f"CONTACT DETECTED at {elapsed:.2f}s — cancelling trajectory!")
                cancel_future = goal_handle.cancel_goal_async()
                wait_for_future(self._node, cancel_future, timeout_sec=2.0)
                time.sleep(0.1)
                break

            time.sleep(poll_interval)

        self._active_traj_goal_handle = None

        if contact_detected:
            log.info("Probe cancelled on contact — recording position")
        else:
            log.info(f"Probe completed ({loop_count * poll_interval:.1f}s) — NO contact")

        return contact_detected

    # -- FindSurface service implementation ----------------------------------
    def find_surface(self, request, response, planner, end_effector_link,
                     base_frame, max_cart_vel, default_acc_scaling):
        """Service callback implementation for /calibration/find_surface."""
        # Lazy imports to avoid circular dependencies at module load time
        from robin_calibration._motion_imports import compute_weld_velocity_scaling, MotionPlanner, PlanRequestParameters

        log = self._node.get_logger()
        log.info(
            f"FindSurface: probing at ({request.x:.3f}, {request.y:.3f}) "
            f"from z={request.z_start:.3f} to z_limit={request.z_limit:.3f}")

        if not planner.is_ready:
            response.success = False
            response.message = "MoveIt not initialized"
            return response

        if self._check_abort("find_surface_start"):
            response.success = False
            response.message = "Calibration aborted"
            return response

        mode_ok, mode_msg = self._ensure_trajectory_control_mode()
        if not mode_ok:
            response.success = False
            response.message = f"Cannot probe in jog mode: {mode_msg}"
            return response

        probe_speed = request.probe_speed if request.probe_speed > 0 else 0.005

        try:
            current_pose = planner.get_current_pose(end_effector_link)
            probe_orientation = current_pose.orientation
            clock = self._node.get_clock()

            start_pose = MotionPlanner.create_pose(
                request.x, request.y, request.z_start,
                base_frame, clock, orientation=probe_orientation)
            log.info(f"Moving to probe start z={request.z_start:.3f}…")
            if not planner.move_to_pose(start_pose, end_effector_link, log):
                response.success = False
                response.message = "Failed to move to probe start"
                return response

            if self._check_abort("find_surface_after_approach"):
                response.success = False
                response.message = "Calibration aborted"
                return response

            ok, err = self._enable_touch_sensing_mandatory("find_surface")
            if not ok:
                response.success = False
                response.message = err
                return response

            self._prepare_touch_monitoring("find_surface")

            limit_pose = MotionPlanner.create_pose(
                request.x, request.y, request.z_limit,
                base_frame, clock, orientation=probe_orientation)
            probe_vel_scaling = compute_weld_velocity_scaling(
                probe_speed, max_cart_vel, log)

            log.info(
                f"Planning probe LIN z={request.z_start:.3f}→{request.z_limit:.3f} "
                f"at {probe_speed:.3f} m/s (scaling={probe_vel_scaling:.3f})")

            planner.arm.set_start_state_to_current_state()
            planner.arm.set_goal_state(
                pose_stamped_msg=limit_pose, pose_link=end_effector_link)

            plan_params = PlanRequestParameters(planner.robot, "")
            plan_params.planning_pipeline = MotionPlanner.PILZ_PIPELINE
            plan_params.planner_id = MotionPlanner.PLANNER_LIN
            plan_params.max_velocity_scaling_factor = probe_vel_scaling
            plan_params.max_acceleration_scaling_factor = default_acc_scaling

            plan_result = planner.arm.plan(single_plan_parameters=plan_params)
            if not plan_result or not plan_result.trajectory:
                self._disable_touch_sensing("find_surface: planning failed")
                response.success = False
                response.message = "Failed to plan probe trajectory"
                return response

            surface_found = self.execute_probe_trajectory(
                plan_result.trajectory, probe_speed)

            self._disable_touch_sensing("find_surface: probe complete")

            if self._check_abort("find_surface_after_probe"):
                response.success = False
                response.message = "Calibration aborted"
                return response

            if surface_found:
                actual_pose = planner.get_current_pose(end_effector_link)
                response.surface_z = actual_pose.position.z
                response.success = True
                response.message = f"Surface found at z={response.surface_z:.4f}"
                log.info(response.message)
            else:
                response.success = False
                response.surface_z = 0.0
                response.message = f"No contact down to z_limit={request.z_limit:.3f}"
                log.warn(response.message)

            retract_pose = MotionPlanner.create_pose(
                request.x, request.y, request.z_start,
                base_frame, clock, orientation=probe_orientation)
            planner.move_to_pose(retract_pose, end_effector_link, log)

        except Exception as e:
            log.error(f"FindSurface failed: {e}")
            self._disable_touch_sensing("find_surface: exception")
            response.success = False
            response.message = str(e)

        return response

    def _plate_probe_points(self, request: CalibratePlatePlane.Request) -> list[tuple[float, float]]:
        corner_key = normalize_corner_id(request.corner_id)

        length = max(0.001, float(request.length))
        width = max(0.001, float(request.width))
        min_edge_margin = 0.050  # m
        min_square_size = 0.100  # m

        margin_x = max(min_edge_margin, float(request.margin_x))
        margin_y = max(min_edge_margin, float(request.margin_y))

        usable_len = length - 2.0 * margin_x
        usable_wid = width - 2.0 * margin_y
        square_size = min(usable_len, usable_wid)

        if square_size < min_square_size:
            raise ValueError(
                f"Plate too small for probing constraints: "
                f"need >= {min_square_size*1000:.0f}mm square after "
                f">= {min_edge_margin*1000:.0f}mm side margins, got "
                f"{max(0.0, square_size)*1000:.1f}mm")

        x0 = margin_x + 0.5 * (usable_len - square_size)
        y0 = margin_y + 0.5 * (usable_wid - square_size)

        inward_points = [
            (x0, y0),
            (x0 + square_size, y0),
            (x0 + square_size, y0 + square_size),
            (x0, y0 + square_size),
        ]

        base_x = float(request.corner_x)
        base_y = float(request.corner_y)
        yaw = float(request.yaw)

        world_points: list[tuple[float, float]] = []
        for ix, iy in inward_points:
            wx, wy = inward_to_world(base_x, base_y, yaw, corner_key, ix, iy)
            world_points.append((wx, wy))
        return world_points

    @staticmethod
    def _solve_3x3(a, b):
        m = [
            [float(a[0][0]), float(a[0][1]), float(a[0][2]), float(b[0])],
            [float(a[1][0]), float(a[1][1]), float(a[1][2]), float(b[1])],
            [float(a[2][0]), float(a[2][1]), float(a[2][2]), float(b[2])],
        ]
        for i in range(3):
            pivot = i
            for r in range(i + 1, 3):
                if abs(m[r][i]) > abs(m[pivot][i]):
                    pivot = r
            if abs(m[pivot][i]) < 1e-12:
                raise ValueError("Degenerate plane fit matrix")
            if pivot != i:
                m[i], m[pivot] = m[pivot], m[i]
            pivot_val = m[i][i]
            for c in range(i, 4):
                m[i][c] /= pivot_val
            for r in range(3):
                if r == i:
                    continue
                factor = m[r][i]
                for c in range(i, 4):
                    m[r][c] -= factor * m[i][c]
        return m[0][3], m[1][3], m[2][3]

    def _fit_plane(self, points_xyz: list[tuple[float, float, float]]) -> tuple[float, float, float]:
        sx = sy = sz = sxx = syy = sxy = sxz = syz = 0.0
        n = float(len(points_xyz))
        for x, y, z in points_xyz:
            sx += x
            sy += y
            sz += z
            sxx += x * x
            syy += y * y
            sxy += x * y
            sxz += x * z
            syz += y * z

        normal_a = [
            [sxx, sxy, sx],
            [sxy, syy, sy],
            [sx, sy, n],
        ]
        normal_b = [sxz, syz, sz]
        return self._solve_3x3(normal_a, normal_b)

    @staticmethod
    def _plane_residuals(points_xyz: list[tuple[float, float, float]], a: float, b: float, c: float):
        return [abs((a * x + b * y + c) - z) for x, y, z in points_xyz]

    def calibrate_plate_plane(self, request, response, planner,
                              end_effector_link, base_frame,
                              max_cart_vel, default_acc_scaling):
        log = self._node.get_logger()
        if not planner.is_ready:
            response.success = False
            response.message = "MoveIt not initialized"
            return response

        if self._check_abort("calibrate_plate_plane_start"):
            response.success = False
            response.message = "Calibration aborted"
            return response

        mode_ok, mode_msg = self._ensure_trajectory_control_mode()
        if not mode_ok:
            response.success = False
            response.message = f"Cannot calibrate plate in jog mode: {mode_msg}"
            return response

        Z_PROBE_CLEARANCE = 0.030
        try:
            table_z_limit = float(self._node.get_parameter("plate_probe_table_z_limit").value)
        except Exception:
            table_z_limit = -0.200
            log.warn(
                "Parameter 'plate_probe_table_z_limit' not available; "
                "using fallback -0.200 m")

        z_start = float(request.corner_z) + Z_PROBE_CLEARANCE
        z_limit = table_z_limit
        if z_start <= z_limit + 0.001:
            z_start = z_limit + 0.010
            log.warn(
                f"Adjusted probe start to stay above z_limit: z_start={z_start:.4f}, "
                f"z_limit={z_limit:.4f}")
        probe_speed = float(request.probe_speed) if request.probe_speed > 1e-4 else 0.015

        xy_points = self._plate_probe_points(request)
        sampled: list[tuple[float, float, float]] = []

        try:
            for idx, (px, py) in enumerate(xy_points):
                if self._check_abort(f"calibrate_plate_plane_point_{idx}"):
                    response.success = False
                    response.message = "Calibration aborted"
                    return response

                fs_req = FindSurface.Request()
                fs_req.x = float(px)
                fs_req.y = float(py)
                fs_req.z_start = z_start
                fs_req.z_limit = z_limit
                fs_req.probe_speed = probe_speed
                fs_res = FindSurface.Response()
                fs_res = self.find_surface(
                    fs_req, fs_res, planner, end_effector_link,
                    base_frame, max_cart_vel, default_acc_scaling)
                if not fs_res.success:
                    response.success = False
                    response.message = (
                        f"Probe point {idx + 1}/4 failed at ({px:.4f}, {py:.4f}): "
                        f"{fs_res.message}")
                    return response
                sampled.append((float(px), float(py), float(fs_res.surface_z)))

            plane_a, plane_b, plane_c = self._fit_plane(sampled)
            residuals = self._plane_residuals(sampled, plane_a, plane_b, plane_c)

            if len(sampled) == 4 and residuals:
                worst_idx = max(range(len(residuals)), key=lambda i: residuals[i])
                worst = residuals[worst_idx]
                sorted_res = sorted(residuals)
                median_res = 0.5 * (sorted_res[1] + sorted_res[2])
                if worst > 0.0015 and worst > 3.0 * max(1e-6, median_res):
                    refined = [p for i, p in enumerate(sampled) if i != worst_idx]
                    try:
                        plane_a, plane_b, plane_c = self._fit_plane(refined)
                        residuals = self._plane_residuals(sampled, plane_a, plane_b, plane_c)
                        log.warn(
                            f"Plate calibration: rejected outlier point {worst_idx + 1}/4 "
                            f"(residual={worst * 1000:.2f}mm)")
                    except Exception:
                        pass

            avg_z = sum(p[2] for p in sampled) / float(len(sampled))

            response.success = True
            response.message = (
                f"Plate '{request.plate_id}' calibrated with 4 points: "
                f"z={plane_a:.6f}x+{plane_b:.6f}y+{plane_c:.6f}")
            response.surface_z = float(avg_z)
            response.plane_a = float(plane_a)
            response.plane_b = float(plane_b)
            response.plane_c = float(plane_c)

            response.probe_points = []
            for x, y, z in sampled:
                pt = Point()
                pt.x = float(x)
                pt.y = float(y)
                pt.z = float(z)
                response.probe_points.append(pt)

            log.info(response.message)
            return response
        except Exception as e:
            response.success = False
            response.message = f"Plate calibration failed: {e}"
            return response

    # -- CalibrateStickout service implementation ----------------------------
    def calibrate_stickout(self, request, response, planner,
                           end_effector_link, base_frame, approach_height,
                           current_stickout, tcp_tf_buffer,
                           max_cart_vel, default_acc_scaling):
        """Service callback implementation for /calibration/calibrate_stickout."""
        # Lazy import to avoid circular dependency at module load
        from robin_calibration._motion_imports import MotionPlanner

        log = self._node.get_logger()
        log.info(
            f"CalibrateStickout: ({request.x:.3f}, {request.y:.3f}), "
            f"surface_z={request.surface_z:.4f}, "
            f"retract={request.retract_length * 1000:.1f}mm")

        if not planner.is_ready:
            response.success = False
            response.message = "MoveIt not initialized"
            return response

        if self._check_abort("calibrate_stickout_start"):
            response.success = False
            response.message = "Calibration aborted"
            return response

        mode_ok, mode_msg = self._ensure_trajectory_control_mode()
        if not mode_ok:
            response.success = False
            response.message = f"Cannot calibrate stickout in jog mode: {mode_msg}"
            return response

        retract_mm = (request.retract_length if request.retract_length > 0 else 0.030) * 1000.0
        surface_z = request.surface_z
        calibration_pose_link = "contact_tip"

        clock = self._node.get_clock()

        try:
            current_pose = planner.get_current_pose(calibration_pose_link)
            cal_orientation = current_pose.orientation

            set_wago_signal_direct(self._node, "/wago/in/robot_ready", True)

            if self._check_abort("calibrate_stickout_before_retract"):
                response.success = False
                response.message = "Calibration aborted"
                return response

            log.info(f"Retracting wire by {retract_mm:.1f}mm…")
            if self._wire_retract_client.wait_for_service(timeout_sec=2.0):
                retract_req = SetFloat32Srv.Request()
                retract_req.data = retract_mm
                future = self._wire_retract_client.call_async(retract_req)
                result = wait_for_future(self._node, future, timeout_sec=10.0)
                if result is None or not result.success:
                    log.warn("Wire retraction may have failed, continuing…")
            else:
                log.warn("Wire retract service not available, continuing…")

            if self._check_abort("calibrate_stickout_after_retract"):
                response.success = False
                response.message = "Calibration aborted"
                return response

            cal_z = surface_z + max(0.0, float(current_stickout))
            approach_z = cal_z + approach_height
            approach_pose = MotionPlanner.create_pose(
                request.x, request.y, approach_z,
                base_frame, clock, orientation=cal_orientation)
            log.info(f"Moving to calibration approach z={approach_z:.4f}…")
            if not planner.move_to_pose(approach_pose, calibration_pose_link, log):
                response.success = False
                response.message = "Failed to move to approach position"
                return response

            if self._check_abort("calibrate_stickout_after_approach"):
                response.success = False
                response.message = "Calibration aborted"
                return response

            cal_pose = MotionPlanner.create_pose(
                request.x, request.y, cal_z,
                base_frame, clock, orientation=cal_orientation)
            log.info(f"Moving contact_tip to z={cal_z:.4f}…")
            if not planner.move_to_pose(cal_pose, calibration_pose_link, log):
                response.success = False
                response.message = "Failed to move to calibration position"
                return response

            if self._check_abort("calibrate_stickout_before_feed"):
                response.success = False
                response.message = "Calibration aborted"
                return response

            log.info("Enabling touch sensing…")
            ok, err = self._enable_touch_sensing_mandatory("calibrate_stickout")
            if not ok:
                response.success = False
                response.message = err
                return response

            self._prepare_touch_monitoring("calibrate_stickout")

            log.info("Feeding wire until touch contact…")
            if self._wire_feed_client.wait_for_service(timeout_sec=2.0):
                future = self._wire_feed_client.call_async(Trigger.Request())
                result = wait_for_future(self._node, future, timeout_sec=35.0)
                if result is None or not result.success:
                    msg = result.message if result else "timeout"
                    self._disable_touch_sensing("calibrate_stickout: wire feed failed")
                    response.success = False
                    response.message = f"Wire feed until touch failed: {msg}"
                    return response
            else:
                self._disable_touch_sensing("calibrate_stickout: wire feed unavailable")
                response.success = False
                response.message = "Wire feed service not available"
                return response

            if self._check_abort("calibrate_stickout_after_feed"):
                self._disable_touch_sensing("calibrate_stickout: aborted after feed")
                response.success = False
                response.message = "Calibration aborted"
                return response

            try:
                nozzle_tf = tcp_tf_buffer.lookup_transform(
                    base_frame, "contact_tip",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0))
                nozzle_z = nozzle_tf.transform.translation.z
                measured_stickout = float(nozzle_z - surface_z)
                if measured_stickout < -0.001:
                    response.success = False
                    response.measured_stickout = 0.0
                    response.message = (
                        f"Invalid calibration pose: contact_tip below surface by "
                        f"{abs(measured_stickout) * 1000:.2f}mm")
                    self._disable_touch_sensing("calibrate_stickout: invalid pose")
                    return response
                measured_stickout = max(0.0, measured_stickout)
            except Exception as e:
                response.success = False
                response.measured_stickout = 0.0
                response.message = f"Nozzle TF lookup failed: {e}"
                self._disable_touch_sensing("calibrate_stickout: tf lookup failed")
                return response

            log.info(f"Measured stickout: {measured_stickout * 1000:.2f}mm")

            self._disable_touch_sensing("calibrate_stickout: measurement complete")

            if self._set_stickout_client.wait_for_service(timeout_sec=2.0):
                stickout_req = SetFloat32Srv.Request()
                stickout_req.data = float(measured_stickout)
                future = self._set_stickout_client.call_async(stickout_req)
                result = wait_for_future(self._node, future, timeout_sec=5.0)
                if result and result.success:
                    log.info(f"TCP manager updated: stickout={measured_stickout * 1000:.2f}mm")
                    if self._mark_stickout_calibrated_client.wait_for_service(timeout_sec=2.0):
                        future = self._mark_stickout_calibrated_client.call_async(
                            Trigger.Request())
                        wait_for_future(self._node, future, timeout_sec=5.0)
                else:
                    log.warn("Failed to update tcp_manager stickout")
            else:
                log.warn("/tcp/set_stickout service not available")

            retract_pose = MotionPlanner.create_pose(
                request.x, request.y, approach_z,
                base_frame, clock, orientation=cal_orientation)
            planner.move_to_pose(retract_pose, calibration_pose_link, log)

            response.success = True
            response.measured_stickout = measured_stickout
            response.message = f"Stickout calibrated: {measured_stickout * 1000:.2f}mm"
            log.info(response.message)

        except Exception as e:
            log.error(f"CalibrateStickout failed: {e}")
            self._disable_touch_sensing("calibrate_stickout: exception")
            response.success = False
            response.measured_stickout = 0.0
            response.message = str(e)

        return response
