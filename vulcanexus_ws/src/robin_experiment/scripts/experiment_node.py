#!/usr/bin/env python3
"""
ExperimentNode — ROS2 node for weld experiment lifecycle management.

Owns the WeldExperiment action server, experiment planning/approval
services, and orchestrates bead execution by sending ExecuteBead goals
to the planner node. This separation provides per-bead and per-sub-step
cancellation boundaries.

Services provided:
  /experiment/plan            — PlanExperiment
  /experiment/approve         — ApproveExperimentPlan
  /experiment/terminate       — Trigger

Action server:
  /weld_experiment            — WeldExperiment

Action client:
  /execute_bead               — ExecuteBead (talks to planner node)
"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger, SetBool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path

from robin_interfaces.action import WeldExperiment, ExecuteBead
from robin_interfaces.msg import ExperimentBead, PlateLayout
from robin_interfaces.srv import PlanExperiment, ApproveExperimentPlan

from robin_experiment.session import PlannedSession
from robin_experiment.planner import assign_beads_to_plates
from robin_experiment.bead_layout import (
    PhysicalBead,
    compute_physical_beads,
    compute_weld_orientation,
)


def _wait_for_future(node, future, timeout_sec: float = 5.0):
    """Poll-based future wait (safe with MultiThreadedExecutor)."""
    start = time.monotonic()
    while not future.done():
        if time.monotonic() - start > timeout_sec:
            node.get_logger().warn(f"Future timed out after {timeout_sec:.1f}s")
            return None
        time.sleep(0.01)
    return future.result()


class ExperimentNode(Node):
    """ROS2 node for experiment lifecycle (planning, approval, orchestration)."""

    def __init__(self):
        super().__init__("robin_experiment")

        # -- Parameters --
        self.declare_parameter("default_bead_pitch", 0.030)
        self.declare_parameter("default_margin_x", 0.015)
        self.declare_parameter("default_margin_y", 0.015)
        self.declare_parameter("default_experiment_spacing_x", 0.030)
        self.declare_parameter("default_experiment_spacing_y", 0.030)
        self.declare_parameter("default_experiment_bead_length", 0.100)
        self.declare_parameter("inter_bead_clearance_height", 0.250)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("preview_topic", "/robin/plates/markers")
        self.declare_parameter("preview_path_topic", "/robin/experiment/preview_path")
        self.declare_parameter("execute_bead_timeout", 120.0)

        self.default_bead_pitch = self.get_parameter("default_bead_pitch").value
        self.default_margin_x = self.get_parameter("default_margin_x").value
        self.default_margin_y = self.get_parameter("default_margin_y").value
        self.default_experiment_spacing_x = float(
            self.get_parameter("default_experiment_spacing_x").value)
        self.default_experiment_spacing_y = float(
            self.get_parameter("default_experiment_spacing_y").value)
        self.default_experiment_bead_length = float(
            self.get_parameter("default_experiment_bead_length").value)
        self.inter_bead_clearance_height = float(
            self.get_parameter("inter_bead_clearance_height").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.preview_topic = str(self.get_parameter("preview_topic").value)
        self.preview_path_topic = str(self.get_parameter("preview_path_topic").value)
        self.execute_bead_timeout = float(self.get_parameter("execute_bead_timeout").value)

        self.get_logger().info("ExperimentNode initializing…")

        # -- State --
        self._cb_group = ReentrantCallbackGroup()
        self._is_executing = False
        self._terminate_requested = False
        self._planned_sessions: dict[str, PlannedSession] = {}
        self._active_bead_goal_handle = None

        # -- ExecuteBead action client (talks to planner node) --
        self._execute_bead_client = ActionClient(
            self, ExecuteBead, "/execute_bead",
            callback_group=self._cb_group)

        # -- Simulation mode client --
        self._simulation_client = self.create_client(
            SetBool, "/wago/in/welding_simulation")

        # -- WeldExperiment action server --
        self._action_server = ActionServer(
            self, WeldExperiment, "weld_experiment",
            execute_callback=self._execute_weld_experiment,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group)

        # -- Experiment services --
        self.create_service(
            PlanExperiment, "/experiment/plan",
            self._plan_experiment_callback, callback_group=self._cb_group)
        self.create_service(
            ApproveExperimentPlan, "/experiment/approve",
            self._approve_experiment_callback, callback_group=self._cb_group)
        self.create_service(
            Trigger, "/experiment/terminate",
            self._terminate_experiment_callback, callback_group=self._cb_group)

        # -- Visualization --
        self._preview_pub = self.create_publisher(MarkerArray, self.preview_topic, 10)
        self._preview_path_pub = self.create_publisher(Path, self.preview_path_topic, 10)

        self.get_logger().info("WeldExperiment action server created")
        self.get_logger().info(
            "Experiment services: /experiment/plan, /experiment/approve, /experiment/terminate")

    # =========================================================================
    # Termination
    # =========================================================================
    def _terminate_experiment_callback(self, _request, response):
        if not self._is_executing:
            response.success = False
            response.message = "No experiment execution is currently active"
            return response

        self._terminate_requested = True
        # Cancel any in-flight bead execution
        if self._active_bead_goal_handle is not None:
            try:
                cancel_future = self._active_bead_goal_handle.cancel_goal_async()
                _wait_for_future(self, cancel_future, timeout_sec=2.0)
            except Exception:
                pass
        response.success = True
        response.message = "Termination requested; cancelling current bead"
        self.get_logger().warn(response.message)
        return response

    # =========================================================================
    # Action server callbacks
    # =========================================================================
    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received weld experiment request with {len(goal_request.beads)} beads "
            f"on {len(goal_request.plates)} plates")
        if self._is_executing:
            self.get_logger().warn("Rejecting goal: another experiment in progress")
            return GoalResponse.REJECT

        if goal_request.plan_id:
            session = self._planned_sessions.get(goal_request.plan_id)
            if session is None:
                self.get_logger().warn(
                    f"Rejecting goal: unknown plan_id '{goal_request.plan_id}'")
                return GoalResponse.REJECT
            if not session.approved:
                self.get_logger().warn(
                    f"Rejecting goal: plan_id '{goal_request.plan_id}' not approved")
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request for experiment")
        self._terminate_requested = True
        # Cancel in-flight bead
        if self._active_bead_goal_handle is not None:
            try:
                cancel_future = self._active_bead_goal_handle.cancel_goal_async()
                _wait_for_future(self, cancel_future, timeout_sec=2.0)
            except Exception:
                pass
        return CancelResponse.ACCEPT

    # =========================================================================
    # Planning services
    # =========================================================================
    def _plan_experiment_callback(self, request, response):
        if self._is_executing:
            response.success = False
            response.message = "Cannot plan experiment while execution is in progress"
            return response

        if not request.beads:
            response.success = False
            response.message = "No bead specs received"
            return response

        if not request.plates:
            response.success = False
            response.message = "No available plates received"
            return response

        spacing_x = float(request.spacing_x) if request.spacing_x > 0.0 else self.default_experiment_spacing_x
        spacing_y = float(request.spacing_y) if request.spacing_y > 0.0 else self.default_experiment_spacing_y
        margin_x = float(request.margin_x) if request.margin_x > 0.0 else self.default_margin_x
        margin_y = float(request.margin_y) if request.margin_y > 0.0 else self.default_margin_y

        if spacing_y <= 0.0:
            response.success = False
            response.message = "Invalid spacing_y (must be > 0)"
            return response

        assignment = assign_beads_to_plates(
            request.beads, request.plates,
            spacing_x=spacing_x,
            spacing_y=spacing_y,
            margin_x=margin_x,
            margin_y=margin_y,
            bead_length=self.default_experiment_bead_length,
            staggered=bool(request.staggered))
        if assignment is None:
            response.success = False
            response.message = "Not enough plate capacity for requested beads"
            return response

        planned_beads, bead_runtime = assignment

        layout = PlateLayout()
        layout.bead_pitch = spacing_y
        layout.margin_x = margin_x
        layout.margin_y = margin_y
        layout.spacing_x = spacing_x
        layout.spacing_y = spacing_y
        layout.staggered = bool(request.staggered)

        physical_beads = compute_physical_beads(
            planned_beads, request.plates, layout,
            self.default_bead_pitch, self.default_margin_x,
            self.default_margin_y,
            bead_length_m=self.default_experiment_bead_length,
            logger=self.get_logger())

        if not physical_beads:
            response.success = False
            response.message = "Planner produced no valid physical beads"
            return response

        timestamp = int(time.time() * 1000)
        prefix = request.experiment_id.strip() if request.experiment_id.strip() else "exp"
        plan_id = f"{prefix}-{timestamp}"

        session = PlannedSession(
            plan_id=plan_id,
            experiment_id=request.experiment_id,
            beads=planned_beads,
            plates=list(request.plates),
            layout=layout,
            physical_beads=physical_beads,
            bead_runtime=bead_runtime,
            approved=False,
        )
        self._planned_sessions[plan_id] = session

        if request.publish_preview:
            self._publish_preview(plan_id, physical_beads)

        response.success = True
        response.plan_id = plan_id
        response.message = (
            f"Planned {len(planned_beads)} beads across {len(request.plates)} plates. "
            "Awaiting explicit approval.")
        response.planned_beads = planned_beads
        response.plates = list(request.plates)
        response.layout = layout
        return response

    def _approve_experiment_callback(self, request, response):
        session = self._planned_sessions.get(request.plan_id)
        if session is None:
            response.success = False
            response.approved = False
            response.message = f"Unknown plan_id '{request.plan_id}'"
            return response

        session.approved = bool(request.approve)
        response.success = True
        response.approved = session.approved
        response.message = (
            f"Plan {request.plan_id} approved"
            if session.approved else f"Plan {request.plan_id} rejected")
        return response

    # =========================================================================
    # Experiment execution
    # =========================================================================
    async def _execute_weld_experiment(self, goal_handle):
        self._is_executing = True
        self._terminate_requested = False
        request = goal_handle.request
        dry_run = request.dry_run
        mode_str = "DRY RUN" if dry_run else "LIVE WELDING"

        session = None
        request_beads = request.beads
        request_plates = request.plates
        request_layout = request.layout
        bead_runtime = {}

        if request.plan_id:
            session = self._planned_sessions.get(request.plan_id)
            if session is None:
                result = WeldExperiment.Result()
                goal_handle.abort()
                result.success = False
                result.completion_percentage = 0.0
                result.message = f"Unknown plan_id '{request.plan_id}'"
                self._is_executing = False
                return result
            if not session.approved:
                result = WeldExperiment.Result()
                goal_handle.abort()
                result.success = False
                result.completion_percentage = 0.0
                result.message = f"Plan '{request.plan_id}' is not approved"
                self._is_executing = False
                return result
            request_beads = session.beads
            request_plates = session.plates
            request_layout = session.layout
            bead_runtime = session.bead_runtime

        self.get_logger().info(
            f"Starting weld experiment ({mode_str}) with "
            f"{len(request_beads)} beads on {len(request_plates)} plates")

        result = WeldExperiment.Result()
        feedback = WeldExperiment.Feedback()

        # Wait for planner node's ExecuteBead action server
        if not self._execute_bead_client.wait_for_server(timeout_sec=10.0):
            goal_handle.abort()
            result.success = False
            result.completion_percentage = 0.0
            result.message = "Planner node /execute_bead action not available"
            self._is_executing = False
            return result

        try:
            physical_beads = compute_physical_beads(
                request_beads, request_plates, request_layout,
                self.default_bead_pitch, self.default_margin_x,
                self.default_margin_y,
                bead_length_m=self.default_experiment_bead_length,
                logger=self.get_logger())

            if session is not None and session.physical_beads:
                physical_beads = session.physical_beads

            if not physical_beads:
                self.get_logger().error("No valid beads computed")
                goal_handle.abort()
                result.success = False
                result.completion_percentage = 0.0
                result.message = "No valid beads could be computed from plates"
                return result

            total_beads = len(physical_beads)
            completed_ids = []

            for i, bead in enumerate(physical_beads):
                # -- Termination check --
                if self._terminate_requested:
                    self.get_logger().warn("Experiment termination requested by operator")
                    goal_handle.abort()
                    result.success = False
                    result.completion_percentage = (i / total_beads) * 100.0
                    result.completed_bead_ids = completed_ids
                    result.message = "Terminated by operator"
                    return result

                # -- Cancel check --
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Experiment canceled")
                    goal_handle.canceled()
                    result.success = False
                    result.completion_percentage = (i / total_beads) * 100.0
                    result.completed_bead_ids = completed_ids
                    result.message = "Canceled by operator"
                    return result

                feedback.status = f"Executing bead {i+1}/{total_beads}: {bead.bead_id}"
                feedback.progress_percentage = (i / total_beads) * 100.0
                feedback.current_bead_id = bead.bead_id
                feedback.current_plate_id = bead.plate_id
                goal_handle.publish_feedback(feedback)

                self.get_logger().info(
                    f"Sending bead {i+1}/{total_beads}: {bead.bead_id} to planner")

                runtime = bead_runtime.get(bead.bead_id, {})
                bead_success = self._execute_single_bead(bead, dry_run, runtime)

                if not bead_success:
                    if self._terminate_requested:
                        self.get_logger().warn("Experiment terminated by operator")
                        goal_handle.abort()
                        result.success = False
                        result.completion_percentage = (i / total_beads) * 100.0
                        result.completed_bead_ids = completed_ids
                        result.message = "Terminated by operator"
                        return result

                    self.get_logger().error(f"Failed bead {bead.bead_id}")
                    goal_handle.abort()
                    result.success = False
                    result.completion_percentage = (i / total_beads) * 100.0
                    result.completed_bead_ids = completed_ids
                    result.message = f"Failed on bead {bead.bead_id}"
                    return result

                completed_ids.append(bead.bead_id)
                feedback.status = f"Completed bead {i+1}/{total_beads}: {bead.bead_id}"
                feedback.progress_percentage = ((i + 1) / total_beads) * 100.0
                goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result.success = True
            result.completion_percentage = 100.0
            result.completed_bead_ids = completed_ids
            result.message = f"All {total_beads} beads completed"
            self.get_logger().info("Weld experiment completed successfully")

        except Exception as e:
            self.get_logger().error(f"Weld experiment failed: {e}")
            goal_handle.abort()
            result.success = False
            result.completion_percentage = 0.0
            result.message = str(e)
            return result
        else:
            return result
        finally:
            self._terminate_requested = False
            self._is_executing = False

    def _execute_single_bead(self, bead: PhysicalBead, dry_run: bool,
                             runtime: dict) -> bool:
        """Send an ExecuteBead goal to the planner node and wait for result."""
        log = self.get_logger()
        goal = ExecuteBead.Goal()
        goal.bead_id = bead.bead_id
        goal.plate_id = bead.plate_id
        goal.start_point = bead.start_point
        goal.end_point = bead.end_point
        goal.target_speed = bead.target_speed
        goal.primary_parameter = bead.primary_parameter
        goal.primary_value = bead.primary_value
        goal.target_current = bead.target_current
        goal.target_voltage = bead.target_voltage
        goal.wire_feed_speed = bead.wire_feed_speed
        goal.requested_stickout = float(runtime.get("stickout", 0.0))
        goal.scan_speed = float(runtime.get("scan_speed", 0.0))
        goal.dry_run = dry_run

        send_future = self._execute_bead_client.send_goal_async(goal)
        goal_handle = _wait_for_future(self, send_future, timeout_sec=10.0)

        if goal_handle is None:
            log.error(f"ExecuteBead goal send timed out for {bead.bead_id}")
            return False
        if not goal_handle.accepted:
            log.error(f"ExecuteBead goal rejected for {bead.bead_id}")
            return False

        self._active_bead_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_wrapper = _wait_for_future(
            self, result_future, timeout_sec=self.execute_bead_timeout)

        self._active_bead_goal_handle = None

        if result_wrapper is None:
            log.error(f"ExecuteBead timed out for {bead.bead_id}")
            return False

        bead_result = result_wrapper.result
        if bead_result.success:
            log.info(f"Bead {bead.bead_id} completed: {bead_result.message}")
        else:
            log.error(f"Bead {bead.bead_id} failed: {bead_result.message}")
        return bead_result.success

    # =========================================================================
    # Visualization (preview markers)
    # =========================================================================
    @staticmethod
    def _capsule_pose_and_length(start: Point, end: Point):
        """Return (Pose, bead_length) for a CYLINDER marker."""
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        pose = Pose()
        pose.position = Point(
            x=0.5 * (start.x + end.x),
            y=0.5 * (start.y + end.y),
            z=0.5 * (start.z + end.z),
        )
        if length < 1e-9:
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            return pose, 0.0
        ux, uy, uz = dx / length, dy / length, dz / length
        dot = uz
        if dot > 1.0 - 1e-9:
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        elif dot < -1.0 + 1e-9:
            pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        else:
            cx, cy = -uy, ux
            sin_a = math.sqrt(cx * cx + cy * cy)
            ax, ay = cx / sin_a, cy / sin_a
            half = math.atan2(sin_a, dot) / 2.0
            s = math.sin(half)
            pose.orientation = Quaternion(
                x=ax * s, y=ay * s, z=0.0, w=math.cos(half)
            )
        return pose, length

    def _publish_preview(self, plan_id: str, physical_beads: list[PhysicalBead]):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        marker_id = 0
        capsule_diameter = max(0.003, self.default_bead_pitch * 0.4)

        for bead in physical_beads:
            capsule_pose, bead_length = self._capsule_pose_and_length(
                bead.start_point, bead.end_point)
            if bead_length < 1e-9:
                continue
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = now
            m.ns = "experiment_preview_beads"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose = capsule_pose
            m.scale.x = capsule_diameter
            m.scale.y = capsule_diameter
            m.scale.z = bead_length
            m.color.r = 0.2
            m.color.g = 1.0
            m.color.b = 0.2
            m.color.a = 0.85
            markers.markers.append(m)

            label = Marker()
            label.header.frame_id = self.base_frame
            label.header.stamp = now
            label.ns = "experiment_preview_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.scale.z = 0.018
            label.color.r = 0.95
            label.color.g = 0.95
            label.color.b = 0.95
            label.color.a = 1.0
            label.pose.position = Point(
                x=0.5 * (bead.start_point.x + bead.end_point.x),
                y=0.5 * (bead.start_point.y + bead.end_point.y),
                z=0.5 * (bead.start_point.z + bead.end_point.z) + 0.012,
            )
            label.text = f"{bead.bead_id}/{bead.plate_id}"
            markers.markers.append(label)

        banner = Marker()
        banner.header.frame_id = self.base_frame
        banner.header.stamp = now
        banner.ns = "experiment_preview_banner"
        banner.id = marker_id
        banner.type = Marker.TEXT_VIEW_FACING
        banner.action = Marker.ADD
        banner.scale.z = 0.025
        banner.color.r = 0.2
        banner.color.g = 0.8
        banner.color.b = 1.0
        banner.color.a = 1.0
        banner.pose.position = Point(x=0.0, y=0.0, z=0.15)
        banner.text = f"Plan: {plan_id} ({len(physical_beads)} beads)"
        markers.markers.append(banner)

        self._preview_pub.publish(markers)
        self._preview_path_pub.publish(self._build_preview_path(now, physical_beads))

    def _build_preview_path(self, stamp, physical_beads: list[PhysicalBead]) -> Path:
        preview_path = Path()
        preview_path.header.frame_id = self.base_frame
        preview_path.header.stamp = stamp

        for index, bead in enumerate(physical_beads):
            start = bead.start_point
            end = bead.end_point
            start_approach_z = start.z + self.inter_bead_clearance_height
            end_retract_z = end.z + self.inter_bead_clearance_height

            preview_path.poses.append(self._path_pose(stamp, start.x, start.y, start_approach_z))
            preview_path.poses.append(self._path_pose(stamp, start.x, start.y, start.z))
            preview_path.poses.append(self._path_pose(stamp, end.x, end.y, end.z))
            preview_path.poses.append(self._path_pose(stamp, end.x, end.y, end_retract_z))

            if index + 1 < len(physical_beads):
                nxt = physical_beads[index + 1].start_point
                preview_path.poses.append(
                    self._path_pose(
                        stamp, nxt.x, nxt.y,
                        nxt.z + self.inter_bead_clearance_height))

        return preview_path

    def _path_pose(self, stamp, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentNode()

    node.get_logger().info(
        "ExperimentNode ready — waiting for goals on /weld_experiment")

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
