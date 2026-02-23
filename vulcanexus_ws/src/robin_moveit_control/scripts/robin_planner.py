#!/usr/bin/env python3
"""
MoveItPy ROS2 Node for UR10e robot motion planning and execution.
Provides an action server for executing weld experiments.
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from scipy.spatial.transform import Rotation

from moveit.planning import MoveItPy, PlanRequestParameters

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from robin_interfaces.action import WeldExperiment
from robin_interfaces.msg import WeldBead, ActiveBead
from robin_interfaces.srv import SetWeldingParameters, StartWeld
from std_srvs.srv import Trigger


class RobinPlanner(Node):
    """ROS2 Node for UR10e motion planning using MoveItPy."""
    
    # Configuration
    PLANNING_GROUP = "ur_manipulator"
    END_EFFECTOR_LINK = "weld_torch_tip"
    BASE_FRAME = "base_link"
    CONTROLLER_NAME = "scaled_joint_trajectory_controller"
    
    # Pilz planning pipeline configuration
    PILZ_PIPELINE = "pilz_industrial_motion_planner"
    PLANNER_PTP = "PTP"  # Point-to-point (joint space)
    PLANNER_LIN = "LIN"  # Linear Cartesian motion
    
    # Default velocity/acceleration scaling for non-weld motions
    DEFAULT_VELOCITY_SCALING = 0.3
    DEFAULT_ACCELERATION_SCALING = 0.3
    
    # Maximum Cartesian velocity from pilz_cartesian_limits.yaml (m/s)
    # This is the max_trans_vel that LIN planner uses when velocity_scaling=1.0
    MAX_CARTESIAN_VELOCITY = 0.25  # 250 mm/s as configured in pilz_cartesian_limits.yaml
    
    def __init__(self):
        super().__init__("robin_planner")
        
        # Declare parameters
        self.declare_parameter("controller_timeout", 30.0)
        self.declare_parameter("approach_height", 0.05)  # 5cm above weld start
        self.declare_parameter("torch_tilt_angle", 0.0)  # Tilt angle in degrees (0 = perpendicular to surface)
        self.declare_parameter("default_velocity_scaling", self.DEFAULT_VELOCITY_SCALING)
        self.declare_parameter("default_acceleration_scaling", self.DEFAULT_ACCELERATION_SCALING)
        self.declare_parameter("max_cartesian_velocity", self.MAX_CARTESIAN_VELOCITY)  # m/s from pilz_cartesian_limits.yaml
        
        # Get parameters
        self.controller_timeout = self.get_parameter("controller_timeout").value
        self.approach_height = self.get_parameter("approach_height").value
        self.torch_tilt_angle = math.radians(self.get_parameter("torch_tilt_angle").value)
        self.default_velocity_scaling = self.get_parameter("default_velocity_scaling").value
        self.default_acceleration_scaling = self.get_parameter("default_acceleration_scaling").value
        self.max_cartesian_velocity = self.get_parameter("max_cartesian_velocity").value
        
        self.get_logger().info("RobinPlanner node initializing...")
        
        # MoveItPy will be initialized after controller is ready
        self.robot = None
        self.arm = None
        self.planning_scene_monitor = None
        
        # Callback group for action server
        self._cb_group = ReentrantCallbackGroup()
        
        # Create WeldExperiment action server
        self._action_server = ActionServer(
            self,
            WeldExperiment,
            'weld_experiment',
            execute_callback=self._execute_weld_experiment,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group
        )
        
        self._current_goal_handle = None
        self._is_executing = False
        self._is_welding = False  # Track if arc is currently on
        
        # Service client for setting welding parameters on Fronius
        self._welding_params_client = self.create_client(
            SetWeldingParameters,
            'set_welding_parameters'
        )
        
        # Service clients for welding coordinator (start/stop arc)
        self._welding_start_client = self.create_client(
            StartWeld,
            '/welding/start'
        )
        self._welding_stop_client = self.create_client(
            Trigger,
            '/welding/stop'
        )
        
        # Publishers for data node (bead tracking with progression)
        self._active_bead_pub = self.create_publisher(
            ActiveBead, '/robin/data/active_bead', 10)
        self._welding_state_pub = self.create_publisher(
            Bool, '/robin/data/is_welding', 10)
        
        self.get_logger().info("WeldExperiment action server created")
        
    def _goal_callback(self, goal_request):
        """Accept or reject a goal request."""
        self.get_logger().info(f"Received weld experiment request with {len(goal_request.weld_beads)} beads")
        
        if self._is_executing:
            self.get_logger().warn("Rejecting goal: another experiment is in progress")
            return GoalResponse.REJECT
        
        if not self.robot:
            self.get_logger().warn("Rejecting goal: MoveIt not initialized")
            return GoalResponse.REJECT
            
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle):
        """Accept or reject a cancel request."""
        self.get_logger().info("Received cancel request")
        # Perform emergency cleanup
        self._emergency_stop()
        return CancelResponse.ACCEPT
    
    def _emergency_stop(self):
        """Emergency stop: stop welding via coordinator."""
        self.get_logger().warn("Emergency stop initiated")
        
        # Stop welding if arc is on (coordinator will handle robot_ready and gas)
        if self._is_welding:
            self.get_logger().warn("Stopping active weld...")
            self._stop_welding()
            self._is_welding = False
        
        # Clear welding state
        self._publish_welding_state(False)
    
    def _publish_active_bead(self, bead: WeldBead, weld_length: float):
        """Publish active bead info to the data node for progression tracking."""
        msg = ActiveBead()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.BASE_FRAME
        msg.bead_id = bead.bead_id
        msg.start_point = bead.start_point
        msg.end_point = bead.end_point
        msg.length = weld_length
        msg.target_speed = bead.target_speed
        msg.target_current = bead.target_current
        msg.target_voltage = bead.target_voltage
        msg.target_wire_speed = bead.wire_feed_speed
        self._active_bead_pub.publish(msg)
    
    def _publish_welding_state(self, is_welding: bool):
        """Publish the welding state (arc on/off) to the data node."""
        msg = Bool()
        msg.data = is_welding
        self._welding_state_pub.publish(msg)
    
    async def _execute_weld_experiment(self, goal_handle):
        """Execute the weld experiment action."""
        self._is_executing = True
        self._current_goal_handle = goal_handle
        
        request = goal_handle.request
        weld_beads = request.weld_beads
        total_beads = len(weld_beads)
        dry_run = request.dry_run
        
        mode_str = "DRY RUN" if dry_run else "LIVE WELDING"
        self.get_logger().info(f"Starting weld experiment ({mode_str}) with {total_beads} beads")
        
        result = WeldExperiment.Result()
        feedback = WeldExperiment.Feedback()
        
        try:
            for i, bead in enumerate(weld_beads):
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Weld experiment canceled by request")
                    # Stop welding if in progress (cancel callback may have already done this)
                    if self._is_welding:
                        self._stop_welding()
                        self._is_welding = False
                    goal_handle.canceled()
                    result.success = False
                    result.completion_percentage = (i / total_beads) * 100.0
                    # Note: finally block will handle robot_ready cleanup
                    self._is_executing = False
                    return result
                
                # Update feedback
                feedback.status = f"Executing bead {i+1}/{total_beads}"
                feedback.progress_percentage = (i / total_beads) * 100.0
                goal_handle.publish_feedback(feedback)
                
                self.get_logger().info(f"Executing weld bead {i+1}/{total_beads}")
                
                # Execute the weld bead
                success = self._execute_weld_bead(bead, dry_run=dry_run)
                
                if not success:
                    self.get_logger().error(f"Failed to execute bead {i+1}")
                    goal_handle.abort()
                    result.success = False
                    result.completion_percentage = (i / total_beads) * 100.0
                    self._is_executing = False
                    return result
                
                # Update progress
                feedback.status = f"Completed bead {i+1}/{total_beads}"
                feedback.progress_percentage = ((i + 1) / total_beads) * 100.0
                goal_handle.publish_feedback(feedback)
            
            # All beads completed successfully
            goal_handle.succeed()
            result.success = True
            result.completion_percentage = 100.0
            self.get_logger().info("Weld experiment completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Weld experiment failed: {e}")
            goal_handle.abort()
            result.success = False
            result.completion_percentage = 0.0
        
        self._is_executing = False
        return result
    
    def _execute_weld_bead(self, bead: WeldBead, dry_run: bool = False) -> bool:
        """Execute a single weld bead motion sequence.
        
        Args:
            bead: WeldBead message with start/end points and welding parameters
            dry_run: If True, execute motion only without activating welding arc
        """
        # Debug: log raw stickout value from message
        self.get_logger().info(f"DEBUG: raw bead.stickout = {bead.stickout}")
        
        # Use stickout from bead, or default to 15mm if not specified or zero
        stickout = bead.stickout if bead.stickout > 0.001 else 0.015
        
        self.get_logger().info(f"DEBUG: effective stickout = {stickout} ({stickout*1000:.1f}mm)")
        
        mode_str = "[DRY RUN] " if dry_run else ""
        self.get_logger().info(
            f"{mode_str}Weld bead: start=({bead.start_point.x:.3f}, {bead.start_point.y:.3f}, {bead.start_point.z:.3f}) "
            f"-> end=({bead.end_point.x:.3f}, {bead.end_point.y:.3f}, {bead.end_point.z:.3f})"
        )
        self.get_logger().info(
            f"{mode_str}Weld params: speed={bead.target_speed} m/s, voltage={bead.target_voltage}, "
            f"current={bead.target_current}, wire_feed={bead.wire_feed_speed}, stickout={stickout*1000:.1f}mm"
        )
        
        # Compute velocity scaling for weld motion based on target_speed
        weld_velocity_scaling = self._compute_weld_velocity_scaling(bead.target_speed)
        
        # Calculate expected weld duration for this bead
        weld_length = math.sqrt(
            (bead.end_point.x - bead.start_point.x)**2 +
            (bead.end_point.y - bead.start_point.y)**2 +
            (bead.end_point.z - bead.start_point.z)**2
        )
        expected_weld_duration = weld_length / bead.target_speed if bead.target_speed > 0 else 0
        
        self.get_logger().info(
            f"Weld velocity scaling: {weld_velocity_scaling:.3f} "
            f"(target_speed={bead.target_speed:.4f} m/s, weld_length={weld_length:.3f} m, "
            f"expected_duration={expected_weld_duration:.1f}s)"
        )
        
        # Compute orientation based on weld direction
        weld_orientation = self._compute_weld_orientation(bead.start_point, bead.end_point)
        
        # 1. Move to approach position (above start point + stickout)
        approach_pose = self._create_pose(
            bead.start_point.x,
            bead.start_point.y,
            bead.start_point.z + stickout + self.approach_height,
            orientation=weld_orientation
        )
        self.get_logger().info(f"Moving to approach position (z={bead.start_point.z + stickout + self.approach_height:.3f})...")
        if not self.move_to_pose(approach_pose):
            return False
        
        # 2. Move down to weld start position (workpiece z + stickout)
        start_pose = self._create_pose(
            bead.start_point.x,
            bead.start_point.y,
            bead.start_point.z + stickout,
            orientation=weld_orientation
        )
        self.get_logger().info(f"Moving to weld start position (z={bead.start_point.z + stickout:.3f}, stickout={stickout*1000:.1f}mm)...")
        if not self.move_to_pose(start_pose):
            return False
        
        # 3. Start welding (sets parameters + activates gas/arc)
        #    Skip if dry_run mode
        if dry_run:
            self.get_logger().info("[DRY RUN] Skipping welding start")
        else:
            # Publish active bead info for data node (enables progression tracking)
            self._publish_active_bead(bead, weld_length)
            
            if not self._start_welding(bead.target_current, bead.target_voltage, bead.wire_feed_speed):
                self.get_logger().error("Failed to start welding!")
                return False
            self._is_welding = True  # Track that arc is on
            self._publish_welding_state(True)  # Notify data node
        
        # 4. Move to weld end position using LIN (linear Cartesian motion)
        #    Use target_speed to control Cartesian velocity via velocity_scaling
        #    LIN planner ensures straight-line motion in Cartesian space
        end_pose = self._create_pose(
            bead.end_point.x,
            bead.end_point.y,
            bead.end_point.z + stickout,
            orientation=weld_orientation
        )
        self.get_logger().info(
            f"{'[DRY RUN] ' if dry_run else ''}Executing weld motion (LIN) to z={bead.end_point.z + stickout:.3f} at {bead.target_speed:.4f} m/s "
            f"(scaling={weld_velocity_scaling:.3f})..."
        )
        weld_success = self.move_to_pose(
            end_pose, 
            velocity_scaling=weld_velocity_scaling,
            planner_id=self.PLANNER_LIN  # Use LIN for linear Cartesian weld path
        )
        
        # 5. Stop welding (arc off, then gas off)
        #    Skip if dry_run mode
        if dry_run:
            self.get_logger().info("[DRY RUN] Skipping welding stop")
        else:
            if not self._stop_welding():
                self.get_logger().warn("Failed to stop welding cleanly")
            self._is_welding = False  # Track that arc is off
            self._publish_welding_state(False)  # Notify data node (clears active bead)
        
        if not weld_success:
            return False
        
        # 6. Retract to safe height (above workpiece + stickout + approach_height)
        retract_pose = self._create_pose(
            bead.end_point.x,
            bead.end_point.y,
            bead.end_point.z + stickout + self.approach_height,
            orientation=weld_orientation
        )
        self.get_logger().info(f"Retracting to z={bead.end_point.z + stickout + self.approach_height:.3f}...")
        if not self.move_to_pose(retract_pose):
            return False
        
        return True
    
    def _compute_weld_orientation(self, start: Point, end: Point) -> Quaternion:
        """Compute tool orientation based on weld direction.
        
        Uses a base orientation (tool pointing down) and rotates around Z
        to align with the weld direction.
        """
        # Weld direction vector (in XY plane)
        dx = end.x - start.x
        dy = end.y - start.y
        
        # Compute yaw angle (rotation around Z) to align with weld direction
        yaw = math.atan2(dy, dx)
        
        # Base orientation: tool pointing down, X aligned with world X
        # This is approximately [0.7071, 0.7071, 0, 0] - 180° rotation around X+Y axis
        # We'll use the reference orientation you provided and rotate it by yaw
        
        # Reference orientation for weld in +X direction (from your working config)
        # [0.72217, 0.69171, 0.0024897, -0.0015471] ≈ tool down, facing +X
        base_rot = Rotation.from_quat([0.7071, 0.7071, 0.0, 0.0])  # x, y, z, w
        
        # Rotation around world Z by yaw angle
        yaw_rot = Rotation.from_euler('z', yaw)
        
        # Combined rotation: first base orientation, then yaw
        # For rotating the tool frame, we apply yaw first in world frame
        combined_rot = yaw_rot * base_rot
        
        q = combined_rot.as_quat()  # Returns [x, y, z, w]
        
        quat = Quaternion()
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]
        
        self.get_logger().info(
            f"Weld direction yaw: {math.degrees(yaw):.1f}°, "
            f"orientation: [{quat.x:.4f}, {quat.y:.4f}, {quat.z:.4f}, {quat.w:.4f}]"
        )
        
        return quat
    
    def _set_welding_parameters(self, voltage: float, current: float, wire_feed_speed: float) -> bool:
        """Set welding parameters on the Fronius welder via service call.
        
        Args:
            voltage: Target welding voltage (V)
            current: Target welding current (A)
            wire_feed_speed: Wire feed speed (m/min)
            
        Returns:
            True if parameters were set successfully, False otherwise
        """
        if not self._welding_params_client.service_is_ready():
            self.get_logger().warn("Welding parameters service not available, waiting...")
            if not self._welding_params_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("Welding parameters service not available")
                return False
        
        request = SetWeldingParameters.Request()
        request.voltage = voltage
        request.current = current
        request.wire_speed = wire_feed_speed
        
        self.get_logger().info(
            f"Setting welding parameters: voltage={voltage:.1f}V, "
            f"current={current:.1f}A, wire_feed={wire_feed_speed:.1f}m/min"
        )
        
        try:
            future = self._welding_params_client.call_async(request)
            # Use a short timeout since we're in a synchronous context
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Welding parameters set: {response.message}")
                    return True
                else:
                    self.get_logger().error(f"Failed to set welding parameters: {response.message}")
                    return False
            else:
                self.get_logger().error("Service call timed out")
                return False
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False
    
    def _start_welding(self, current: float, voltage: float, wire_feed_speed: float) -> bool:
        """Start welding via welding coordinator.
        
        This calls /welding/start which handles the full sequence:
        1. Sets Fronius parameters (current, voltage, wire_speed)
        2. Sets robot_ready
        3. Activates gas_on
        4. Waits for gas purge (2s)
        5. Activates welding_start (arc on)
        
        The service only returns success after arc is on, so robot
        can safely begin weld motion after this call returns.
        
        Args:
            current: Target welding current (A)
            voltage: Target welding voltage (V)
            wire_feed_speed: Wire feed speed (m/min)
            
        Returns:
            True if welding started successfully, False otherwise
        """
        if not self._welding_start_client.service_is_ready():
            self.get_logger().warn("/welding/start service not available, waiting...")
            if not self._welding_start_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("/welding/start service not available")
                return False
        
        request = StartWeld.Request()
        request.current = current
        request.voltage = voltage
        request.wire_speed = wire_feed_speed
        
        self.get_logger().info(
            f"Starting welding: current={current:.1f}A, voltage={voltage:.1f}V, "
            f"wire_feed={wire_feed_speed:.1f}m/min"
        )
        
        try:
            future = self._welding_start_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Welding started: {response.message}")
                    return True
                else:
                    self.get_logger().error(f"Failed to start welding: {response.message}")
                    return False
            else:
                self.get_logger().error("Start welding service call timed out")
                return False
        except Exception as e:
            self.get_logger().error(f"Start welding service call failed: {e}")
            return False
    
    def _stop_welding(self) -> bool:
        """Stop welding via welding coordinator.
        
        This calls /welding/stop which handles the full sequence:
        1. Deactivates welding_start (arc off)
        2. Deactivates gas_on
        3. Deactivates robot_ready
        
        Returns:
            True if welding stopped successfully, False otherwise
        """
        if not self._welding_stop_client.service_is_ready():
            self.get_logger().warn("/welding/stop service not available, waiting...")
            if not self._welding_stop_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("/welding/stop service not available")
                return False
        
        request = Trigger.Request()
        
        self.get_logger().info("Stopping welding...")
        
        try:
            future = self._welding_stop_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Welding stopped: {response.message}")
                    return True
                else:
                    self.get_logger().error(f"Failed to stop welding: {response.message}")
                    return False
            else:
                self.get_logger().error("Stop welding service call timed out")
                return False
        except Exception as e:
            self.get_logger().error(f"Stop welding service call failed: {e}")
            return False
    
    def _compute_weld_velocity_scaling(self, target_speed: float) -> float:
        """Compute velocity scaling factor based on target weld speed.
        
        For Pilz LIN planner, the velocity_scaling_factor scales the Cartesian
        translational velocity defined in pilz_cartesian_limits.yaml (max_trans_vel).
        
        Args:
            target_speed: Desired weld speed in m/s
            
        Returns:
            Velocity scaling factor (0.0 to 1.0)
        """
        if target_speed <= 0:
            self.get_logger().warn(f"Invalid target_speed {target_speed}, using minimum")
            return 0.01
        
        # For Pilz LIN planner:
        # actual_velocity = max_trans_vel * velocity_scaling_factor
        # So: velocity_scaling_factor = target_speed / max_trans_vel
        scaling = target_speed / self.max_cartesian_velocity
        
        if scaling > 1.0:
            self.get_logger().warn(
                f"Target speed {target_speed:.4f} m/s exceeds max Cartesian velocity "
                f"{self.max_cartesian_velocity:.4f} m/s, clamping to 1.0"
            )
        
        # Clamp to valid range
        scaling = max(0.01, min(1.0, scaling))
        
        return scaling
    
    def _create_pose(self, x: float, y: float, z: float, orientation: Quaternion = None) -> PoseStamped:
        """Create a pose at specified position.
        
        Args:
            x, y, z: Target position
            orientation: Target orientation. If None, uses current end effector orientation.
        """
        pose = PoseStamped()
        pose.header.frame_id = self.BASE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        if orientation is not None:
            pose.pose.orientation = orientation
        else:
            current_pose = self.get_current_pose()
            pose.pose.orientation = current_pose.orientation
        
        return pose
        
    def wait_for_controller(self) -> bool:
        """Wait for the trajectory controller action server to be available."""
        self.get_logger().info(f"Waiting for {self.CONTROLLER_NAME} action server...")
        
        action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self.CONTROLLER_NAME}/follow_joint_trajectory"
        )
        
        start_time = time.time()
        while not action_client.wait_for_server(timeout_sec=1.0):
            if time.time() - start_time > self.controller_timeout:
                self.get_logger().error(f"Timeout waiting for {self.CONTROLLER_NAME}")
                return False
            self.get_logger().info(f"Still waiting for {self.CONTROLLER_NAME}...")
        
        self.get_logger().info(f"{self.CONTROLLER_NAME} is ready!")
        action_client.destroy()
        return True
    
    def initialize_moveit(self) -> bool:
        """Initialize MoveItPy after controller is ready."""
        try:
            self.robot = MoveItPy(node_name="robin_planner_moveit")
            self.arm = self.robot.get_planning_component(self.PLANNING_GROUP)
            self.planning_scene_monitor = self.robot.get_planning_scene_monitor()
            self.get_logger().info("MoveItPy instance created for UR10e")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveItPy: {e}")
            return False
    
    def get_current_pose(self):
        """Get the current end effector pose."""
        with self.planning_scene_monitor.read_only() as scene:
            robot_state = scene.current_state
            pose = robot_state.get_pose(self.END_EFFECTOR_LINK)
            return pose
    
    def plan_and_execute(self, sleep_time: float = 0.0, 
                         velocity_scaling: float = None,
                         acceleration_scaling: float = None,
                         planner_id: str = None) -> bool:
        """Plan and execute a motion using Pilz industrial motion planner.
        
        Args:
            sleep_time: Time to sleep after execution
            velocity_scaling: Velocity scaling factor (0.0-1.0). If None, uses default.
                For PTP: scales joint velocities
                For LIN: scales Cartesian translational velocity (max_trans_vel)
            acceleration_scaling: Acceleration scaling factor (0.0-1.0). If None, uses default.
            planner_id: Pilz planner to use - "PTP" (point-to-point) or "LIN" (linear Cartesian).
                If None, uses "PTP" for general motions.
        """
        # Use defaults if not specified
        vel_scale = velocity_scaling if velocity_scaling is not None else self.default_velocity_scaling
        acc_scale = acceleration_scaling if acceleration_scaling is not None else self.default_acceleration_scaling
        planner = planner_id if planner_id is not None else self.PLANNER_PTP
        
        # Create plan request parameters for Pilz planner
        # Note: PlanRequestParameters(robot, namespace) loads from ROS params with that namespace
        # We need to set the attributes explicitly after construction
        plan_params = PlanRequestParameters(
            self.robot,
            ""  # Empty namespace - we'll set attributes manually
        )
        plan_params.planning_pipeline = self.PILZ_PIPELINE  # Use Pilz planning pipeline
        plan_params.planner_id = planner  # PTP, LIN, or CIRC
        plan_params.max_velocity_scaling_factor = vel_scale
        plan_params.max_acceleration_scaling_factor = acc_scale
        
        self.get_logger().info(
            f"Planning with Pilz {planner}: pipeline={plan_params.planning_pipeline}, "
            f"vel_scale={vel_scale:.3f}, acc_scale={acc_scale:.3f}"
        )
        
        plan_result = self.arm.plan(single_plan_parameters=plan_params)
        
        if plan_result and plan_result.trajectory:
            # Log trajectory info to verify velocity scaling is applied
            traj = plan_result.trajectory
            try:
                # Get trajectory duration
                duration_sec = traj.get_duration()
                self.get_logger().info(
                    f"Planned trajectory duration: {duration_sec:.2f}s (planner={planner})"
                )
            except Exception as e:
                self.get_logger().debug(f"Could not get trajectory duration: {e}")

        if plan_result:
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])
            time.sleep(sleep_time)
            return True
        else:
            self.get_logger().error("Planning failed!")
            return False
    
    def move_to_pose(self, target_pose: PoseStamped, sleep_time: float = 0.0,
                     velocity_scaling: float = None,
                     acceleration_scaling: float = None,
                     planner_id: str = None) -> bool:
        """Move the end effector to a target pose.
        
        Args:
            target_pose: Target pose for the end effector
            sleep_time: Time to sleep after execution
            velocity_scaling: Velocity scaling factor (0.0-1.0). If None, uses default.
            acceleration_scaling: Acceleration scaling factor (0.0-1.0). If None, uses default.
            planner_id: Pilz planner - "PTP" for point-to-point, "LIN" for linear Cartesian.
        """
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(
            pose_stamped_msg=target_pose, 
            pose_link=self.END_EFFECTOR_LINK
        )
        return self.plan_and_execute(sleep_time, velocity_scaling, acceleration_scaling, planner_id)
    
    def move_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, 
                      sleep_time: float = 0.0) -> bool:
        """Move the end effector relative to current position."""
        current_pose = self.get_current_pose()
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.BASE_FRAME
        target_pose.pose.position.x = current_pose.position.x + dx
        target_pose.pose.position.y = current_pose.position.y + dy
        target_pose.pose.position.z = current_pose.position.z + dz
        target_pose.pose.orientation = current_pose.orientation
        
        return self.move_to_pose(target_pose, sleep_time)


def main(args=None):
    rclpy.init(args=args)
    
    node = RobinPlanner()
    
    # Wait for controller
    if not node.wait_for_controller():
        node.get_logger().error("Controller not available, exiting.")
        rclpy.shutdown()
        return
    
    # Initialize MoveIt
    if not node.initialize_moveit():
        node.get_logger().error("Failed to initialize MoveIt, exiting.")
        rclpy.shutdown()
        return
    
    node.get_logger().info("RobinPlanner ready - waiting for weld experiment goals on /weld_experiment")
    
    # Spin to handle action requests
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()