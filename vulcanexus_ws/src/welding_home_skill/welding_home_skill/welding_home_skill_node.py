#!/usr/bin/env python3
"""
WeldingHomeSkillNode: lifecycle action server for MOVE_TO_HOME.

Modes (ROS 2 parameter 'use_simulation', default True):
  use_simulation=True  — lite mock: this node is the sole /joint_states publisher.
                         It relays /joint_states_manual -> /joint_states (so the seam
                         skill can drive motion), holds the last pose between goals,
                         and on MOVE_TO_HOME interpolates to HOME_RADIANS. No MoveIt.
  use_simulation=False — Gazebo/hardware: delegates to the /move_home action server
                         in robin_core (MoveItPy move to the SRDF 'home' state).

Preemption safety:
  goal_handle.is_active is checked on every loop iteration so that when two
  STOP_PROCESS intents arrive simultaneously (e.g. from a double-click), the
  first preempted goal exits cleanly without calling succeed() on an already-
  ABORTED handle (which would raise RCLError).
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import JointState

from welding_msgs.action import MoveToHome


class WeldingHomeSkillNode(LifecycleNode):
    """Skill: moves robot to home position."""

    ACTION_NAME     = 'welding_home_skill/execute'
    MOCK_DURATION_S = 3.0
    PUBLISH_HZ      = 20   # interpolation publish rate during homing
    HOLD_HZ         = 10   # idle relay rate between goals

    # UR joint names — must match robot_state_publisher / URDF
    UR_JOINTS = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
    ]

    # Home position from robin_moveit_config/srdf/ur_macro.srdf.xacro
    HOME_RADIANS = [2.763658, -1.64229, 1.7227086, -2.143056, -1.384997, -0.331205]

    # Labels used in action feedback
    JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    def __init__(self):
        super().__init__('welding_home_skill')
        self._skill_action_server: ActionServer | None = None
        self._move_home_action_client: ActionClient | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None
        self._current_moveit_goal_handle = None
        self._use_simulation: bool = True

        # Lite-mode joint-state ownership
        self._js_pub = None          # publishes to /joint_states
        self._gui_sub = None         # subscribes to /joint_states_manual (seam/GUI output)
        self._hold_timer = None      # idle relay timer
        self._held_positions: list[float] = list(self.HOME_RADIANS)
        self._homing_active: bool = False
        self._positions_lock = threading.Lock()

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.declare_parameter('use_simulation', True)
        self._use_simulation = (
            self.get_parameter('use_simulation').get_parameter_value().bool_value
        )
        mode = 'SIMULATION (lite)' if self._use_simulation else 'HARDWARE/Gazebo (MoveIt)'
        self.get_logger().info(f'welding_home_skill: configuring [{mode}]')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_home_skill: activating — advertising {self.ACTION_NAME}'
        )

        if self._use_simulation:
            # Sole /joint_states publisher in lite mode: relay /joint_states_manual
            # and hold the last pose so the robot stays put between goals.
            self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
            self._gui_sub = self.create_subscription(
                JointState, '/joint_states_manual',
                self._relay_joint_states_callback, 10,
            )
            self._hold_timer = self.create_timer(
                1.0 / self.HOLD_HZ, self._publish_held_positions
            )
        else:
            # Hardware/Gazebo: delegate to MoveIt's /move_home action.
            self._move_home_action_client = ActionClient(self, MoveToHome, '/move_home')

        # Create skill action server
        self._skill_action_server = ActionServer(
            self,
            MoveToHome,
            self.ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('welding_home_skill: deactivating')
        if self._hold_timer:
            self._hold_timer.cancel()
            self._hold_timer = None
        if self._skill_action_server:
            self._skill_action_server.destroy()
            self._skill_action_server = None
        if self._move_home_action_client:
            self._move_home_action_client.destroy()
            self._move_home_action_client = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── Lite-mode joint-state relay/hold ──────────────────────────────────────

    def _relay_joint_states_callback(self, msg: JointState) -> None:
        """Store seam/GUI joint values into _held_positions when not homing."""
        if self._homing_active:
            return
        with self._positions_lock:
            names = list(msg.name)
            for i, name in enumerate(self.UR_JOINTS):
                if name in names:
                    idx = names.index(name)
                    if idx < len(msg.position):
                        self._held_positions[i] = msg.position[idx]

    def _publish_held_positions(self) -> None:
        """Timer callback: relay the current held positions to /joint_states."""
        if self._homing_active or self._js_pub is None:
            return
        with self._positions_lock:
            positions = list(self._held_positions)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.UR_JOINTS)
        js.position = positions
        self._js_pub.publish(js)

    # ── Action server callbacks ───────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f'MOVE_TO_HOME goal received: fast_speed={goal_request.use_fast_speed}'
        )
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._current_goal_handle and self._current_goal_handle.is_active:
                self.get_logger().warning('Preempting previous home goal')
                self._current_goal_handle.abort()
            self._current_goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('MOVE_TO_HOME: cancel requested')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> MoveToHome.Result:
        if self._use_simulation:
            return self._execute_mock_home(goal_handle)
        return self._execute_moveit_home(goal_handle)

    # ── Lite-mode home interpolation ──────────────────────────────────────────

    def _execute_mock_home(self, goal_handle) -> MoveToHome.Result:
        self.get_logger().info('MOVE_TO_HOME [sim]: interpolating to home ...')
        result = MoveToHome.Result()

        if not goal_handle.is_active:
            result.success = False
            result.message = 'Goal preempted before execution started'
            return result

        with self._positions_lock:
            start_positions = list(self._held_positions)

        self._homing_active = True  # suppress relay + hold timer
        n_steps = int(self.MOCK_DURATION_S * self.PUBLISH_HZ)
        step_delay_s = 1.0 / self.PUBLISH_HZ
        n_joints = len(self.UR_JOINTS)
        feedback = MoveToHome.Feedback()

        try:
            for step in range(n_steps + 1):
                if not goal_handle.is_active:
                    result.success = False
                    result.message = 'Goal preempted'
                    return result
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal cancelled by operator'
                    return result

                alpha = step / n_steps
                positions = [
                    start_positions[j] + alpha * (self.HOME_RADIANS[j] - start_positions[j])
                    for j in range(n_joints)
                ]
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = list(self.UR_JOINTS)
                js.position = positions
                if self._js_pub is not None:
                    self._js_pub.publish(js)

                joint_idx = min(int(alpha * n_joints), n_joints - 1)
                if hasattr(feedback, 'progress_pct'):
                    feedback.progress_pct = alpha * 100.0
                if hasattr(feedback, 'current_joint'):
                    feedback.current_joint = self.JOINTS[joint_idx]
                goal_handle.publish_feedback(feedback)
                time.sleep(step_delay_s)
        finally:
            self._homing_active = False

        if not goal_handle.is_active:
            result.success = False
            result.message = 'Goal preempted'
            return result

        with self._positions_lock:
            self._held_positions = list(self.HOME_RADIANS)

        goal_handle.succeed()
        result.success = True
        result.message = 'Robot moved to home position successfully'
        self.get_logger().info('MOVE_TO_HOME [sim]: complete')
        return result

    # ── Hardware/Gazebo home via MoveIt /move_home ────────────────────────────

    def _execute_moveit_home(self, goal_handle) -> MoveToHome.Result:
        self.get_logger().info('MOVE_TO_HOME: executing via MoveIt ...')

        result = MoveToHome.Result()

        # Bail out immediately if this goal was already preempted before we started
        if not goal_handle.is_active:
            result.success = False
            result.message = 'Goal preempted before execution started'
            return result

        # Check if MoveIt action server is available
        if not self._move_home_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/move_home action server not available')
            result.success = False
            result.message = 'MoveIt /move_home action server not available'
            goal_handle.abort()
            return result

        # Send goal to MoveIt /move_home
        moveit_goal = MoveToHome.Goal()
        moveit_goal.use_fast_speed = goal_handle.request.use_fast_speed

        send_goal_future = self._move_home_action_client.send_goal_async(
            moveit_goal,
            feedback_callback=lambda fb: self._moveit_feedback_callback(
                fb, goal_handle),
        )

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        if not send_goal_future.done():
            self.get_logger().error('MOVE_TO_HOME: timed out waiting for goal acceptance')
            result.success = False
            result.message = 'Timed out sending goal to /move_home'
            goal_handle.abort()
            return result

        moveit_goal_handle = send_goal_future.result()
        if not moveit_goal_handle.accepted:
            self.get_logger().warning('MOVE_TO_HOME: MoveIt rejected goal')
            result.success = False
            result.message = '/move_home rejected the goal'
            goal_handle.abort()
            return result

        self.get_logger().info('MoveIt home goal accepted')
        self._current_moveit_goal_handle = moveit_goal_handle

        # Request the result (this returns a Future that completes when the
        # action finishes, succeeds, aborts, or is cancelled).
        result_future = moveit_goal_handle.get_result_async()

        # Poll until the result arrives, checking for cancellation
        while rclpy.ok() and not result_future.done():
            if not goal_handle.is_active:
                self.get_logger().info('MOVE_TO_HOME: preempted, cancelling MoveIt goal')
                moveit_goal_handle.cancel_goal_async()
                result.success = False
                result.message = 'Goal preempted'
                return result

            if goal_handle.is_cancel_requested:
                self.get_logger().info(
                    'MOVE_TO_HOME: cancel requested, cancelling MoveIt goal')
                moveit_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal cancelled by operator'
                return result

            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)

        self._current_moveit_goal_handle = None

        # Extract result from the GetResult response wrapper
        if not result_future.done():
            self.get_logger().error('MOVE_TO_HOME: result future not completed')
            result.success = False
            result.message = 'No result from /move_home'
            goal_handle.abort()
            return result

        try:
            moveit_response = result_future.result()
            moveit_result = moveit_response.result  # unwrap GetResult_Response
            result.success = moveit_result.success
            result.message = moveit_result.message
        except Exception as e:
            self.get_logger().error(
                f'MOVE_TO_HOME: Exception reading MoveIt result: {e}')
            result.success = False
            result.message = f'Exception: {e}'

        if result.success:
            if goal_handle.is_active:
                goal_handle.succeed()
            self.get_logger().info('MOVE_TO_HOME: complete')
        else:
            if goal_handle.is_active:
                goal_handle.abort()
            self.get_logger().error(
                f'MOVE_TO_HOME: failed — {result.message}')

        return result

    def _moveit_feedback_callback(self, feedback_msg, skill_goal_handle) -> None:
        """Forward MoveIt feedback to the skill caller."""
        if skill_goal_handle and skill_goal_handle.is_active:
            fb = feedback_msg.feedback
            skill_feedback = MoveToHome.Feedback()
            if hasattr(fb, 'progress_pct'):
                skill_feedback.progress_pct = fb.progress_pct
            if hasattr(fb, 'current_joint'):
                skill_feedback.current_joint = fb.current_joint
            skill_goal_handle.publish_feedback(skill_feedback)


def main(args=None):
    rclpy.init(args=args)
    node = WeldingHomeSkillNode()

    executor = MultiThreadedExecutor(num_threads=2)
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
