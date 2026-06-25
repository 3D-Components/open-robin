#!/usr/bin/env python3
"""
WeldingHomeSkillNode: lifecycle action server for MOVE_TO_HOME.

Uses Gazebo and MoveIt for motion control.
Calls the /move_home action server in robin_moveit_control to move to home.

Preemption safety:
  goal_handle.is_active is checked on every loop iteration so that when two
  STOP_PROCESS intents arrive simultaneously (e.g. from a double-click), the
  first preempted goal exits cleanly without calling succeed() on an already-
  ABORTED handle (which would raise RCLError).
"""
from __future__ import annotations

import threading

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

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

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_home_skill: activating — advertising {self.ACTION_NAME}'
        )

        # Create action client to call MoveIt home motion
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
