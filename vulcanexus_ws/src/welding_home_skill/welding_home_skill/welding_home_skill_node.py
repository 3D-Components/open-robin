#!/usr/bin/env python3
"""
WeldingHomeSkillNode: lifecycle action server for MOVE_TO_HOME.

ros4hri concept: maps to motion_skills/action/ExecuteJointTrajectory.

Modes (controlled by ROS2 parameter 'use_simulation', default True):
  use_simulation=True  — relay/override strategy:
    * Subscribes to /joint_states_manual (joint_state_publisher_gui output,
      remapped in the launch file so it doesn't fight /joint_states directly).
    * A 10 Hz hold timer relays those slider values to /joint_states, so the
      GUI controls the robot normally.
    * On a MOVE_TO_HOME goal: homing_active=True suppresses the relay;
      the execute callback publishes interpolated positions at 20 Hz,
      animating from the current arm pose to HOME_RADIANS over 3 s.
    * After homing: held_positions is set to HOME_RADIANS; the relay
      resumes and holds home until the user moves the GUI sliders again.
  use_simulation=False — hardware mode: currently falls back to mock with a warning.
                         Production integration requires a /move_home action server
                         in robin_moveit_control (MoveItPy set_named_target('home')).

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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
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
    HOME_RADIANS = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]

    # Labels used in action feedback
    JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    def __init__(self):
        super().__init__('welding_home_skill')
        self._action_server: ActionServer | None = None
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None
        self._use_simulation: bool = True

        self._js_pub       = None   # publishes to /joint_states
        self._gui_sub      = None   # subscribes to /joint_states_manual (GUI output)
        self._hold_timer   = None   # 10 Hz relay timer

        # Start at home so the arm is in a known position before the user
        # touches the GUI sliders for the first time.
        self._held_positions: list[float] = list(self.HOME_RADIANS)
        self._homing_active: bool = False
        self._positions_lock = threading.Lock()

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.declare_parameter('use_simulation', True)
        self._use_simulation = (
            self.get_parameter('use_simulation').get_parameter_value().bool_value
        )
        mode = 'SIMULATION' if self._use_simulation else 'HARDWARE'
        self.get_logger().info(f'welding_home_skill: configuring [{mode} mode]')
        if not self._use_simulation:
            self.get_logger().warning(
                'welding_home_skill: hardware mode selected but /move_home action '
                'server is not yet implemented in robin_moveit_control — '
                'falling back to simulation behaviour. '
                'Add a /move_home action to robin_planner.py to enable real motion.'
            )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(
            f'welding_home_skill: activating — advertising {self.ACTION_NAME}'
        )
        # Sole /joint_states publisher in simulation
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Relay: subscribe to the GUI's remapped output and pass it through
        # to /joint_states while not homing
        self._gui_sub = self.create_subscription(
            JointState,
            '/joint_states_manual',
            self._gui_joint_states_callback,
            10,
        )

        # Hold timer: publishes held_positions at 10 Hz when not homing
        self._hold_timer = self.create_timer(
            1.0 / self.HOLD_HZ, self._publish_held_positions
        )

        self._action_server = ActionServer(
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
        if self._action_server:
            self._action_server.destroy()
            self._action_server = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── GUI relay ────────────────────────────────────────────────────────────

    def _gui_joint_states_callback(self, msg: JointState) -> None:
        """Pass GUI slider values into _held_positions when not homing.

        During homing this callback is ignored — the execute callback is the
        sole publisher and _held_positions is only written at goal completion.
        """
        if self._homing_active:
            return
        with self._positions_lock:
            for i, name in enumerate(self.UR_JOINTS):
                if name in msg.name:
                    idx = list(msg.name).index(name)
                    if idx < len(msg.position):
                        self._held_positions[i] = msg.position[idx]

    def _publish_held_positions(self) -> None:
        """Timer callback: relays the current held positions to /joint_states."""
        if self._homing_active or self._js_pub is None:
            return
        with self._positions_lock:
            positions = list(self._held_positions)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = list(self.UR_JOINTS)
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
        mode = '[sim]' if self._use_simulation else '[hw→sim fallback]'
        self.get_logger().info(f'MOVE_TO_HOME {mode}: executing ...')

        result = MoveToHome.Result()

        # Bail out immediately if this goal was already preempted before we started
        if not goal_handle.is_active:
            result.success = False
            result.message = 'Goal preempted before execution started'
            return result

        # Snapshot current position as the animation start point
        with self._positions_lock:
            start_positions = list(self._held_positions)

        self._homing_active = True   # suppresses GUI relay and hold timer

        n_steps      = int(self.MOCK_DURATION_S * self.PUBLISH_HZ)
        step_delay_s = 1.0 / self.PUBLISH_HZ
        n_joints     = len(self.UR_JOINTS)
        feedback     = MoveToHome.Feedback()

        try:
            for step in range(n_steps + 1):
                # Exit: preempted by a newer goal
                if not goal_handle.is_active:
                    self.get_logger().info('MOVE_TO_HOME: preempted — exiting cleanly')
                    result.success = False
                    result.message = 'Goal preempted'
                    return result

                # Exit: operator cancel
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal cancelled by operator'
                    return result

                # Interpolate from current pose to home
                alpha = step / n_steps
                positions = [
                    start_positions[j] + alpha * (self.HOME_RADIANS[j] - start_positions[j])
                    for j in range(n_joints)
                ]

                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name     = list(self.UR_JOINTS)
                js.position = positions
                if self._js_pub is not None:
                    self._js_pub.publish(js)

                joint_idx = min(int(alpha * n_joints), n_joints - 1)
                feedback.progress_pct  = alpha * 100.0
                feedback.current_joint = self.JOINTS[joint_idx]
                goal_handle.publish_feedback(feedback)

                time.sleep(step_delay_s)

        finally:
            # Always release the relay, even on exception
            self._homing_active = False

        # Check again — could have been preempted on the very last step
        if not goal_handle.is_active:
            result.success = False
            result.message = 'Goal preempted'
            return result

        # Persist home position: relay will now hold here until GUI sliders move
        with self._positions_lock:
            self._held_positions = list(self.HOME_RADIANS)

        goal_handle.succeed()
        result.success = True
        result.message = 'Robot moved to home position successfully'
        self.get_logger().info('MOVE_TO_HOME: complete')
        return result


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
