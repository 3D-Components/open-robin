"""Lifecycle manager — ordered lifecycle bring-up with retry and monitoring.

Improvements over the original fire-and-forget manager:
  - Retries startup failures in node order so dependencies settle first.
  - Stays alive after initial bring-up and keeps retrying pending nodes.
  - Monitors managed nodes and re-manages them if they drop out.
  - Supports configure-only managers (``activate:=false``) correctly.
  - Cleans up transient service clients after every call.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State


class LifecycleManager(Node):
    """Configure and monitor lifecycle nodes in a dependency-aware order."""

    def __init__(self):
        super().__init__('lifecycle_manager')

        self.declare_parameter('node_names', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('configure_timeout', 10.0)
        self.declare_parameter('activate', True)
        self.declare_parameter('max_retries', 5)
        self.declare_parameter('retry_delay', 3.0)
        self.declare_parameter('monitor_period', 5.0)  # 0 disables monitoring
        self.declare_parameter('strict_order', True)

        self._node_names = list(self.get_parameter('node_names').value)
        self._timeout = self.get_parameter('configure_timeout').value
        self._activate = self.get_parameter('activate').value
        self._max_retries = int(self.get_parameter('max_retries').value)
        self._retry_delay = self.get_parameter('retry_delay').value
        self._monitor_period = self.get_parameter('monitor_period').value
        self._strict_order = bool(self.get_parameter('strict_order').value)

        # Nodes that have been successfully brought up at least once.
        self._managed_nodes: set[str] = set()
        self._startup_warned = False

        # Callback group for the health-monitor timer so its blocking
        # service calls don't starve other callbacks.
        self._monitor_cb_group = MutuallyExclusiveCallbackGroup()
        self._monitor_timer = None

        clock_mode = (
            'sim_time' if bool(self.get_parameter('use_sim_time').value)
            else 'system_time'
        )
        self.get_logger().info(
            f'Configured lifecycle manager: clock_mode={clock_mode}, '
            f'managed_nodes={self._node_names}'
        )

    def _is_ready_state(self, state: int | None) -> bool:
        """Return True when *state* satisfies this manager's goal."""
        if state is None:
            return False
        if self._activate:
            return state == State.PRIMARY_STATE_ACTIVE
        return state in (
            State.PRIMARY_STATE_INACTIVE,
            State.PRIMARY_STATE_ACTIVE,
        )

    def _unmanaged_nodes(self) -> list[str]:
        return [name for name in self._node_names if name not in self._managed_nodes]

    # ── Synchronous helpers (safe before executor.spin) ───────────────

    def _get_state_sync(self, node_name):
        client = self.create_client(GetState, f'{node_name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=self._timeout):
                self.get_logger().warn(
                    f'[{node_name}] get_state service not available')
                return None
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=self._timeout)
            return future.result().current_state.id if future.done() else None
        finally:
            self.destroy_client(client)

    def _change_state_sync(self, node_name, transition_id):
        client = self.create_client(
            ChangeState, f'{node_name}/change_state')
        try:
            if not client.wait_for_service(timeout_sec=self._timeout):
                self.get_logger().warn(
                    f'[{node_name}] change_state service not available')
                return False
            req = ChangeState.Request()
            req.transition.id = transition_id
            future = client.call_async(req)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=self._timeout)
            return future.result().success if future.done() else False
        finally:
            self.destroy_client(client)

    # ── Async helpers (safe inside callbacks while executor is spinning) ─

    def _get_state_async(self, node_name, timeout=5.0):
        client = self.create_client(GetState, f'{node_name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            future = client.call_async(GetState.Request())
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            return future.result().current_state.id if future.done() else None
        finally:
            self.destroy_client(client)

    def _change_state_async(self, node_name, transition_id, timeout=5.0):
        client = self.create_client(
            ChangeState, f'{node_name}/change_state')
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return False
            req = ChangeState.Request()
            req.transition.id = transition_id
            future = client.call_async(req)
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            return future.result().success if future.done() else False
        finally:
            self.destroy_client(client)

    # ── Bring-up logic ────────────────────────────────────────────────

    def _bring_up_sync(self, name) -> bool:
        """Configure and, when requested, activate one node."""
        state = self._get_state_sync(name)
        if state is None:
            return False

        if self._is_ready_state(state):
            return True

        if state == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info(f'[{name}] Configuring...')
            if not self._change_state_sync(
                    name, Transition.TRANSITION_CONFIGURE):
                self.get_logger().error(f'[{name}] Configure FAILED')
                return False
            self.get_logger().info(f'[{name}] Configured')
            state = self._get_state_sync(name)
            if state is None:
                return False

        if self._activate and state == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info(f'[{name}] Activating...')
            if not self._change_state_sync(
                    name, Transition.TRANSITION_ACTIVATE):
                self.get_logger().error(f'[{name}] Activate FAILED')
                return False
            self.get_logger().info(f'[{name}] Active')
            state = self._get_state_sync(name)

        if self._is_ready_state(state):
            return True

        self.get_logger().warn(
            f'[{name}] Unexpected state after bring-up attempt: {state}')
        return False

    def _bring_up_async(self, name) -> bool:
        """Async variant of :meth:`_bring_up_sync` for monitor callbacks."""
        state = self._get_state_async(name, timeout=self._timeout)
        if state is None:
            return False

        if self._is_ready_state(state):
            return True

        if state == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info(f'[{name}] Configuring...')
            if not self._change_state_async(
                    name, Transition.TRANSITION_CONFIGURE, timeout=self._timeout):
                self.get_logger().error(f'[{name}] Configure FAILED')
                return False
            self.get_logger().info(f'[{name}] Configured')
            state = self._get_state_async(name, timeout=self._timeout)
            if state is None:
                return False

        if self._activate and state == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info(f'[{name}] Activating...')
            if not self._change_state_async(
                    name, Transition.TRANSITION_ACTIVATE, timeout=self._timeout):
                self.get_logger().error(f'[{name}] Activate FAILED')
                return False
            self.get_logger().info(f'[{name}] Active')
            state = self._get_state_async(name, timeout=self._timeout)

        if self._is_ready_state(state):
            return True

        self.get_logger().warn(
            f'[{name}] Unexpected state after async bring-up attempt: {state}')
        return False

    def manage_all(self) -> bool:
        """Bring up managed nodes, retrying in dependency order."""
        if not self._node_names:
            self.get_logger().error('No node_names provided')
            return False

        self.get_logger().info(
            f'Managing {len(self._node_names)} nodes: {self._node_names}')

        for attempt in range(1, self._max_retries + 1):
            first_failed = None
            for name in self._unmanaged_nodes():
                if self._bring_up_sync(name):
                    self._managed_nodes.add(name)
                    continue

                first_failed = name
                if self._strict_order:
                    break

            if len(self._managed_nodes) == len(self._node_names):
                self.get_logger().info(
                    f'All {len(self._node_names)} nodes brought up successfully')
                return True

            pending = self._unmanaged_nodes()
            if first_failed is None and pending:
                first_failed = pending[0]

            self.get_logger().warn(
                f'Attempt {attempt}/{self._max_retries} incomplete; '
                f'pending={pending}')

            if attempt < self._max_retries:
                delay = self._retry_delay * attempt   # linear backoff
                self.get_logger().info(f'Retrying in {delay:.0f}s...')
                time.sleep(delay)

        pending = self._unmanaged_nodes()
        message = (
            f'Startup still pending after {self._max_retries} attempts: '
            f'{pending}'
        )
        if bool(self.get_parameter('use_sim_time').value):
            message += '; verify `/clock`, Gazebo startup, and lifecycle dependencies'
        self.get_logger().warn(message)
        return False

    # ── Health monitoring ─────────────────────────────────────────────

    def _monitor_callback(self):
        """Re-check managed nodes and continue bringing up pending ones."""
        # Validate already-managed nodes in launch order. If one drops out,
        # remove it from the managed set and let the ordered bring-up pass
        # recover it together with any downstream dependants.
        for name in self._node_names:
            if name not in self._managed_nodes:
                continue
            state = self._get_state_async(name, timeout=3.0)
            if state is None:
                self.get_logger().warn(
                    f'[{name}] Unreachable — will retry bring-up')
                self._managed_nodes.discard(name)
                if self._strict_order:
                    break
                continue

            if self._is_ready_state(state):
                continue

            self.get_logger().warn(
                f'[{name}] Left desired lifecycle state (state={state}) — '
                'will retry bring-up')
            self._managed_nodes.discard(name)
            if self._strict_order:
                break

        newly_managed = []
        for name in self._unmanaged_nodes():
            if self._bring_up_async(name):
                self._managed_nodes.add(name)
                newly_managed.append(name)
                continue
            if self._strict_order:
                break

        if newly_managed:
            self.get_logger().info(f'Background bring-up succeeded for: {newly_managed}')

        if len(self._managed_nodes) == len(self._node_names):
            if self._startup_warned:
                self.get_logger().info('All managed nodes are now ready')
                self._startup_warned = False
        elif not self._startup_warned:
            self.get_logger().warn(
                f'Waiting on lifecycle nodes: {self._unmanaged_nodes()}')
            self._startup_warned = True

    def start_monitoring(self):
        """Begin periodic health checks on managed nodes."""
        if self._monitor_period <= 0:
            return
        self._monitor_timer = self.create_timer(
            self._monitor_period, self._monitor_callback,
            callback_group=self._monitor_cb_group)
        self.get_logger().info(
            f'Health monitor started (period={self._monitor_period:.0f}s)')


def main(args=None):
    rclpy.init(args=args)
    manager = LifecycleManager()

    # Phase 1: synchronous bring-up with retry (no executor spinning yet)
    startup_complete = manager.manage_all()
    if not startup_complete:
        manager.get_logger().warn(
            'Initial bring-up incomplete; continuing retries in background')

    # Phase 2: stay alive with health monitoring
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(manager)
    manager.start_monitoring()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()
