#!/usr/bin/env python3
"""
Welding Coordinator Node

Orchestrates welding operations by coordinating:
- WAGO PLC signals (gas, robot_ready, welding_start)
- Fronius welder parameters (current, voltage, wire_speed)

Services:
- /welding/start - Set parameters and start welding sequence
- /welding/stop  - Stop welding sequence
- /welding/set_params - Set welding parameters without starting

Welding Start Sequence:
1. Set Fronius parameters (current, voltage, wire_speed)
2. Set robot_ready (must be before gas_on for proper flow)
3. Activate gas_on
4. Wait for gas purge (configurable, default 2.0s)
5. Activate welding_start (arc on)
6. Return success - robot can now move

Welding Stop Sequence:
1. Deactivate welding_start (arc off)
2. Deactivate gas_on
3. Deactivate robot_ready
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool, Trigger
from robin_interfaces.srv import StartWeld, SetFloat32


class WeldingCoordinator(Node):
    """Coordinates welding start/stop sequences via OPC UA bridge services."""

    def __init__(self):
        super().__init__('welding_coordinator')
        
        # Declare parameters
        self.declare_parameter('gas_purge_time', 2.0)  # seconds to wait after gas_on before arc
        self.gas_purge_time = self.get_parameter('gas_purge_time').value
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Service clients for WAGO PLC
        self.gas_on_client = self.create_client(
            SetBool, '/wago/gas_on', callback_group=self.callback_group)
        self.robot_ready_client = self.create_client(
            SetBool, '/wago/robot_ready', callback_group=self.callback_group)
        self.welding_start_client = self.create_client(
            SetBool, '/wago/welding_start', callback_group=self.callback_group)
        
        # Service clients for Fronius parameters
        self.set_current_client = self.create_client(
            SetFloat32, '/fronius/set_current', callback_group=self.callback_group)
        self.set_voltage_client = self.create_client(
            SetFloat32, '/fronius/set_voltage', callback_group=self.callback_group)
        self.set_wire_speed_client = self.create_client(
            SetFloat32, '/fronius/set_wire_speed', callback_group=self.callback_group)
        
        # Services provided by this node
        self.start_srv = self.create_service(
            StartWeld, '/welding/start', self.start_weld_callback,
            callback_group=self.callback_group)
        self.stop_srv = self.create_service(
            Trigger, '/welding/stop', self.stop_weld_callback,
            callback_group=self.callback_group)
        self.set_params_srv = self.create_service(
            StartWeld, '/welding/set_params', self.set_params_callback,
            callback_group=self.callback_group)
        
        # Track welding state
        self.is_welding = False
        
        self.get_logger().info('Welding Coordinator started')
        self.get_logger().info('Services: /welding/start, /welding/stop, /welding/set_params')

    def _call_service_sync(self, client, request, timeout=2.0):
        """Call a service synchronously with timeout."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None, f"Service {client.srv_name} not available"
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.done():
            return future.result(), None
        return None, f"Service {client.srv_name} timed out"

    def _set_wago_signal(self, client, value: bool) -> tuple[bool, str]:
        """Set a WAGO PLC signal."""
        request = SetBool.Request()
        request.data = value
        result, error = self._call_service_sync(client, request)
        if error:
            return False, error
        return result.success, result.message

    def _set_fronius_param(self, client, value: float) -> tuple[bool, str]:
        """Set a Fronius parameter."""
        request = SetFloat32.Request()
        request.data = value
        result, error = self._call_service_sync(client, request)
        if error:
            return False, error
        return result.success, result.message

    def set_params_callback(self, request: StartWeld.Request, 
                            response: StartWeld.Response) -> StartWeld.Response:
        """Set welding parameters without starting."""
        self.get_logger().info(
            f'Setting params: {request.current}A, {request.voltage}V, {request.wire_speed}m/min')
        
        errors = []
        
        # Set current
        ok, msg = self._set_fronius_param(self.set_current_client, request.current)
        if not ok:
            errors.append(f"current: {msg}")
        
        # Set voltage
        ok, msg = self._set_fronius_param(self.set_voltage_client, request.voltage)
        if not ok:
            errors.append(f"voltage: {msg}")
        
        # Set wire speed
        ok, msg = self._set_fronius_param(self.set_wire_speed_client, request.wire_speed)
        if not ok:
            errors.append(f"wire_speed: {msg}")
        
        if errors:
            response.success = False
            response.message = "Failed: " + ", ".join(errors)
        else:
            response.success = True
            response.message = f"Parameters set: {request.current}A, {request.voltage}V, {request.wire_speed}m/min"
        
        return response

    def start_weld_callback(self, request: StartWeld.Request, 
                            response: StartWeld.Response) -> StartWeld.Response:
        """Start welding: set parameters, then robot_ready -> gas_on -> purge -> welding_start.
        
        Sequence:
        1. Set Fronius parameters
        2. Set robot_ready (must be before gas_on)
        3. Activate gas_on
        4. Wait for gas purge (gas_purge_time seconds)
        5. Activate welding_start (arc on)
        6. Return success - robot can now start weld motion
        """
        self.get_logger().info(
            f'START WELD: {request.current}A, {request.voltage}V, {request.wire_speed}m/min')
        
        errors = []
        
        # 1. Set welding parameters
        ok, msg = self._set_fronius_param(self.set_current_client, request.current)
        if not ok:
            errors.append(f"set_current: {msg}")
        
        ok, msg = self._set_fronius_param(self.set_voltage_client, request.voltage)
        if not ok:
            errors.append(f"set_voltage: {msg}")
        
        ok, msg = self._set_fronius_param(self.set_wire_speed_client, request.wire_speed)
        if not ok:
            errors.append(f"set_wire_speed: {msg}")
        
        # 2. Set robot_ready FIRST (required before gas_on for proper gas flow)
        self.get_logger().info('Setting robot_ready...')
        ok, msg = self._set_wago_signal(self.robot_ready_client, True)
        if not ok:
            errors.append(f"robot_ready: {msg}")
        
        # 3. Activate gas_on
        self.get_logger().info('Activating gas_on...')
        ok, msg = self._set_wago_signal(self.gas_on_client, True)
        if not ok:
            errors.append(f"gas_on: {msg}")
        
        # 4. Gas purge delay - wait for gas to flow before striking arc
        if not errors:
            self.get_logger().info(f'Gas purge: waiting {self.gas_purge_time}s...')
            time.sleep(self.gas_purge_time)
        
        # 5. Activate welding_start (arc on)
        self.get_logger().info('Activating welding_start (arc on)...')
        ok, msg = self._set_wago_signal(self.welding_start_client, True)
        if not ok:
            errors.append(f"welding_start: {msg}")
        
        if errors:
            response.success = False
            response.message = "Start failed: " + ", ".join(errors)
            self.get_logger().error(response.message)
        else:
            self.is_welding = True
            response.success = True
            response.message = "Welding started - arc is on, robot can move"
            self.get_logger().info('Welding started successfully - robot can begin weld motion')
        
        return response

    def stop_weld_callback(self, request: Trigger.Request, 
                           response: Trigger.Response) -> Trigger.Response:
        """Stop welding: welding_start(false) -> gas_on(false) -> robot_ready(false).
        
        Sequence:
        1. Deactivate welding_start (arc off)
        2. Deactivate gas_on
        3. Deactivate robot_ready
        """
        self.get_logger().info('STOP WELD')
        
        errors = []
        
        # 1. Stop arc first
        self.get_logger().info('Deactivating welding_start (arc off)...')
        ok, msg = self._set_wago_signal(self.welding_start_client, False)
        if not ok:
            errors.append(f"welding_start: {msg}")
        
        # 2. Stop gas
        self.get_logger().info('Deactivating gas_on...')
        ok, msg = self._set_wago_signal(self.gas_on_client, False)
        if not ok:
            errors.append(f"gas_on: {msg}")
        
        # 3. Clear robot_ready
        self.get_logger().info('Clearing robot_ready...')
        ok, msg = self._set_wago_signal(self.robot_ready_client, False)
        if not ok:
            errors.append(f"robot_ready: {msg}")
        
        if errors:
            response.success = False
            response.message = "Stop failed: " + ", ".join(errors)
            self.get_logger().error(response.message)
        else:
            self.is_welding = False
            response.success = True
            response.message = "Welding stopped"
            self.get_logger().info('Welding stopped successfully')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WeldingCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
