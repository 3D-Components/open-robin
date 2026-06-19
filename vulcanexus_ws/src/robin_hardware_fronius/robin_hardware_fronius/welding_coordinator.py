#!/usr/bin/env python3
"""
Welding Coordinator Node

Orchestrates welding operations by coordinating:
- WAGO PLC signals via GVL_Fronius_IN (control) and GVL_Fronius_OUT (feedback)
- Fronius welder parameters (current, voltage, wire_speed)

Services:
- /welding/start - Set parameters and start welding sequence
- /welding/stop  - Stop welding sequence
- /welding/set_params - Set welding parameters without starting

Welding Start Sequence:
1. Check power source ready (from WAGO OUT)
2. Set WorkingMode if needed
3. Set Fronius parameters (primary + optional overrides)
4. Set robot_ready (must be before gas_on for proper flow)
5. Activate gas_on
6. Wait for gas purge (configurable, default 2.0s)
7. Activate welding_start (arc on)
8. Return success - robot can now move

Welding Stop Sequence:
1. Deactivate welding_start (arc off)
2. Deactivate gas_on
3. Deactivate robot_ready

WAGO WorkingMode enum values:
  0 = Internal parameter selection (synergetic)
  1 = Special 2-step mode
  2 = Job Mode
  8 = 2-step characteristic
  9 = MIG/MAG Manual
  24 = R/L Measurement
  25 = R/L Alignment
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Bool
from robin_interfaces.srv import StartWeld, SetFloat32, SetInt32, FindSurface, CalibrateStickout


class WeldingCoordinator(Node):
    """Coordinates welding start/stop sequences via OPC UA bridge services."""

    # WorkingMode constants
    WORKING_MODE_INTERNAL = 0   # Synergetic mode
    WORKING_MODE_MANUAL = 9     # MIG/MAG Manual (all params external)

    # Primary parameter constants (match StartWeld.srv)
    PRIMARY_CURRENT = 0
    PRIMARY_VOLTAGE = 1
    PRIMARY_WIRE_FEED_SPEED = 2

    def __init__(self):
        super().__init__('welding_coordinator')
        
        # Declare parameters
        self.declare_parameter('gas_purge_time', 2.0)  # seconds to wait after gas_on before arc
        self.declare_parameter('wire_retract_speed_mm_s', 10.0)
        self.gas_purge_time = self.get_parameter('gas_purge_time').value
        self.wire_retract_speed_mm_s = self.get_parameter('wire_retract_speed_mm_s').value
        
        self.callback_group = ReentrantCallbackGroup()
        
        # ----- Service clients for WAGO PLC (GVL_Fronius_IN) -----
        self.gas_on_client = self.create_client(
            SetBool, '/wago/in/gas_on', callback_group=self.callback_group)
        self.robot_ready_client = self.create_client(
            SetBool, '/wago/in/robot_ready', callback_group=self.callback_group)
        self.welding_start_client = self.create_client(
            SetBool, '/wago/in/welding_start', callback_group=self.callback_group)
        self.touch_sensing_client = self.create_client(
            SetBool, '/wago/in/touch_sensing', callback_group=self.callback_group)
        self.error_quit_client = self.create_client(
            SetBool, '/wago/in/error_quit', callback_group=self.callback_group)
        self.wire_forward_client = self.create_client(
            SetBool, '/wago/in/wire_forward', callback_group=self.callback_group)
        self.wire_backward_client = self.create_client(
            SetBool, '/wago/in/wire_backward', callback_group=self.callback_group)
        self.wire_move_length_client = self.create_client(
            SetFloat32, '/wago/in/wire_move_length', callback_group=self.callback_group)
        self.working_mode_client = self.create_client(
            SetInt32, '/wago/in/working_mode', callback_group=self.callback_group)
        self.welding_speed_client = self.create_client(
            SetFloat32, '/wago/in/welding_speed', callback_group=self.callback_group)
        
        # ----- Service clients for Fronius parameters (direct OPC UA) -----
        self.set_current_client = self.create_client(
            SetFloat32, '/fronius/set_current', callback_group=self.callback_group)
        self.set_voltage_client = self.create_client(
            SetFloat32, '/fronius/set_voltage', callback_group=self.callback_group)
        self.set_wire_speed_client = self.create_client(
            SetFloat32, '/fronius/set_wire_speed', callback_group=self.callback_group)
        
        # ----- Subscriptions to WAGO OUT feedback signals -----
        self._power_source_ready = False
        self._process_active = False
        self._touch_signal = False
        self._warning = False
        
        self.create_subscription(
            Bool, '/wago/out/power_source_ready',
            self._power_source_ready_cb, 10, callback_group=self.callback_group)
        self.create_subscription(
            Bool, '/wago/out/process_active',
            self._process_active_cb, 10, callback_group=self.callback_group)
        self.create_subscription(
            Bool, '/wago/out/touch_signal',
            self._touch_signal_cb, 10, callback_group=self.callback_group)
        self.create_subscription(
            Bool, '/wago/out/warning',
            self._warning_cb, 10, callback_group=self.callback_group)
        
        # ----- Services provided by this node -----
        self.start_srv = self.create_service(
            StartWeld, '/welding/start', self.start_weld_callback,
            callback_group=self.callback_group)
        self.stop_srv = self.create_service(
            Trigger, '/welding/stop', self.stop_weld_callback,
            callback_group=self.callback_group)
        self.set_params_srv = self.create_service(
            StartWeld, '/welding/set_params', self.set_params_callback,
            callback_group=self.callback_group)
        self.touch_probe_srv = self.create_service(
            Trigger, '/welding/touch_probe', self.touch_probe_callback,
            callback_group=self.callback_group)
        self.wire_feed_until_touch_srv = self.create_service(
            Trigger, '/welding/wire_feed_until_touch',
            self.wire_feed_until_touch_callback,
            callback_group=self.callback_group)
        self.wire_retract_srv = self.create_service(
            SetFloat32, '/welding/wire_retract',
            self.wire_retract_callback,
            callback_group=self.callback_group)
        
        # Track welding state
        self.is_welding = False
        
        self.get_logger().info('Welding Coordinator started')
        self.get_logger().info(
            'Services: /welding/start, /welding/stop, /welding/set_params, '
            '/welding/touch_probe, /welding/wire_feed_until_touch, /welding/wire_retract')

        # Set robot_ready on startup so the operator panel / PLC signals work
        self._startup_timer = self.create_timer(
            2.0, self._set_robot_ready_on_startup)

    def _set_robot_ready_on_startup(self):
        """Set robot_ready=True once on startup after services are available."""
        self._startup_timer.cancel()
        self.get_logger().info('Setting robot_ready=True on startup...')
        ok, msg = self._set_wago_signal(self.robot_ready_client, True)
        if ok:
            self.get_logger().info('robot_ready set to True')
        else:
            self.get_logger().warn(f'Failed to set robot_ready on startup: {msg}')

    # ----- WAGO OUT feedback callbacks -----
    def _power_source_ready_cb(self, msg: Bool):
        self._power_source_ready = msg.data

    def _process_active_cb(self, msg: Bool):
        self._process_active = msg.data

    def _touch_signal_cb(self, msg: Bool):
        self._touch_signal = msg.data

    def _warning_cb(self, msg: Bool):
        if msg.data and not self._warning:
            self.get_logger().warn('Fronius power source warning active!')
        self._warning = msg.data

    # ----- Service call helpers -----
    def _call_service_sync(self, client, request, timeout=2.0):
        """Call a service synchronously with timeout.

        Uses a poll-wait loop instead of ``rclpy.spin_until_future_complete``
        to avoid re-entering the executor spin, which can deadlock.
        The ``MultiThreadedExecutor`` running in ``main()`` will deliver
        the response on a separate thread while this one sleeps.
        """
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=timeout):
                return None, f"Service {client.srv_name} not available"

        future = client.call_async(request)

        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                future.cancel()
                return None, f"Service {client.srv_name} timed out"
            time.sleep(0.01)  # 10 ms poll — yields thread to executor

        return future.result(), None

    def _set_wago_signal(self, client, value: bool) -> tuple[bool, str]:
        """Set a WAGO PLC signal (bool)."""
        request = SetBool.Request()
        request.data = value
        result, error = self._call_service_sync(client, request)
        if error:
            return False, error
        return result.success, result.message

    def _set_fronius_param(self, client, value: float) -> tuple[bool, str]:
        """Set a Fronius parameter (float)."""
        request = SetFloat32.Request()
        request.data = value
        result, error = self._call_service_sync(client, request)
        if error:
            return False, error
        return result.success, result.message

    def _set_working_mode(self, mode: int) -> tuple[bool, str]:
        """Set the WAGO WorkingMode (int32 enum)."""
        request = SetInt32.Request()
        request.data = mode
        result, error = self._call_service_sync(self.working_mode_client, request)
        if error:
            return False, error
        return result.success, result.message

    def _set_welding_params_synergy(self, request: StartWeld.Request) -> list[str]:
        """Set welding parameters respecting synergy mode.
        
        If only the primary parameter is set (others are 0.0), use synergetic mode.
        If overrides are provided (non-zero), set those too.
        """
        errors = []
        primary = request.primary_parameter

        # Determine if we need manual mode (overrides provided)
        has_overrides = False
        if primary == self.PRIMARY_CURRENT:
            has_overrides = request.voltage > 0.0 or request.wire_speed > 0.0
        elif primary == self.PRIMARY_VOLTAGE:
            has_overrides = request.current > 0.0 or request.wire_speed > 0.0
        elif primary == self.PRIMARY_WIRE_FEED_SPEED:
            has_overrides = request.current > 0.0 or request.voltage > 0.0

        # Set working mode
        if has_overrides:
            self.get_logger().info('Setting manual working mode (overrides provided)')
            ok, msg = self._set_working_mode(self.WORKING_MODE_MANUAL)
            if not ok:
                errors.append(f"working_mode: {msg}")
        else:
            self.get_logger().info('Setting synergetic working mode')
            ok, msg = self._set_working_mode(self.WORKING_MODE_INTERNAL)
            if not ok:
                errors.append(f"working_mode: {msg}")

        # Set primary parameter
        if primary == self.PRIMARY_CURRENT:
            self.get_logger().info(f'Setting primary: current={request.current}A')
            ok, msg = self._set_fronius_param(self.set_current_client, request.current)
            if not ok:
                errors.append(f"current: {msg}")
        elif primary == self.PRIMARY_VOLTAGE:
            self.get_logger().info(f'Setting primary: voltage={request.voltage}V')
            ok, msg = self._set_fronius_param(self.set_voltage_client, request.voltage)
            if not ok:
                errors.append(f"voltage: {msg}")
        elif primary == self.PRIMARY_WIRE_FEED_SPEED:
            self.get_logger().info(f'Setting primary: wire_speed={request.wire_speed}m/min')
            ok, msg = self._set_fronius_param(self.set_wire_speed_client, request.wire_speed)
            if not ok:
                errors.append(f"wire_speed: {msg}")

        # Set overrides if non-zero
        if primary != self.PRIMARY_CURRENT and request.current > 0.0:
            self.get_logger().info(f'Setting override: current={request.current}A')
            ok, msg = self._set_fronius_param(self.set_current_client, request.current)
            if not ok:
                errors.append(f"current_override: {msg}")

        if primary != self.PRIMARY_VOLTAGE and request.voltage > 0.0:
            self.get_logger().info(f'Setting override: voltage={request.voltage}V')
            ok, msg = self._set_fronius_param(self.set_voltage_client, request.voltage)
            if not ok:
                errors.append(f"voltage_override: {msg}")

        if primary != self.PRIMARY_WIRE_FEED_SPEED and request.wire_speed > 0.0:
            self.get_logger().info(f'Setting override: wire_speed={request.wire_speed}m/min')
            ok, msg = self._set_fronius_param(self.set_wire_speed_client, request.wire_speed)
            if not ok:
                errors.append(f"wire_speed_override: {msg}")

        return errors

    # ----- Service callbacks -----
    def set_params_callback(self, request: StartWeld.Request, 
                            response: StartWeld.Response) -> StartWeld.Response:
        """Set welding parameters without starting."""
        self.get_logger().info(
            f'Setting params: primary={request.primary_parameter}, '
            f'current={request.current}A, voltage={request.voltage}V, '
            f'wire_speed={request.wire_speed}m/min')
        
        errors = self._set_welding_params_synergy(request)
        
        if errors:
            response.success = False
            response.message = "Failed: " + ", ".join(errors)
        else:
            response.success = True
            response.message = (f"Parameters set: primary={request.primary_parameter}, "
                              f"current={request.current}A, voltage={request.voltage}V, "
                              f"wire_speed={request.wire_speed}m/min")
        
        return response

    def start_weld_callback(self, request: StartWeld.Request, 
                            response: StartWeld.Response) -> StartWeld.Response:
        """Start welding: set parameters, then robot_ready -> gas_on -> purge -> welding_start.
        
        Sequence:
        1. Check power source readiness
        2. Set welding parameters (with synergy support)
        3. Set robot_ready (must be before gas_on)
        4. Activate gas_on
        5. Wait for gas purge (gas_purge_time seconds)
        6. Activate welding_start (arc on)
        7. Return success - robot can now start weld motion
        """
        self.get_logger().info(
            f'START WELD: primary={request.primary_parameter}, '
            f'current={request.current}A, voltage={request.voltage}V, '
            f'wire_speed={request.wire_speed}m/min')
        
        errors = []

        # 0. Check power source (warn but don't block - signal may not be available yet)
        if not self._power_source_ready:
            self.get_logger().warn('Power source not confirmed ready (proceeding anyway)')
        
        # 1. Set welding parameters with synergy support
        param_errors = self._set_welding_params_synergy(request)
        errors.extend(param_errors)
        
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

    def touch_probe_callback(self, request: Trigger.Request,
                             response: Trigger.Response) -> Trigger.Response:
        """Activate/deactivate touch sensing mode on the WAGO PLC.
        
        This enables the Fronius touch sensing circuit. When active, the wire
        is energised with a low detection voltage. Contact with the workpiece
        triggers the touch_signal feedback from the WAGO OUT side.
        
        The planner calls this to enable touch sensing before probing,
        then monitors /wago/out/touch_signal and disables it afterwards.
        
        Returns:
            Trigger.Response with success/message
        """
        self.get_logger().info('Enabling touch sensing mode...')
        
        # Enable touch sensing via WAGO
        ok, msg = self._set_wago_signal(self.touch_sensing_client, True)
        
        if not ok:
            response.success = False
            response.message = f"Failed to enable touch sensing: {msg}"
            self.get_logger().error(response.message)
        else:
            response.success = True
            response.message = "Touch sensing enabled - wire energised for contact detection"
            self.get_logger().info(response.message)
        
        return response

    def touch_probe_disable(self):
        """Disable touch sensing (internal helper, also exposed if needed)."""
        ok, msg = self._set_wago_signal(self.touch_sensing_client, False)
        if not ok:
            self.get_logger().error(f"Failed to disable touch sensing: {msg}")
        else:
            self.get_logger().info("Touch sensing disabled")
        self._touch_signal = False
        return ok

    # ----- Wire feed / retract helpers -----
    def _set_wire_move_length(self, length_mm: float) -> tuple[bool, str]:
        """Set the WAGO WireMoveLength parameter (float, in mm).
        
        When set to 0, the wire feeds/retracts continuously while the
        forward/backward flag is active. When > 0, the wire moves a
        fixed distance per forward/backward activation.
        """
        request = SetFloat32.Request()
        request.data = length_mm
        result, error = self._call_service_sync(self.wire_move_length_client, request)
        if error:
            return False, error
        return result.success, result.message

    def wire_feed_until_touch_callback(self, request: Trigger.Request,
                                       response: Trigger.Response) -> Trigger.Response:
        """Feed wire continuously until touch contact is detected.
        
        Sequence:
        1. Set WireMoveLength = 0 (continuous feed mode)
        2. Activate WireForward
        3. Wait for TouchSignal = True (Fronius auto-stops wire on contact)
        4. Deactivate WireForward
        
        Touch sensing MUST be enabled before calling this service.
        The Fronius machine automatically stops wire feed when
        the touch-sensing circuit detects workpiece contact.
        
        Returns:
            Trigger.Response with success/message
        """
        self.get_logger().info('Wire feed until touch: starting continuous feed...')

        # Guard: do not start if touch is already active (stale/ongoing contact)
        if self._touch_signal:
            response.success = False
            response.message = (
                'Touch signal already active before wire feed. '
                'Reposition tool / clear contact and retry calibration.')
            self.get_logger().warn(response.message)
            return response

        # 1. Set continuous feed mode (length = 0)
        ok, msg = self._set_wire_move_length(0.0)
        if not ok:
            response.success = False
            response.message = f"Failed to set wire_move_length=0: {msg}"
            self.get_logger().error(response.message)
            return response

        # 2. Start feeding
        ok, msg = self._set_wago_signal(self.wire_forward_client, True)
        if not ok:
            response.success = False
            response.message = f"Failed to activate wire_forward: {msg}"
            self.get_logger().error(response.message)
            return response

        # 3. Wait for touch signal (timeout after 30s for safety)
        timeout = 30.0
        poll_interval = 0.05  # 50ms
        elapsed = 0.0

        stable_touch_count = 0
        required_stable_counts = 2  # 2 x 50ms = 100ms

        while elapsed < timeout:
            time.sleep(poll_interval)
            elapsed += poll_interval

            if self._touch_signal:
                stable_touch_count += 1
                if stable_touch_count >= required_stable_counts:
                    self.get_logger().info(
                        f'Touch detected after {elapsed:.2f}s of wire feed')
                    break
            else:
                stable_touch_count = 0
        else:
            # Timeout — stop wire and report failure
            self._set_wago_signal(self.wire_forward_client, False)
            response.success = False
            response.message = (
                f"Wire feed timeout ({timeout}s) — no touch signal detected. "
                "Check touch sensing is enabled and wire can reach workpiece.")
            self.get_logger().error(response.message)
            return response

        # 4. Stop wire forward (may already be stopped by Fronius, but ensure)
        self._set_wago_signal(self.wire_forward_client, False)

        response.success = True
        response.message = f"Wire touched workpiece after {elapsed:.2f}s"
        self.get_logger().info(response.message)
        return response

    def wire_retract_callback(self, request: SetFloat32.Request,
                              response: SetFloat32.Response) -> SetFloat32.Response:
        """Retract wire by a specified distance (mm) or continuously.
        
        If request.data > 0: retract by that distance in mm
        If request.data == 0: retract continuously for a brief period (1s)
        
        Args:
            request.data: Retraction distance in mm (0 = continuous for 1s)
        """
        retract_mm = request.data
        self.get_logger().info(f'Wire retract: {retract_mm:.1f}mm')

        if retract_mm > 0:
            # Fixed-distance retract
            ok, msg = self._set_wire_move_length(retract_mm)
            if not ok:
                response.success = False
                response.message = f"Failed to set wire_move_length: {msg}"
                return response

            # Activate backward — the PLC will retract the set distance
            ok, msg = self._set_wago_signal(self.wire_backward_client, True)
            if not ok:
                response.success = False
                response.message = f"Failed to activate wire_backward: {msg}"
                return response

            # Wait for the retraction to complete
            speed_mm_s = max(1.0, float(self.wire_retract_speed_mm_s))
            wait_time = max(0.5, (retract_mm / speed_mm_s) + 0.3)
            self.get_logger().info(
                f'Wire retract wait={wait_time:.2f}s for {retract_mm:.1f}mm '
                f'(speed_est={speed_mm_s:.1f}mm/s)')
            time.sleep(wait_time)

            # Deactivate backward
            self._set_wago_signal(self.wire_backward_client, False)
            # Small pause before next operation
            time.sleep(0.2)

        else:
            # Continuous retract for a brief period
            ok, msg = self._set_wire_move_length(0.0)
            if not ok:
                response.success = False
                response.message = f"Failed to set wire_move_length=0: {msg}"
                return response

            ok, msg = self._set_wago_signal(self.wire_backward_client, True)
            if not ok:
                response.success = False
                response.message = f"Failed to activate wire_backward: {msg}"
                return response

            time.sleep(1.0)
            self._set_wago_signal(self.wire_backward_client, False)
            time.sleep(0.2)

        response.success = True
        response.message = f"Wire retracted {retract_mm:.1f}mm"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WeldingCoordinator()

    # MultiThreadedExecutor is required so that service callbacks can
    # call other services (via _call_service_sync poll-wait) and use
    # time.sleep() without deadlocking the executor.  Combined with
    # ReentrantCallbackGroup this allows concurrent callback execution.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
