#!/usr/bin/env python3
"""
Welding Coordinator Node

Orchestrates welding operations by coordinating:
- WAGO PLC signals via GVL_Fronius_IN (control) and GVL_Fronius_OUT (feedback)
- Fronius welder parameters (wire_speed, arc_length_correction)

Services:
- /welding/start - Set parameters and start welding sequence
- /welding/stop  - Stop welding sequence
- /welding/set_params - Set welding parameters without starting

Welding Start Sequence:
1. Check power source ready (from WAGO OUT)
2. Set WorkingMode to synergetic
3. Set Fronius wire feed speed
4. Set arc length correction
5. Activate welding_start (arc on)
6. Wait for robot_motion_release = True (machine confirms arc stable, safe to move)
7. Return success - robot can now move

The Fronius power source automatically handles gas pre-flow (GPr), ignition,
starting current ramp, and signals robot_motion_release when the arc is stable
and the starting current phase has elapsed.

Welding Stop Sequence:
1. Deactivate welding_start (arc off)
2. Wait for robot_motion_release = False (machine finished post-flow/GPo)
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
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Bool
from robin_interfaces.srv import StartWeld, SetFloat32, SetInt32


class WeldingCoordinator(Node):
    """Coordinates welding start/stop sequences via OPC UA bridge services."""

    # WorkingMode constants
    WORKING_MODE_INTERNAL = 0   # Synergetic mode

    def __init__(self):
        super().__init__('welding_coordinator')

        self.declare_parameter('wire_retract_speed_mm_s', 10.0)
        self.declare_parameter('robot_motion_release_timeout', 10.0)
        self.declare_parameter('arc_length_correction_min_mm', -10.0)
        self.declare_parameter('arc_length_correction_max_mm', 10.0)

        self._abort_wire_ops = threading.Event()
        self.is_welding = False
        self._subscriptions = []
        self._wago_signals = {}

        # -- Configuration (formerly on_configure) -------------------------

        self.wire_retract_speed_mm_s = self.get_parameter('wire_retract_speed_mm_s').value
        self.robot_motion_release_timeout = self.get_parameter('robot_motion_release_timeout').value
        self.arc_length_correction_min_mm = float(
            self.get_parameter('arc_length_correction_min_mm').value)
        self.arc_length_correction_max_mm = float(
            self.get_parameter('arc_length_correction_max_mm').value)

        self.callback_group = ReentrantCallbackGroup()

        # WAGO service clients
        self.robot_ready_client = self.create_client(
            SetBool, 'wago/in/robot_ready', callback_group=self.callback_group)
        self.welding_start_client = self.create_client(
            SetBool, 'wago/in/welding_start', callback_group=self.callback_group)
        self.touch_sensing_client = self.create_client(
            SetBool, 'wago/in/touch_sensing', callback_group=self.callback_group)
        self.error_quit_client = self.create_client(
            SetBool, 'wago/in/error_quit', callback_group=self.callback_group)
        self.wire_forward_client = self.create_client(
            SetBool, 'wago/in/wire_forward', callback_group=self.callback_group)
        self.wire_backward_client = self.create_client(
            SetBool, 'wago/in/wire_backward', callback_group=self.callback_group)
        self.wire_move_length_client = self.create_client(
            SetFloat32, 'wago/in/wire_move_length', callback_group=self.callback_group)
        self.working_mode_client = self.create_client(
            SetInt32, 'wago/in/working_mode', callback_group=self.callback_group)
        self.welding_speed_client = self.create_client(
            SetFloat32, 'wago/in/welding_speed', callback_group=self.callback_group)
        self.arc_length_correction_client = self.create_client(
            SetFloat32, 'fronius/set_arc_length_correction', callback_group=self.callback_group)

        # Fronius parameter clients (direct OPC UA)
        self.set_current_client = self.create_client(
            SetFloat32, 'fronius/set_current', callback_group=self.callback_group)
        self.set_voltage_client = self.create_client(
            SetFloat32, 'fronius/set_voltage', callback_group=self.callback_group)
        self.set_wire_speed_client = self.create_client(
            SetFloat32, 'fronius/set_wire_speed', callback_group=self.callback_group)

        # WAGO OUT feedback subscriptions
        self._wago_signals = {
            'power_source_ready': False,
            'process_active': False,
            'touch_signal': False,
            'warning': False,
            'robot_motion_release': False,
            'current_flow': False,
            'main_current_signal': False,
        }
        self._subscriptions = []
        for name in self._wago_signals:
            sub = self.create_subscription(
                Bool, f'wago/out/{name}',
                lambda msg, n=name: self._wago_signal_cb(msg, n),
                10, callback_group=self.callback_group)
            self._subscriptions.append(sub)

        # Service servers
        self.start_srv = self.create_service(
            StartWeld, 'welding/start', self.start_weld_callback,
            callback_group=self.callback_group)
        self.stop_srv = self.create_service(
            Trigger, 'welding/stop', self.stop_weld_callback,
            callback_group=self.callback_group)
        self.set_params_srv = self.create_service(
            StartWeld, 'welding/set_params', self.set_params_callback,
            callback_group=self.callback_group)
        self.touch_probe_srv = self.create_service(
            Trigger, 'welding/touch_probe', self.touch_probe_callback,
            callback_group=self.callback_group)
        self.wire_feed_until_touch_srv = self.create_service(
            Trigger, 'welding/wire_feed_until_touch',
            self.wire_feed_until_touch_callback,
            callback_group=self.callback_group)
        self.wire_retract_srv = self.create_service(
            SetFloat32, 'welding/wire_retract',
            self.wire_retract_callback,
            callback_group=self.callback_group)

        self.get_logger().info('Configured: 13 service clients, 7 subscriptions, 6 service servers')

        # -- Activation (formerly on_activate) -----------------------------

        self.is_welding = False
        self._abort_wire_ops.clear()

        # Set robot_ready so operator panel / PLC signals work
        try:
            ok, msg = self._set_wago_signal(self.robot_ready_client, True)
            if ok:
                self.get_logger().info('robot_ready set to True')
            else:
                self.get_logger().warn(
                    f'Could not set robot_ready during init: {msg}')
        except Exception as e:
            self.get_logger().warn(
                f'robot_ready service not available yet: {e}')

        self.get_logger().info('Welding services ready')

    def destroy_node(self):
        """Safety shutdown: abort wire ops, stop welding, clear signals."""
        self._abort_wire_ops.set()

        if hasattr(self, 'welding_start_client'):
            # Emergency stop if welding
            if self.is_welding:
                self.get_logger().warn(
                    'Shutting down while welding — sending emergency stop')
                self._set_wago_signal(self.welding_start_client, False)
                self.is_welding = False

            # Clear robot_ready
            self._set_wago_signal(self.robot_ready_client, False)

        self.get_logger().info('Shutdown complete')
        super().destroy_node()

    # ----- WAGO OUT feedback callback -----
    def _wago_signal_cb(self, msg: Bool, name: str):
        if name == 'warning' and msg.data and not self._wago_signals['warning']:
            self.get_logger().warn('Fronius power source warning active!')
        self._wago_signals[name] = msg.data

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

    def _set_fronius_param(self, client, value: float,
                           retries: int = 2, delay: float = 0.15,
                           ) -> tuple[bool, str]:
        """Set a Fronius parameter (float), retrying on transient failures.

        After a working-mode change the Fronius OPC UA node may briefly
        return BadNotWritable until the mode transition completes.  A short
        retry loop absorbs this race without blocking on success.
        """
        request = SetFloat32.Request()
        request.data = value
        last_msg = ""
        for attempt in range(1 + retries):
            result, error = self._call_service_sync(client, request)
            if error:
                last_msg = error
            elif result.success:
                return True, result.message
            else:
                last_msg = result.message
            if attempt < retries:
                self.get_logger().warn(
                    f'Fronius write attempt {attempt + 1} failed ({last_msg}), '
                    f'retrying in {delay}s...')
                time.sleep(delay)
        return False, last_msg

    def _set_working_mode(self, mode: int) -> tuple[bool, str]:
        """Set the WAGO WorkingMode (int32 enum)."""
        request = SetInt32.Request()
        request.data = mode
        result, error = self._call_service_sync(self.working_mode_client, request)
        if error:
            return False, error
        return result.success, result.message

    def _set_wago_float(self, client, value: float) -> tuple[bool, str]:
        """Set a WAGO PLC float value."""
        request = SetFloat32.Request()
        request.data = float(value)
        result, error = self._call_service_sync(client, request)
        if error:
            return False, error
        return result.success, result.message

    def _clamp_arc_length_correction(self, value_mm: float) -> float:
        return max(self.arc_length_correction_min_mm,
                   min(self.arc_length_correction_max_mm, float(value_mm)))

    def _set_welding_params(self, request: StartWeld.Request) -> list[str]:
        """Set welding parameters
        """
        errors = []

        # Always synergetic
        self.get_logger().info('Setting synergetic working mode')
        ok, msg = self._set_working_mode(self.WORKING_MODE_INTERNAL)
        if not ok:
            errors.append(f"working_mode: {msg}")

        # Set wire feed speed
        ok, msg = self._set_fronius_param(self.set_wire_speed_client, request.wire_speed)
        if not ok:
            errors.append(f"wire_speed: {msg}")

        # Arc length correction
        requested_arc_corr = float(request.arc_length_correction_mm)
        clamped_arc_corr = self._clamp_arc_length_correction(requested_arc_corr)
        if clamped_arc_corr != requested_arc_corr:
            self.get_logger().warn(
                f"arc_length_correction clamped: {requested_arc_corr:.1f}% -> "
                f"{clamped_arc_corr:.1f}%")
        self.get_logger().info(
            f"Setting arc length correction: {clamped_arc_corr:.1f}%")
        ok, msg = self._set_wago_float(
            self.arc_length_correction_client,
            clamped_arc_corr,
        )
        if not ok:
            errors.append(f"arc_length_correction_mm: {msg}")

        # Set travel speed on the PLC (used by sim bridge for Gazebo bead model)
        if request.weld_speed > 0.0:
            ok, msg = self._set_wago_float(
                self.welding_speed_client, request.weld_speed)
            if not ok:
                errors.append(f"welding_speed: {msg}")

        return errors

    # ----- Service callbacks -----
    def set_params_callback(self, request: StartWeld.Request,
                            response: StartWeld.Response) -> StartWeld.Response:
        """Set welding parameters without starting."""
        self.get_logger().info(
            f'Setting params: WFS={request.wire_speed:.1f}m/min, '
            f'ArcCorr={request.arc_length_correction_mm:.1f}%')

        errors = self._set_welding_params(request)

        if errors:
            response.success = False
            response.message = "Failed: " + ", ".join(errors)
        else:
            response.success = True
            response.message = (f"Parameters set: WFS={request.wire_speed:.1f}m/min, "
                              f"ArcCorr={request.arc_length_correction_mm:.1f}%")

        return response

    def _wait_for_signal(self, signal_name: str, expected: bool, timeout: float) -> bool:
        """Poll-wait for a feedback signal to reach the expected value."""
        poll_interval = 0.02  # 20 ms
        elapsed = 0.0
        while elapsed < timeout:
            if self._wago_signals[signal_name] == expected:
                return True
            time.sleep(poll_interval)
            elapsed += poll_interval
        return self._wago_signals[signal_name] == expected

    def start_weld_callback(self, request: StartWeld.Request,
                            response: StartWeld.Response) -> StartWeld.Response:
        """Start welding: set parameters, then robot_ready -> welding_start -> wait for robot_motion_release.

        The Fronius power source automatically handles gas pre-flow, ignition,
        and starting current. It signals robot_motion_release when the arc is
        stable and it is safe to move the robot.

        Sequence:
        1. Check power source readiness
        2. Set welding parameters (with synergy support)
        3. Set robot_ready
        4. Activate welding_start (arc on)
        5. Wait for robot_motion_release = True (machine confirms safe to move)
        6. Return success - robot can now start weld motion
        """
        self.get_logger().info(
            f'START WELD: WFS={request.wire_speed:.1f}m/min, '
            f'ArcCorr={request.arc_length_correction_mm:.1f}%')

        # Clear any previous abort flag
        self._abort_wire_ops.clear()

        # 0. Check power source (warn but don't block - signal may not be available yet)
        if not self._wago_signals['power_source_ready']:
            self.get_logger().warn('Power source not confirmed ready (proceeding anyway)')

        # 1. Set welding parameters with synergy support
        param_errors = self._set_welding_params(request)
        if param_errors:
            response.success = False
            response.message = "Start failed (params): " + ", ".join(param_errors)
            self.get_logger().error(response.message)
            return response

        # 2. Set robot_ready
        self.get_logger().info('Setting robot_ready...')
        ok, msg = self._set_wago_signal(self.robot_ready_client, True)
        if not ok:
            response.success = False
            response.message = f"Start failed: robot_ready: {msg}"
            self.get_logger().error(response.message)
            return response

        # 3. Activate welding_start (machine handles GPr + ignition automatically)
        self.get_logger().info('Activating welding_start (arc on)...')
        ok, msg = self._set_wago_signal(self.welding_start_client, True)
        if not ok:
            self._set_wago_signal(self.robot_ready_client, False)
            response.success = False
            response.message = f"Start failed: welding_start: {msg}"
            self.get_logger().error(response.message)
            return response

        # 4. Wait for main_current_signal AND robot_motion_release
        #    main_current_signal = main welding current is flowing (arc ignited)
        #    robot_motion_release = starting current elapsed, safe to move
        self.get_logger().info('Waiting for main_current_signal (arc ignition)...')
        if not self._wait_for_signal('main_current_signal', True,
                                     self.robot_motion_release_timeout):
            self.get_logger().error('main_current_signal timeout — arc did not ignite, aborting')
            self._set_wago_signal(self.welding_start_client, False)
            self._set_wago_signal(self.robot_ready_client, False)
            response.success = False
            response.message = (
                f'Start failed: main_current_signal timeout '
                f'({self.robot_motion_release_timeout}s) '
                '- arc did not ignite')
            self.get_logger().error(response.message)
            return response
        self.get_logger().info('main_current_signal confirmed — arc is burning')

        self.get_logger().info('Waiting for robot_motion_release (arc stable)...')
        if self._wait_for_signal('robot_motion_release', True,
                                 self.robot_motion_release_timeout):
            self.get_logger().info('robot_motion_release received - safe to move')
        else:
            # Arc fired but didn't stabilise — stop and rollback
            self.get_logger().error('robot_motion_release timeout — aborting weld')
            self._set_wago_signal(self.welding_start_client, False)
            self._set_wago_signal(self.robot_ready_client, False)
            response.success = False
            response.message = (
                f'Start failed: robot_motion_release timeout '
                f'({self.robot_motion_release_timeout}s) '
                '- arc may not have stabilised')
            self.get_logger().error(response.message)
            return response

        self.is_welding = True
        response.success = True
        response.message = "Welding started - arc stable, robot can move"
        self.get_logger().info('Welding started successfully - robot can begin weld motion')
        return response

    def stop_weld_callback(self, request: Trigger.Request,
                           response: Trigger.Response) -> Trigger.Response:
        """Stop welding: welding_start(false) -> wait for robot_motion_release=false -> robot_ready(false).

        The Fronius power source automatically handles end-current ramp-down
        and gas post-flow (GPo). robot_motion_release drops Low once the full
        cycle (including GPo) is complete.

        Sequence:
        1. Deactivate welding_start (arc off)
        2. Wait for robot_motion_release = False (post-flow complete)
        3. Deactivate robot_ready
        """
        self.get_logger().info('STOP WELD')

        # Signal any in-progress wire operations to abort immediately
        self._abort_wire_ops.set()

        errors = []

        # 1. Stop arc first
        self.get_logger().info('Deactivating welding_start (arc off)...')
        ok, msg = self._set_wago_signal(self.welding_start_client, False)
        if not ok:
            errors.append(f"welding_start: {msg}")

        # 2. Wait for robot_motion_release to drop (post-flow / GPo complete)
        self.get_logger().info('Waiting for robot_motion_release to drop (post-flow)...')
        if self._wait_for_signal('robot_motion_release', False,
                                 self.robot_motion_release_timeout):
            self.get_logger().info('robot_motion_release dropped - post-flow complete')
        else:
            self.get_logger().warn(
                f'robot_motion_release did not drop within '
                f'{self.robot_motion_release_timeout}s (proceeding with stop)')

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
            response.message = "Welding stopped - post-flow complete"
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
        self._wago_signals['touch_signal'] = False
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

        Self-contained sequence:
        1. Clear any stale abort flag
        2. Enable touch sensing
        3. Set WireMoveLength = 0 (continuous feed mode)
        4. Activate WireForward
        5. Wait for TouchSignal = True (Fronius auto-stops wire on contact)
        6. Deactivate WireForward
        7. Disable touch sensing

        Returns:
            Trigger.Response with success/message
        """
        self.get_logger().info('Wire feed until touch: starting...')

        # Clear stale abort flag from previous stop
        self._abort_wire_ops.clear()

        # 1. Ensure robot_ready is set (required for wire motor to engage)
        ok, msg = self._set_wago_signal(self.robot_ready_client, True)
        if not ok:
            response.success = False
            response.message = f"Failed to set robot_ready: {msg}"
            self.get_logger().error(response.message)
            return response

        # 2. Enable touch sensing
        ok, msg = self._set_wago_signal(self.touch_sensing_client, True)
        if not ok:
            response.success = False
            response.message = f"Failed to enable touch sensing: {msg}"
            self.get_logger().error(response.message)
            return response
        self.get_logger().info('Touch sensing enabled')
        time.sleep(0.2)  # allow touch circuit to stabilise

        # Guard: do not start if touch is already active (wire already in contact)
        if self._wago_signals['touch_signal']:
            # Wire is already touching — that's actually success
            self._set_wago_signal(self.touch_sensing_client, False)
            response.success = True
            response.message = 'Wire already in contact with workpiece'
            self.get_logger().info(response.message)
            return response

        # 2. Set continuous feed mode (length = 0)
        ok, msg = self._set_wire_move_length(0.0)
        if not ok:
            self._set_wago_signal(self.touch_sensing_client, False)
            response.success = False
            response.message = f"Failed to set wire_move_length=0: {msg}"
            self.get_logger().error(response.message)
            return response

        # 3. Start feeding
        ok, msg = self._set_wago_signal(self.wire_forward_client, True)
        if not ok:
            self._set_wago_signal(self.touch_sensing_client, False)
            response.success = False
            response.message = f"Failed to activate wire_forward: {msg}"
            self.get_logger().error(response.message)
            return response

        # 4. Wait for touch signal (timeout after 30s for safety)
        timeout = 30.0
        poll_interval = 0.05  # 50ms
        elapsed = 0.0

        stable_touch_count = 0
        required_stable_counts = 2  # 2 x 50ms = 100ms

        while elapsed < timeout:
            if self._abort_wire_ops.is_set():
                self._set_wago_signal(self.wire_forward_client, False)
                self._set_wago_signal(self.touch_sensing_client, False)
                response.success = False
                response.message = "Wire feed aborted"
                self.get_logger().warn(response.message)
                return response

            time.sleep(poll_interval)
            elapsed += poll_interval

            if self._wago_signals['touch_signal']:
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
            self._set_wago_signal(self.touch_sensing_client, False)
            response.success = False
            response.message = (
                f"Wire feed timeout ({timeout}s) — no touch signal detected. "
                "Check wire can reach workpiece.")
            self.get_logger().error(response.message)
            return response

        # 5. Stop wire forward and disable touch sensing
        self._set_wago_signal(self.wire_forward_client, False)
        self._set_wago_signal(self.touch_sensing_client, False)

        response.success = True
        response.message = f"Wire touched workpiece after {elapsed:.2f}s"
        self.get_logger().info(response.message)
        return response

    def wire_retract_callback(self, request: SetFloat32.Request,
                              response: SetFloat32.Response) -> SetFloat32.Response:
        """Retract wire by a specified distance (mm) using timed backward signal.

        Holds wire_backward=True for the time needed to retract the
        requested distance at the configured wire_retract_speed, then
        releases the signal.

        Args:
            request.data: Retraction distance in mm (0 = continuous for 1s)
        """
        retract_mm = request.data
        self.get_logger().info(f'Wire retract: {retract_mm:.1f}mm')

        # Clear stale abort flag from previous stop
        self._abort_wire_ops.clear()

        # Ensure robot_ready is set (required for wire motor to engage).
        # stop_weld clears robot_ready, but wire retract runs after stop.
        ok, msg = self._set_wago_signal(self.robot_ready_client, True)
        if not ok:
            response.success = False
            response.message = f"Failed to set robot_ready: {msg}"
            self.get_logger().error(response.message)
            return response

        # Ensure continuous mode — no fixed-distance PLC mode
        ok, msg = self._set_wire_move_length(0.0)
        if not ok:
            self.get_logger().warn(f"Could not clear wire_move_length: {msg}")

        # Activate backward signal
        ok, msg = self._set_wago_signal(self.wire_backward_client, True)
        if not ok:
            response.success = False
            response.message = f"Failed to activate wire_backward: {msg}"
            return response

        # Hold for the time needed to cover the distance
        speed_mm_s = max(1.0, float(self.wire_retract_speed_mm_s))
        if retract_mm > 0:
            hold_time = (retract_mm / speed_mm_s) + 0.5
        else:
            hold_time = 1.0
        self.get_logger().info(
            f'Wire retract hold={hold_time:.2f}s for {retract_mm:.1f}mm '
            f'(speed={speed_mm_s:.1f}mm/s)')

        # Sleep in small increments so we can respond to abort
        elapsed = 0.0
        poll = 0.05
        while elapsed < hold_time:
            if self._abort_wire_ops.is_set():
                self.get_logger().warn("Wire retract aborted")
                break
            time.sleep(min(poll, hold_time - elapsed))
            elapsed += poll

        # Release backward signal
        self._set_wago_signal(self.wire_backward_client, False)
        time.sleep(0.2)

        # Clear robot_ready so it doesn't leave stale state
        self._set_wago_signal(self.robot_ready_client, False)

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
