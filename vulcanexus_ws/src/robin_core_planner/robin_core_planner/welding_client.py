"""Welding service client — start/stop arc, publish bead state."""

import math
import time

from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from robin_interfaces.msg import ActiveBead
from robin_interfaces.srv import StartWeld
from robin_interfaces.srv import SetFloat32 as SetFloat32Srv

from robin_core_planner.utils import wait_for_future


class WeldingClient:
    """Wraps the welding-coordinator service clients.

    Provides :meth:`start`, :meth:`stop`, :meth:`emergency_stop` and
    convenience publishers for the data-node bead tracking.
    """

    def __init__(self, node, welding_service_timeout: float = 5.0,
                 parameter_service_timeout: float = 2.0,
                 base_frame: str = "base_link"):
        self._node = node
        self._welding_service_timeout = welding_service_timeout
        self._parameter_service_timeout = parameter_service_timeout
        self._base_frame = base_frame

        self._start_client = node.create_client(StartWeld, "/welding/start")
        self._stop_client = node.create_client(Trigger, "/welding/stop")
        self._wire_retract_client = node.create_client(SetFloat32Srv, "/welding/wire_retract")
        self._simulation_client = node.create_client(SetBool, "/wago/in/welding_simulation")

        # Data-node publishers
        self._active_bead_pub = node.create_publisher(
            ActiveBead, "/robin/data/active_bead", 10)
        self._welding_state_pub = node.create_publisher(
            Bool, "/robin/data/is_welding", 10)

    # -- publishing helpers --------------------------------------------------
    def publish_active_bead(self, bead, weld_length: float):
        """Publish active bead info for the data-node progression tracker."""
        msg = ActiveBead()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.bead_id = bead.bead_id
        msg.start_point = bead.start_point
        msg.end_point = bead.end_point
        msg.length = weld_length
        msg.target_speed = bead.target_speed
        msg.target_current = bead.target_current
        msg.target_voltage = bead.target_voltage
        msg.target_wire_speed = bead.wire_feed_speed
        self._active_bead_pub.publish(msg)

    def publish_welding_state(self, is_welding: bool):
        """Publish arc on/off state to the data node."""
        msg = Bool()
        msg.data = is_welding
        self._welding_state_pub.publish(msg)

    # -- service calls -------------------------------------------------------
    def start(self, bead) -> bool:
        """Start welding via the welding coordinator.

        Sends primary parameter, current, voltage, wire_speed to
        ``/welding/start``.  Returns True on success.
        """
        if not self._start_client.service_is_ready():
            self._node.get_logger().warn("/welding/start not available, waiting…")
            if not self._start_client.wait_for_service(
                    timeout_sec=self._parameter_service_timeout):
                self._node.get_logger().error("/welding/start not available")
                return False

        request = StartWeld.Request()
        request.primary_parameter = bead.primary_parameter

        param_names = {0: "CURRENT", 1: "VOLTAGE", 2: "WIRE_FEED_SPEED"}
        primary_name = param_names.get(bead.primary_parameter, "UNKNOWN")

        if bead.primary_parameter == StartWeld.Request.PRIMARY_CURRENT:
            request.current = bead.primary_value
            request.voltage = bead.target_voltage
            request.wire_speed = bead.wire_feed_speed
        elif bead.primary_parameter == StartWeld.Request.PRIMARY_VOLTAGE:
            request.voltage = bead.primary_value
            request.current = bead.target_current
            request.wire_speed = bead.wire_feed_speed
        elif bead.primary_parameter == StartWeld.Request.PRIMARY_WIRE_FEED_SPEED:
            request.wire_speed = bead.primary_value
            request.current = bead.target_current
            request.voltage = bead.target_voltage

        overrides = []
        if bead.target_current > 0.0 and bead.primary_parameter != 0:
            overrides.append(f"I={bead.target_current:.1f}A")
        if bead.target_voltage > 0.0 and bead.primary_parameter != 1:
            overrides.append(f"U={bead.target_voltage:.1f}V")
        if bead.wire_feed_speed > 0.0 and bead.primary_parameter != 2:
            overrides.append(f"WFS={bead.wire_feed_speed:.1f}m/min")

        mode = "synergy" if not overrides else f"manual ({', '.join(overrides)})"
        self._node.get_logger().info(
            f"Starting welding [{mode}]: primary={primary_name}={bead.primary_value:.1f}, "
            f"I={request.current:.1f}A, U={request.voltage:.1f}V, "
            f"WFS={request.wire_speed:.1f}m/min")

        try:
            future = self._start_client.call_async(request)
            result = wait_for_future(
                self._node, future, timeout_sec=self._welding_service_timeout)
            if result is not None:
                if result.success:
                    self._node.get_logger().info(f"Welding started: {result.message}")
                    return True
                self._node.get_logger().error(
                    f"Failed to start welding: {result.message}")
                return False
            self._node.get_logger().error("Start welding service call timed out")
            return False
        except Exception as e:
            self._node.get_logger().error(f"Start welding service call failed: {e}")
            return False

    def stop(self) -> bool:
        """Stop welding via the welding coordinator."""
        if not self._stop_client.service_is_ready():
            self._node.get_logger().warn("/welding/stop not available, waiting…")
            if not self._stop_client.wait_for_service(
                    timeout_sec=self._parameter_service_timeout):
                self._node.get_logger().error("/welding/stop not available")
                return False

        self._node.get_logger().info("Stopping welding…")
        try:
            future = self._stop_client.call_async(Trigger.Request())
            result = wait_for_future(
                self._node, future, timeout_sec=self._welding_service_timeout)
            if result is not None:
                if result.success:
                    self._node.get_logger().info(f"Welding stopped: {result.message}")
                    return True
                self._node.get_logger().error(
                    f"Failed to stop welding: {result.message}")
                return False
            self._node.get_logger().error("Stop welding service call timed out")
            return False
        except Exception as e:
            self._node.get_logger().error(f"Stop welding service call failed: {e}")
            return False

    def emergency_stop(self, is_welding: bool):
        """Emergency stop: stop arc if active, publish state=False."""
        self._node.get_logger().warn("Emergency stop initiated")
        if is_welding:
            self._node.get_logger().warn("Stopping active weld…")
            self.stop()
        self.publish_welding_state(False)

    def wire_retract(self, retract_m: float) -> bool:
        """Retract wire by given length in meters."""
        if not self._wire_retract_client.service_is_ready():
            self._node.get_logger().warn("/welding/wire_retract not available, waiting…")
            if not self._wire_retract_client.wait_for_service(
                    timeout_sec=self._parameter_service_timeout):
                self._node.get_logger().error("/welding/wire_retract not available")
                return False

        req = SetFloat32Srv.Request()
        req.data = float(retract_m)

        try:
            future = self._wire_retract_client.call_async(req)
            result = wait_for_future(
                self._node, future, timeout_sec=self._welding_service_timeout)
            if result is not None:
                if result.success:
                    self._node.get_logger().info(
                        f"Wire retract succeeded ({retract_m * 1000:.1f}mm)")
                    return True
                self._node.get_logger().error(
                    f"Wire retract failed: {result.message}")
                return False
            self._node.get_logger().error("Wire retract service call timed out")
            return False
        except Exception as e:
            self._node.get_logger().error(f"Wire retract service call failed: {e}")
            return False

    def set_simulation_mode(self, enabled: bool) -> bool:
        """Enable/disable PLC welding simulation mode."""
        op = "ENABLED" if enabled else "DISABLED"
        wait_timeout = max(self._welding_service_timeout, self._parameter_service_timeout)
        if not self._simulation_client.service_is_ready():
            self._node.get_logger().warn(
                "/wago/in/welding_simulation not available, waiting…")
            if not self._simulation_client.wait_for_service(timeout_sec=wait_timeout):
                self._node.get_logger().error(
                    "/wago/in/welding_simulation not available")
                return False

        req = SetBool.Request()
        req.data = bool(enabled)

        max_attempts = 3
        for attempt in range(1, max_attempts + 1):
            try:
                future = self._simulation_client.call_async(req)
                result = wait_for_future(
                    self._node, future, timeout_sec=self._welding_service_timeout)
                if result is None:
                    self._node.get_logger().warn(
                        f"Welding simulation set {op} timed out "
                        f"(attempt {attempt}/{max_attempts})")
                elif result.success:
                    self._node.get_logger().info(
                        f"Welding simulation {op}")
                    return True
                else:
                    self._node.get_logger().warn(
                        f"Failed to set welding simulation mode (attempt "
                        f"{attempt}/{max_attempts}): {result.message}")
            except Exception as e:
                self._node.get_logger().warn(
                    f"Welding simulation service call failed "
                    f"(attempt {attempt}/{max_attempts}): {e}")

            if attempt < max_attempts:
                time.sleep(0.2)

        self._node.get_logger().error(
            f"Welding simulation {op} failed after {max_attempts} attempts")
        return False
