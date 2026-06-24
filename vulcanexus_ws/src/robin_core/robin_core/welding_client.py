"""Welding service client — start/stop arc, publish bead state.

This is a data helper: it does NOT create any ROS2 interfaces itself.
All service clients and publishers must be created by the parent node
and passed into the constructor.
"""

import math
import time

from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from robin_interfaces.msg import ActiveBead
from robin_interfaces.srv import StartWeld
from robin_interfaces.srv import SetFloat32 as SetFloat32Srv

from robin_core.utils import wait_for_future


class WeldingClient:
    """Wraps the welding-coordinator service clients.

    Provides :meth:`start`, :meth:`stop`, :meth:`emergency_stop` and
    convenience publishers for the data-node bead tracking.

    All ROS2 interfaces (service clients, publishers) must be created
    by the parent node and passed in via ``clients`` and ``publishers``.
    """

    def __init__(self, node, *,
                 clients: dict,
                 publishers: dict,
                 welding_service_timeout: float = 5.0,
                 parameter_service_timeout: float = 2.0,
                 base_frame: str = "base_link"):
        self._node = node
        self._welding_service_timeout = welding_service_timeout
        self._parameter_service_timeout = parameter_service_timeout
        self._base_frame = base_frame

        self._start_client = clients['welding_start']
        self._set_params_client = clients['welding_set_params']
        self._stop_client = clients['welding_stop']
        self._wire_retract_client = clients['welding_wire_retract']
        self._simulation_client = clients['welding_simulation']

        self._active_bead_pub = publishers['active_bead']
        self._welding_state_pub = publishers['is_welding']
        self._scanning_state_pub = publishers['is_scanning']

    # -- publishing helpers --------------------------------------------------
    def publish_active_bead(self, bead, weld_length: float,
                           voltage_recommvalue: float = 0.0,
                           current_recommvalue: float = 0.0,
                           arc_length_correction_mm: float = 0.0):
        """Publish active bead info for the data-node progression tracker."""
        msg = ActiveBead()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.bead_id = bead.bead_id
        msg.path = list(bead.path)
        msg.total_length = weld_length
        msg.weld_speed = bead.weld_speed
        msg.wire_feed_speed = bead.wire_feed_speed
        msg.voltage_recommvalue = voltage_recommvalue
        msg.current_recommvalue = current_recommvalue
        msg.arc_length_correction_mm = arc_length_correction_mm
        self._active_bead_pub.publish(msg)

    def publish_idle_bead(self):
        """Publish an explicit idle ActiveBead to clear latched bead state."""
        msg = ActiveBead()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.bead_id = ""
        msg.total_length = 0.0
        msg.weld_speed = 0.0
        msg.wire_feed_speed = 0.0
        msg.voltage_recommvalue = 0.0
        msg.current_recommvalue = 0.0
        msg.arc_length_correction_mm = 0.0
        self._active_bead_pub.publish(msg)

    def publish_welding_state(self, is_welding: bool):
        """Publish arc on/off state to the data node."""
        msg = Bool()
        msg.data = is_welding
        self._welding_state_pub.publish(msg)

    def publish_scanning_state(self, is_scanning: bool):
        """Publish scan pass on/off state to the data node."""
        msg = Bool()
        msg.data = is_scanning
        self._scanning_state_pub.publish(msg)

    # -- service calls -------------------------------------------------------
    def _call_service(self, client, request, timeout_sec: float, label: str) -> bool:
        """Wait for service, call it, return True on success."""
        if not client.service_is_ready():
            self._node.get_logger().warn(f"{label} not available, waiting…")
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self._node.get_logger().error(f"{label} not available")
                return False
        try:
            future = client.call_async(request)
            result = wait_for_future(self._node, future, timeout_sec=timeout_sec)
            if result is not None:
                if result.success:
                    self._node.get_logger().info(f"{label}: {result.message}")
                    return True
                self._node.get_logger().error(f"{label} failed: {result.message}")
                return False
            self._node.get_logger().error(f"{label} timed out")
            return False
        except Exception as e:
            self._node.get_logger().error(f"{label} call failed: {e}")
            return False

    def _make_start_request(self, bead, arc_length_correction_mm: float) -> StartWeld.Request:
        req = StartWeld.Request()
        req.wire_speed = float(bead.wire_feed_speed)
        req.arc_length_correction_mm = float(arc_length_correction_mm)
        req.weld_speed = float(bead.weld_speed)
        return req

    def set_params(self, bead, arc_length_correction_mm: float = 0.0) -> bool:
        """Set welding parameters without starting arc."""
        return self._call_service(
            self._set_params_client,
            self._make_start_request(bead, arc_length_correction_mm),
            self._parameter_service_timeout,
            "welding/set_params")

    def start(self, bead, arc_length_correction_mm: float = 0.0) -> bool:
        """Start welding via the welding coordinator."""
        request = self._make_start_request(bead, arc_length_correction_mm)
        self._node.get_logger().info(
            f"Starting welding [synergy]: "
            f"WFS={request.wire_speed:.1f}m/min, "
            f"ArcCorr={request.arc_length_correction_mm:.2f}mm")
        return self._call_service(
            self._start_client, request,
            self._welding_service_timeout, "welding/start")

    def stop(self) -> bool:
        """Stop welding via the welding coordinator."""
        self._node.get_logger().info("Stopping welding…")
        return self._call_service(
            self._stop_client, Trigger.Request(),
            self._welding_service_timeout, "welding/stop")

    def emergency_stop(self, is_welding: bool):
        """Emergency stop: stop arc if active, publish state=False."""
        self._node.get_logger().warn("Emergency stop initiated")
        if is_welding:
            self._node.get_logger().warn("Stopping active weld…")
            self.stop()
        self.publish_welding_state(False)

    def wire_retract(self, retract_m: float) -> bool:
        """Retract wire by given length in meters."""
        req = SetFloat32Srv.Request()
        req.data = float(retract_m) * 1000.0  # service expects mm

        # Timeout must cover the physical retraction time on the PLC side.
        retract_timeout = max(
            self._welding_service_timeout,
            req.data / 5.0 + 3.0)

        return self._call_service(
            self._wire_retract_client, req,
            retract_timeout, "welding/wire_retract")

    def set_simulation_mode(self, enabled: bool) -> bool:
        """Enable/disable PLC welding simulation mode."""
        op = "ENABLED" if enabled else "DISABLED"
        wait_timeout = max(self._welding_service_timeout, self._parameter_service_timeout)
        if not self._simulation_client.service_is_ready():
            self._node.get_logger().warn(
                "wago/in/welding_simulation not available, waiting\u2026")
            if not self._simulation_client.wait_for_service(timeout_sec=wait_timeout):
                self._node.get_logger().error(
                    "wago/in/welding_simulation not available")
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
