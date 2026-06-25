"""CalibrationManager — ROS service host for calibration operations.

Creates calibration services on the parent node and delegates to
ProbeCalibration. Provides abort() for operator-initiated cancellation.
"""

from std_srvs.srv import Trigger

from robin_interfaces.srv import (
    FindSurface,
    CalibrateWireTip,
    CalibratePlatePlane,
)

from robin_core.calibration.probe_calibration import ProbeCalibration


class CalibrationManager:
    """Hosts calibration ROS services and manages probe lifecycle.

    Instantiated by the planner node, which passes its own node handle,
    callback group, and references to the MotionPlanner and TcpHelper.

    Attributes:
        probe: The underlying ProbeCalibration instance.
    """

    def __init__(self, node, cb_group, controller_name: str):
        self._node = node
        self._cb_group = cb_group
        self.probe = ProbeCalibration(node, cb_group, controller_name)

        # These are set by the planner node after initialization
        self._planner = None
        self._tcp = None
        self._is_executing_fn = None

    def create_services(self):
        """Create calibration ROS services on the parent node.

        Call this only after all dependencies are ready (MoveIt, controller)
        so that services only become visible when truly operational.
        """
        node = self._node
        cb_group = self._cb_group

        node.create_service(
            FindSurface, "calibration/find_surface",
            self._find_surface_callback, callback_group=cb_group)
        node.create_service(
            CalibrateWireTip, "calibration/calibrate_wire_tip",
            self._calibrate_wire_tip_callback, callback_group=cb_group)
        node.create_service(
            CalibratePlatePlane, "calibration/calibrate_plate_plane",
            self._calibrate_plate_plane_callback, callback_group=cb_group)
        node.create_service(
            Trigger, "calibration/abort",
            self._abort_callback, callback_group=cb_group)

        node.get_logger().info(
            "Calibration services: calibration/find_surface, "
            "calibration/calibrate_wire_tip, calibration/calibrate_plate_plane, "
            "calibration/abort")

    def set_dependencies(self, planner, tcp, is_executing_fn):
        """Inject runtime dependencies from the planner node.

        Args:
            planner: MotionPlanner instance
            tcp: TcpHelper instance
            is_executing_fn: callable returning True if an experiment is in progress
        """
        self._planner = planner
        self._tcp = tcp
        self._is_executing_fn = is_executing_fn

    def abort(self):
        """Request abort of any in-progress calibration."""
        self.probe.request_abort()

    def request_abort(self):
        """Request abort of any in-progress calibration."""
        self.probe.request_abort()

    def clear_abort(self):
        """Clear any pending abort flag on the probe."""
        self.probe.clear_abort()

    # -- Service callbacks ---------------------------------------------------
    def _abort_callback(self, _request, response):
        self.probe.request_abort()
        response.success = True
        response.message = "Calibration abort requested"
        self._node.get_logger().warn(response.message)
        return response

    def _find_surface_callback(self, request, response):
        if self._is_executing_fn and self._is_executing_fn():
            response.success = False
            response.message = "Cannot probe: weld experiment in progress"
            return response
        self.probe.clear_abort()
        return self.probe.find_surface(
            request, response, self._planner,
            self._node.end_effector_link, self._node.base_frame,
            self._node.max_cartesian_velocity, self._node.default_acceleration_scaling)

    def _calibrate_wire_tip_callback(self, request, response):
        if self._is_executing_fn and self._is_executing_fn():
            response.success = False
            response.message = "Cannot calibrate: weld experiment in progress"
            return response
        # Disable welding simulation for touch sensing (required for real
        # Fronius wire energisation); user can re-enable from the panel.
        welding = getattr(self._node, 'welding', None)
        if welding is not None:
            welding.set_simulation_mode(False)
        return self.calibrate_wire_tip_impl(
            request, response, allow_during_execution=False)

    def calibrate_wire_tip_impl(self, request, response, allow_during_execution: bool):
        """Run wire tip calibration, optionally allowing it during experiment execution."""
        if self._is_executing_fn and self._is_executing_fn():
            if not allow_during_execution:
                response.success = False
                response.message = "Cannot calibrate: weld experiment in progress"
                return response
        self.probe.clear_abort()
        target = getattr(request, 'target_wire_tip', 0.0) or 0.0
        return self.probe.calibrate_wire_tip(
            request, response, self._planner,
            self._node.end_effector_link, self._node.base_frame,
            self._node.approach_height, self._tcp.current_ctwd,
            self._tcp.tf_buffer,
            self._node.max_cartesian_velocity, self._node.default_acceleration_scaling,
            target_wire_tip=float(target))

    def _calibrate_plate_plane_callback(self, request, response):
        if self._is_executing_fn and self._is_executing_fn():
            response.success = False
            response.message = "Cannot calibrate plate: weld experiment in progress"
            return response
        self.probe.clear_abort()
        return self.probe.calibrate_plate_plane(
            request, response, self._planner,
            "wire_tip", self._node.base_frame,
            self._node.max_cartesian_velocity, self._node.default_acceleration_scaling)
