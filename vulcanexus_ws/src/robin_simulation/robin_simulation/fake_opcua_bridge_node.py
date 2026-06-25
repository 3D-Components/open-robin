"""Fake OPC UA bridge for simulation.

Replaces the real opcua_bridge node by providing identical ROS2 topics
and services without any OPC UA / PLC connection.  Routes all I/O through
a FroniusSimulator instance that models the power source state machine.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs import msg
from std_msgs.msg import Bool, Float32, Float64, String
from std_srvs.srv import SetBool
from robin_interfaces.srv import SetFloat32, SetInt32
from robin_interfaces.msg import ActiveBead

from robin_simulation.fronius_simulator import FroniusSimulator


LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


# ---------------------------------------------------------------------------
# Topic / service declarations — mirrors opcua_bridge.yaml exactly
# ---------------------------------------------------------------------------

FRONIUS_FLOAT_TOPICS = [
    'fronius/display_current',
    'fronius/display_voltage',
    'fronius/display_wfs',
    'fronius/display_power',
    'fronius/display_energy',
    'fronius/actual_power',
    'fronius/actual_voltage',
    'fronius/actual_current',
    'fronius/actual_wfs',
    'fronius/current_recommvalue',
    'fronius/voltage_recommvalue',
    'fronius/wfs_commandvalue',
    'fronius/arc_length_correction',
    'fronius/wire_position',
]

# Fronius param name used by set_fronius_param() keyed by service name
FRONIUS_SERVICE_TO_PARAM = {
    'fronius/set_current': 'current_recommvalue',
    'fronius/set_voltage': 'voltage_recommvalue',
    'fronius/set_wire_speed': 'wfs_commandvalue',
    'fronius/set_arc_length_correction': 'arc_length_correction',
}

WAGO_OUT_BOOL_TOPICS = [
    'wago/out/arc_stable',
    'wago/out/current_flow',
    'wago/out/heartbeat',
    'wago/out/main_current_signal',
    'wago/out/power_source_ready',
    'wago/out/process_active',
    'wago/out/robot_motion_release',
    'wago/out/touch_signal',
    'wago/out/touch_signal_gas_nozzle',
    'wago/out/warning',
]

WAGO_OUT_FLOAT_TOPICS = [
    'wago/out/welding_current',
    'wago/out/welding_voltage',
    'wago/out/wire_feed_speed',
]

WAGO_IN_BOOL_READBACK = [
    'wago/in/robot_ready/state',
    'wago/in/gas_on/state',
    'wago/in/error_quit/state',
    'wago/in/teach_mode/state',
    'wago/in/torch_blow_out/state',
    'wago/in/touch_sensing/state',
    'wago/in/welding_simulation/state',
    'wago/in/welding_start/state',
    'wago/in/wire_backward/state',
    'wago/in/wire_forward/state',
    'wago/in/wire_sense_break/state',
    'wago/in/wire_sense_start/state',
]

WAGO_IN_BOOL_SERVICES = [
    'wago/in/error_quit',
    'wago/in/gas_on',
    'wago/in/robot_ready',
    'wago/in/teach_mode',
    'wago/in/torch_blow_out',
    'wago/in/touch_sensing',
    'wago/in/welding_simulation',
    'wago/in/welding_start',
    'wago/in/wire_backward',
    'wago/in/wire_forward',
    'wago/in/wire_sense_break',
    'wago/in/wire_sense_start',
]

WAGO_IN_FLOAT_SERVICES = [
    'wago/in/welding_speed',
    'wago/in/wire_move_length',
]

WAGO_IN_INT_SERVICES = [
    'wago/in/working_mode',
]

# Map from service name "wago/in/X" → simulator input key "X"
def _wago_service_to_input(service_name: str) -> str:
    return service_name.removeprefix('wago/in/')

# Map from topic name "wago/out/X" → simulator output key "X"
def _wago_topic_to_output(topic_name: str) -> str:
    return topic_name.removeprefix('wago/out/')


class FakeOpcUaBridge(Node):
    """Drop-in replacement for opcua_bridge in simulation."""

    def __init__(self):
        super().__init__('opcua_bridge')
        self._pubs = {}
        self._srvs = []
        self._timer = None
        self._sim = FroniusSimulator()

        cb = ReentrantCallbackGroup()

        # --- Publishers ---
        for name in FRONIUS_FLOAT_TOPICS:
            self._pubs[name] = self.create_publisher(Float32, name, 10)

        for name in WAGO_OUT_BOOL_TOPICS:
            self._pubs[name] = self.create_publisher(Bool, name, 10)

        for name in WAGO_OUT_FLOAT_TOPICS:
            self._pubs[name] = self.create_publisher(Float32, name, 10)

        for name in WAGO_IN_BOOL_READBACK:
            self._pubs[name] = self.create_publisher(Bool, name, 10)

        self._weld_active_pub = self.create_publisher(Bool, '/robin/weld_active', 10)
        self._weld_current_pub = self.create_publisher(Float64, '/robin/weld_current', 10)
        self._weld_wfs_pub = self.create_publisher(Float64, '/robin/weld_wfs', 10)
        self._weld_voltage_pub = self.create_publisher(Float64, '/robin/weld_voltage', 10)
        self._weld_travel_speed_pub = self.create_publisher(Float64, '/robin/weld_travel_speed', 10)
        self._weld_bead_id_pub = self.create_publisher(String, '/robin/weld_bead_id', 10)

        # Subscribe to active bead from planner to forward bead_id to Gazebo
        self._active_bead_sub = self.create_subscription(
            ActiveBead, 'robin/data/active_bead',
            self._active_bead_callback, LATCHED_QOS)
        self._current_bead_id = ''

        # --- Services ---

        for name in FRONIUS_SERVICE_TO_PARAM:
            srv = self.create_service(
                SetFloat32, name,
                lambda req, resp, n=name: self._handle_fronius_set(req, resp, n),
                callback_group=cb)
            self._srvs.append(srv)

        for name in WAGO_IN_BOOL_SERVICES:
            srv = self.create_service(
                SetBool, name,
                lambda req, resp, n=name: self._handle_wago_bool(req, resp, n),
                callback_group=cb)
            self._srvs.append(srv)

        for name in WAGO_IN_FLOAT_SERVICES:
            srv = self.create_service(
                SetFloat32, name,
                lambda req, resp, n=name: self._handle_wago_float(req, resp, n),
                callback_group=cb)
            self._srvs.append(srv)

        for name in WAGO_IN_INT_SERVICES:
            srv = self.create_service(
                SetInt32, name,
                lambda req, resp, n=name: self._handle_wago_int(req, resp, n),
                callback_group=cb)
            self._srvs.append(srv)

        self.get_logger().info(
            f'Configured: {len(self._pubs)} topics, {len(self._srvs)} services')

        # Start 50 Hz timer
        self._timer = self.create_timer(0.02, self._tick_and_publish)
        self.get_logger().info('Fronius simulator running at 50 Hz')

    # -- Service handlers ---------------------------------------------------

    def _active_bead_callback(self, msg: ActiveBead):
        self._current_bead_id = msg.bead_id

    def _handle_fronius_set(self, request, response, service_name):
        param = FRONIUS_SERVICE_TO_PARAM[service_name]
        self._sim.set_fronius_param(param, request.data)
        response.success = True
        response.message = f'Set to {request.data}'
        return response

    def _handle_wago_bool(self, request, response, service_name):
        key = _wago_service_to_input(service_name)
        self._sim.set_input(key, request.data)
        response.success = True
        response.message = 'ON' if request.data else 'OFF'
        return response

    def _handle_wago_float(self, request, response, service_name):
        key = _wago_service_to_input(service_name)
        self._sim.set_input(key, request.data)
        response.success = True
        response.message = f'Set to {request.data}'
        return response

    def _handle_wago_int(self, request, response, service_name):
        key = _wago_service_to_input(service_name)
        self._sim.set_input(key, int(request.data))
        response.success = True
        response.message = f'Set to {request.data}'
        return response

    # -- Tick + publish -----------------------------------------------------

    def _tick_and_publish(self):
        self._sim.tick()

        # WAGO OUT bools
        outputs = self._sim.get_outputs()
        for topic in WAGO_OUT_BOOL_TOPICS:
            key = _wago_topic_to_output(topic)
            msg = Bool()
            msg.data = bool(outputs[key])
            self._pubs[topic].publish(msg)

        # WAGO OUT floats
        for topic in WAGO_OUT_FLOAT_TOPICS:
            key = _wago_topic_to_output(topic)
            msg = Float32()
            msg.data = float(outputs[key])
            self._pubs[topic].publish(msg)

        # WAGO IN readback (bool)
        for topic in WAGO_IN_BOOL_READBACK:
            # "wago/in/robot_ready/state" → input key "robot_ready"
            key = topic.removeprefix('wago/in/').removesuffix('/state')
            val = self._sim.get_input(key)
            msg = Bool()
            msg.data = bool(val) if val is not None else False
            self._pubs[topic].publish(msg)

        # Fronius display/actual values
        display = self._sim.get_fronius_display()
        for topic in FRONIUS_FLOAT_TOPICS:
            msg = Float32()
            msg.data = float(display.get(topic, 0.0))
            self._pubs[topic].publish(msg)

        is_arc_active = self._sim.state.name in (
            'STARTING_CURRENT', 'WELDING', 'END_CURRENT')
        weld_msg = Bool()
        weld_msg.data = is_arc_active
        self._weld_active_pub.publish(weld_msg)

        # Forward bead_id to Gazebo plugin
        bead_id_msg = String()
        bead_id_msg.data = self._current_bead_id
        self._weld_bead_id_pub.publish(bead_id_msg)

        if is_arc_active:
            outputs = self._sim.get_outputs()
            m = Float64(); m.data = float(outputs.get('welding_current', 0.0))
            self._weld_current_pub.publish(m)
            m = Float64(); m.data = float(outputs.get('wire_feed_speed', 0.0))
            self._weld_wfs_pub.publish(m)
            m = Float64(); m.data = float(outputs.get('welding_voltage', 0.0))
            self._weld_voltage_pub.publish(m)
            m = Float64(); m.data = float(self._sim.get_input('welding_speed') or 0.0) * 1000.0
            self._weld_travel_speed_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOpcUaBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
