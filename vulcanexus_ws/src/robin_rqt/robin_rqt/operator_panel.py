"""
ROBIN Operator Panel - RQT plugin for operator-facing welding system controls.

Tabs:
  Status       - Live WAGO PLC & Fronius feedback (read-only indicators)
    Plates       - Define / edit / manage WeldPlate definitions
  Manual       - Wire jog, gas, error quit, teach mode, welding simulation
    Calibration  - Guided stickout calibration, MoveIt Servo jog, TCP management
                Sensor       - Laser ON/OFF control and live YZ profile plotting

This file is the thin shell that wires everything together.  The heavy
lifting lives in the per-tab modules:
    components/*.py, tabs/*.py
"""

from functools import partial

from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QTabWidget, QTextEdit

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import PointCloud2
from moveit_msgs.srv import ServoCommandType
from robin_interfaces.action import WeldExperiment
from robin_interfaces.srv import (
    SetFloat32 as SetFloat32Srv, CalibrateStickout, CalibratePlatePlane, SetTcpMode,
    PlanExperiment, ApproveExperimentPlan, SensorCommand,
)
from robin_interfaces.msg import FroniusSample
from controller_manager_msgs.srv import SwitchController

import tf2_ros
from qt_gui.plugin import Plugin

from .components.widgets import (
    StatusIndicator, FloatReadout, ToggleServiceButton,
    WAGO_BOOL_TOPICS, WAGO_FLOAT_TOPICS, FRONIUS_FLOAT_TOPICS,
    WAGO_BOOL_SERVICES,
)
from .tabs.status_tab import build_status_tab
from .tabs.plates_tab import build_plates_tab, load_plates_from_config
from .tabs.manual_tab import build_manual_tab, is_robot_ready, ensure_robot_ready
from .tabs.calibration_tab import build_calibration_tab, _disable_servo
from .tabs.experiment_tab import build_experiment_tab
from .tabs.sensor_tab import build_sensor_tab, update_sensor_pointcloud
from .calibration_controller import CalibrationController
from .manual_controller import ManualController
from .panel_state import OperatorPanelState
from .components.jog_component import build_jog_group
from .components.stickout_component import (
    update_stickout_calibration_status,
    update_stickout_value_from_topic,
    update_active_tcp_frame,
)


class OperatorPanel(Plugin):
    """Main RQT Plugin — assembles tabs and routes ROS signals to Qt."""

    # Qt signals for thread-safe UI updates from ROS callbacks
    _sig_bool = Signal(str, bool)
    _sig_float = Signal(str, float)
    _sig_string = Signal(str, str)
    _sig_fronius = Signal(str, float, float, float, float)
    _sig_pointcloud = Signal(object)

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('OperatorPanel')

        # ROS node (re-use the rqt context node)
        self._node: Node = context.node
        self.state = OperatorPanelState()

        # ---- state stores ----
        self._bool_values: dict[str, bool] = {}
        self._float_values: dict[str, float] = {}
        self._bool_indicators: dict[str, StatusIndicator] = {}
        self._float_readouts: dict[str, FloatReadout] = {}

        # Plate definitions stored in-memory
        self._plates: list[dict] = []
        self._plates_path: str = ''
        self._calibration_point: dict | None = None

        # Toggle buttons keyed by service name for readback updates
        self._toggle_buttons: dict[str, ToggleServiceButton] = {}

        # ---- widget tree ----
        self._widget = QWidget()
        self._widget.setWindowTitle('ROBIN Operator Panel')
        layout = QVBoxLayout(self._widget)

        # ---- Robot ready state tracking (needed before tabs) ----
        self._robot_ready = self.state.robot_ready
        self._robot_ready_client = self._node.create_client(
            SetBool, '/wago/in/robot_ready')
        self._manual_ctrl = ManualController(self._node)
        self._robot_ready_btn = None

        # ---- Servo / jog state (needed before calibration tab) ----
        self._servo_active = self.state.calibration.servo_active
        self._jog_publisher = self._node.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self._jog_speed = self.state.calibration.jog_speed
        self._jog_frame = self.state.calibration.jog_frame
        self._jog_twist_dir = self.state.calibration.jog_twist_dir
        self._jog_ros_timer = None    # created once in jog_component (rclpy timer)
        self._switch_ctrl_client = self._node.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self._servo_cmd_type_client = self._node.create_client(
            ServoCommandType, '/servo_node/switch_command_type')
        self._servo_pause_client = self._node.create_client(
            SetBool, '/servo_node/pause_servo')
        self._set_stickout_client = self._node.create_client(
            SetFloat32Srv, '/tcp/set_stickout')
        self._set_tcp_mode_client = self._node.create_client(
            SetTcpMode, '/tcp/set_mode')
        self._calibrate_stickout_client = self._node.create_client(
            CalibrateStickout, '/calibration/calibrate_stickout')
        self._calibrate_plate_plane_client = self._node.create_client(
            CalibratePlatePlane, '/calibration/calibrate_plate_plane')
        self._abort_calibration_client = self._node.create_client(
            Trigger, '/calibration/abort')
        self._plan_experiment_client = self._node.create_client(
            PlanExperiment, '/experiment/plan')
        self._approve_plan_client = self._node.create_client(
            ApproveExperimentPlan, '/experiment/approve')
        self._weld_experiment_action_client = ActionClient(
            self._node, WeldExperiment, '/weld_experiment')
        self._terminate_experiment_client = self._node.create_client(
            Trigger, '/experiment/terminate')
        self._sensor_activate_client = self._node.create_client(
            SensorCommand, '/profilometer_activate')
        self._sensor_deactivate_client = self._node.create_client(
            SensorCommand, '/profilometer_deactivate')
        self._active_experiment_goal_handle = None
        self._current_plan_id = ''
        self._calibration_ctrl = CalibrationController(self)

        # ---- TF listener for live TCP display ----
        self._tcp_active_frame = self.state.calibration.tcp_active_frame
        self._tcp_stickout = self.state.calibration.tcp_stickout
        self._tcp_stickout_calibrated = self.state.calibration.tcp_stickout_calibrated
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self._node)

        # ---- Log panel (set by manual_tab) ----
        self._log_text: QTextEdit | None = None
        self._exp_log: QTextEdit | None = None

        # ---- Fixed global controls (shared across tabs) ----
        layout.addWidget(build_jog_group(self))

        # ---- Build tabs (delegates to tabs package modules) ----
        tabs = QTabWidget()
        tabs.addTab(build_status_tab(self), 'Status')
        tabs.addTab(build_plates_tab(self), 'Plates')
        tabs.addTab(build_manual_tab(self), 'Manual Control')
        tabs.addTab(build_calibration_tab(self), 'Calibration')
        tabs.addTab(build_experiment_tab(self), 'Experiment')
        tabs.addTab(build_sensor_tab(self), 'Sensor')
        layout.addWidget(tabs)

        context.add_widget(self._widget)

        # ---- ROS subscriptions ----
        self._subscriptions = []
        for topic, _ in WAGO_BOOL_TOPICS:
            sub = self._node.create_subscription(
                Bool, topic,
                partial(self._on_bool, topic=topic), 10)
            self._subscriptions.append(sub)

        for topic, _ in WAGO_FLOAT_TOPICS + FRONIUS_FLOAT_TOPICS:
            sub = self._node.create_subscription(
                Float32, topic,
                partial(self._on_float, topic=topic), 10)
            self._subscriptions.append(sub)

        # Subscribe to WAGO IN readback topics (actual PLC state)
        for srv, _label in WAGO_BOOL_SERVICES:
            readback_topic = srv + '/state'
            sub = self._node.create_subscription(
                Bool, readback_topic,
                partial(self._on_bool, topic=readback_topic), 10)
            self._subscriptions.append(sub)

        # Connect Qt signals (thread-safe)
        self._sig_bool.connect(self._update_bool_indicator)
        self._sig_float.connect(self._update_float_readout)
        self._sig_string.connect(self._update_string_readout)
        self._sig_fronius.connect(self._update_fronius_plot)
        self._sig_pointcloud.connect(self._update_sensor_pointcloud)

        # TCP manager subscriptions
        self._tcp_frame_sub = self._node.create_subscription(
            String, '/tcp/active_frame',
            lambda msg: self._sig_string.emit('/tcp/active_frame', msg.data), 10)
        self._tcp_stickout_sub = self._node.create_subscription(
            Float32, '/tcp/stickout',
            partial(self._on_float, topic='/tcp/stickout'), 10)
        self._tcp_stickout_cal_sub = self._node.create_subscription(
            Bool, '/tcp/stickout_calibrated',
            partial(self._on_bool, topic='/tcp/stickout_calibrated'), 10)
        self._subscriptions.extend([
            self._tcp_frame_sub, self._tcp_stickout_sub,
            self._tcp_stickout_cal_sub])

        self._fronius_plot_sub = self._node.create_subscription(
            FroniusSample, '/robin/data/fronius', self._on_fronius_sample, 10)
        self._subscriptions.append(self._fronius_plot_sub)

        self._pointcloud_sub = self._node.create_subscription(
            PointCloud2, '/robin/pointcloud', self._on_pointcloud, 10)
        self._subscriptions.append(self._pointcloud_sub)

        # ---- Auto-load plates from config ----
        load_plates_from_config(self)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown_plugin(self):
        if self.state.calibration.servo_active:
            _disable_servo(self)
        if hasattr(self, '_tcp_pos_timer'):
            self._tcp_pos_timer.stop()
        for sub in self._subscriptions:
            self._node.destroy_subscription(sub)
        self._subscriptions.clear()

    # ------------------------------------------------------------------
    # Public helpers used by widget classes
    # ------------------------------------------------------------------
    def is_robot_ready(self) -> bool:
        return is_robot_ready(self)

    def ensure_robot_ready(self):
        ensure_robot_ready(self)

    # ------------------------------------------------------------------
    # ROS callbacks → Qt signals
    # ------------------------------------------------------------------
    def _on_bool(self, msg: Bool, topic: str):
        self._bool_values[topic] = msg.data
        self._sig_bool.emit(topic, msg.data)

    def _on_float(self, msg: Float32, topic: str):
        self._float_values[topic] = msg.data
        self._sig_float.emit(topic, msg.data)

    def _on_fronius_sample(self, msg: FroniusSample):
        self._sig_fronius.emit(
            msg.bead_id,
            float(msg.progression),
            float(msg.current),
            float(msg.voltage),
            float(msg.wire_feed_speed),
        )

    def _on_pointcloud(self, msg: PointCloud2):
        self._sig_pointcloud.emit(msg)

    def _update_bool_indicator(self, topic: str, value: bool):
        ind = self._bool_indicators.get(topic)
        if ind:
            ind.set_value(value)

        # Route readback topics to toggle buttons
        if topic.endswith('/state'):
            srv_name = topic[:-len('/state')]
            btn = self._toggle_buttons.get(srv_name)
            if btn is not None:
                btn.set_from_readback(value)
            if srv_name == '/wago/in/robot_ready':
                self.state.robot_ready = value
                self._robot_ready = value

        # Update stickout calibration indicator
        if topic == '/tcp/stickout_calibrated':
            update_stickout_calibration_status(self, value)

    def _update_float_readout(self, topic: str, value: float):
        ro = self._float_readouts.get(topic)
        if ro:
            ro.set_value(value)
        if topic == '/tcp/stickout':
            update_stickout_value_from_topic(self, value)

    def _update_string_readout(self, topic: str, value: str):
        if topic == '/tcp/active_frame':
            update_active_tcp_frame(self, value)

    def _update_fronius_plot(self, bead_id: str, progression: float,
                             current: float, voltage: float, wfs: float):
        plot = getattr(self, '_exp_process_plot', None)
        if plot is not None:
            plot.add_sample(bead_id, progression, current, voltage, wfs)

    def _update_sensor_pointcloud(self, msg: PointCloud2):
        update_sensor_pointcloud(self, msg)
