"""
ROBIN Operator Panel - RQT plugin for operator-facing welding system controls.

Tabs (touch-optimized for quarter-screen on 24" display):
  Control      - Status indicators, PLC toggles, wire control, laser ON/OFF
  Setup        - Plate management, CTWD/TCP calibration, jog controls
  Experiment   - Schema planning, approval, and execution workflow

This file is the thin shell that wires everything together.  The heavy
lifting lives in the per-tab modules:
    components/*.py, tabs/*.py
"""

from functools import partial

from python_qt_binding.QtCore import Signal, QTimer
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QTabWidget, QTextEdit, QLabel, QScrollArea,
)

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State

from rclpy.action import ActionClient
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from robin_interfaces.action import WeldExperiment, ExecuteBead
from robin_interfaces.srv import (
    SetFloat32 as SetFloat32Srv, SetCtwd, CalibrateWireTip, CalibratePlatePlane, SetTcpMode,
    PlanExperiment, ApproveExperimentPlan,
)
from controller_manager_msgs.srv import SwitchController

import tf2_ros
from qt_gui.plugin import Plugin

from .components.widgets import (
    StatusIndicator, FloatReadout, ToggleServiceButton,
    WAGO_BOOL_TOPICS, WAGO_FLOAT_TOPICS, FRONIUS_FLOAT_TOPICS,
    WAGO_BOOL_SERVICES,
)
from .tabs.control_tab import build_control_tab, is_robot_ready, ensure_robot_ready
from .tabs.setup_tab import build_setup_tab
from .tabs.plates_tab import load_plates_from_config
from .tabs.experiment_tab import build_experiment_tab
from .calibration_controller import CalibrationController
from .manual_controller import ManualController
from .panel_state import OperatorPanelState
from .components.jog_component_v2 import disable_servo as _disable_servo
from .components.ctwd_component import (
    build_live_tcp_widget,
    switch_tcp_mode,
    update_ctwd_calibration_status,
    update_ctwd_value_from_topic,
    update_active_tcp_frame,
)


class OperatorPanel(Plugin):
    """Main RQT Plugin — assembles tabs and routes ROS signals to Qt."""

    # Qt signals for thread-safe UI updates from ROS callbacks
    _sig_bool = Signal(str, bool)
    _sig_float = Signal(str, float)
    _sig_string = Signal(str, str)

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
        layout.setSpacing(2)
        layout.setContentsMargins(2, 2, 2, 2)

        # Backend health banner — hidden by default, shown only on error
        self._backend_health_label = QLabel('')
        self._backend_health_label.setStyleSheet(
            'font-size: 12px; font-weight: bold; padding: 4px 8px; '
            'background-color: #7F0000; color: #FFEBEE; border-radius: 4px;')
        self._backend_health_label.setVisible(False)
        layout.addWidget(self._backend_health_label)

        # ---- Robot ready state tracking (needed before tabs) ----
        self._robot_ready_client = self._node.create_client(
            SetBool, '/wago/in/robot_ready')
        self._manual_ctrl = ManualController(self._node)
        self._robot_ready_btn = None

        # ---- Servo / jog state (needed before setup tab) ----
        self._servo_active = self.state.calibration.servo_active
        self._jog_publisher = self._node.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self._jog_speed = self.state.calibration.jog_speed
        self._jog_frame = self.state.calibration.jog_frame
        self._jog_twist_dir = self.state.calibration.jog_twist_dir
        self._jog_ros_timer = None
        self._switch_ctrl_client = self._node.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self._servo_cmd_type_client = self._node.create_client(
            ServoCommandType, '/servo_node/switch_command_type')
        self._servo_pause_client = self._node.create_client(
            SetBool, '/servo_node/pause_servo')
        self._set_ctwd_client = self._node.create_client(
            SetCtwd, '/tcp/set_ctwd')
        self._set_tcp_mode_client = self._node.create_client(
            SetTcpMode, '/tcp/set_mode')
        self._calibrate_wire_tip_client = self._node.create_client(
            CalibrateWireTip, '/calibration/calibrate_wire_tip')
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
        self._execute_bead_action_client = ActionClient(
            self._node, ExecuteBead, '/execute_bead')
        self._terminate_experiment_client = self._node.create_client(
            Trigger, '/experiment/terminate')
        self._continue_experiment_client = self._node.create_client(
            Trigger, '/experiment/continue')
        self._sensor_change_state = self._node.create_client(
            ChangeState, '/garmo_sensor_node/change_state')
        self._sensor_get_state = self._node.create_client(
            GetState, '/garmo_sensor_node/get_state')
        self._active_experiment_goal_handle = None
        self._current_plan_id = ''
        self._calibration_ctrl = CalibrationController(self)

        # ---- TF listener for live TCP display ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self._node)

        # ---- Log panels (set by tab builders) ----
        self._log_text: QTextEdit | None = None
        self._exp_log: QTextEdit | None = None
        self._cal_log: QTextEdit | None = None

        # ---- Build 3-tab layout ────────────────────────────────────
        tabs = QTabWidget()
        tabs.setStyleSheet(
            'QTabBar::tab { min-height: 36px; min-width: 90px; '
            'font-size: 14px; font-weight: bold; padding: 4px 12px; }')

        for tab_widget, title in [
            (build_control_tab(self), 'Control'),
            (build_setup_tab(self), 'Setup'),
            (build_experiment_tab(self), 'Experiment'),
        ]:
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QScrollArea.NoFrame)
            scroll.setWidget(tab_widget)
            tabs.addTab(scroll, title)

        layout.addWidget(tabs)

        # Cap panel height so plot widget underneath stays visible.
        # Target ≈ top 2/3 of a 1080p display (~700 px).
        self._widget.setMaximumHeight(700)

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

        # TCP manager subscriptions (latched QoS to match publisher)
        from rclpy.qos import QoSProfile, DurabilityPolicy
        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._tcp_frame_sub = self._node.create_subscription(
            String, '/tcp/active_frame',
            lambda msg: self._sig_string.emit('/tcp/active_frame', msg.data),
            _latched)
        self._tcp_ctwd_sub = self._node.create_subscription(
            Float32, '/tcp/ctwd',
            partial(self._on_float, topic='/tcp/ctwd'), _latched)
        self._tcp_ctwd_cal_sub = self._node.create_subscription(
            Bool, '/tcp/ctwd_calibrated',
            partial(self._on_bool, topic='/tcp/ctwd_calibrated'), _latched)
        self._subscriptions.extend([
            self._tcp_frame_sub, self._tcp_ctwd_sub,
            self._tcp_ctwd_cal_sub])

        # ---- Auto-load plates from config ----
        load_plates_from_config(self)

        # ---- Backend health check (error-only banner) ----
        self._backend_health_timer = QTimer()
        self._backend_health_timer.timeout.connect(
            self._update_backend_health_banner)
        self._backend_health_timer.start(2000)
        self._update_backend_health_banner()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown_plugin(self):
        if hasattr(self, '_backend_health_timer'):
            self._backend_health_timer.stop()
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

        # Update CTWD calibration indicator
        if topic == '/tcp/ctwd_calibrated':
            update_ctwd_calibration_status(self, value)

    def _update_float_readout(self, topic: str, value: float):
        ro = self._float_readouts.get(topic)
        if ro:
            ro.set_value(value)
        if topic == '/tcp/ctwd':
            update_ctwd_value_from_topic(self, value)

    def _update_string_readout(self, topic: str, value: str):
        if topic == '/tcp/active_frame':
            update_active_tcp_frame(self, value)

    def _update_backend_health_banner(self):
        checks = [
            ('/calibration/calibrate_wire_tip',
             self._calibrate_wire_tip_client.service_is_ready()),
            ('/calibration/calibrate_plate_plane',
             self._calibrate_plate_plane_client.service_is_ready()),
            ('/experiment/plan',
             self._plan_experiment_client.service_is_ready()),
            ('/experiment/approve',
             self._approve_plan_client.service_is_ready()),
            ('/execute_bead',
             self._execute_bead_action_client.server_is_ready()),
            ('/weld_experiment',
             self._weld_experiment_action_client.server_is_ready()),
        ]

        missing = [name for name, ready in checks if not ready]
        if not missing:
            # All services ready — hide banner
            self._backend_health_label.setVisible(False)
            return

        self._backend_health_label.setText(
            'MISSING: ' + ', '.join(missing))
        self._backend_health_label.setStyleSheet(
            'font-size: 12px; font-weight: bold; padding: 4px 8px; '
            'background-color: #7F0000; color: #FFEBEE; border-radius: 4px;')
        self._backend_health_label.setVisible(True)
