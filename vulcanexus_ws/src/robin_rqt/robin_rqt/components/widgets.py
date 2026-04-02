"""
Reusable Qt widgets and constants for the ROBIN Operator Panel.
"""

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QLabel, QPushButton

from rclpy.node import Node
from std_srvs.srv import SetBool


# ---------------------------------------------------------------------------
# Topic / service constant tables
# ---------------------------------------------------------------------------

WAGO_BOOL_TOPICS = [
    ('/wago/out/power_source_ready', 'Power Source Ready'),
    ('/wago/out/process_active',     'Process Active'),
    ('/wago/out/touch_signal',       'Touch Signal'),
    ('/wago/out/warning',            'Warning'),
    ('/wago/out/arc_stable',         'Arc Stable'),
    ('/wago/out/current_flow',       'Current Flow'),
    ('/wago/out/heartbeat',          'Heartbeat'),
    ('/wago/out/main_current_signal','Main Current Signal'),
    ('/wago/out/robot_motion_release','Robot Motion Release'),
    ('/wago/out/touch_signal_gas_nozzle','Touch Signal (Gas Nozzle)'),
]

WAGO_FLOAT_TOPICS = [
    ('/wago/out/welding_current',  'Welding Current (A)'),
    ('/wago/out/welding_voltage',  'Welding Voltage (V)'),
    ('/wago/out/wire_feed_speed',  'Wire Feed Speed (m/min)'),
]

FRONIUS_FLOAT_TOPICS = [
    ('/fronius/display_current', 'Display Current (A)'),
    ('/fronius/display_voltage', 'Display Voltage (V)'),
    ('/fronius/display_wfs',     'Display WFS (m/min)'),
    ('/fronius/display_power',   'Display Power (W)'),
]

WAGO_BOOL_SERVICES = [
    ('/wago/in/robot_ready',          'Robot Ready'),
    ('/wago/in/wire_forward',         'Wire Forward'),
    ('/wago/in/wire_backward',        'Wire Backward'),
    ('/wago/in/gas_on',               'Gas On/Off'),
    ('/wago/in/error_quit',           'Error Quit'),
    ('/wago/in/teach_mode',           'Teach Mode'),
    ('/wago/in/torch_blow_out',       'Torch Blow-Out'),
    ('/wago/in/welding_simulation',   'Welding Simulation'),
    ('/wago/in/touch_sensing',        'Touch Sensing'),
    ('/wago/in/wire_sense_break',     'Wire Sense Break'),
    ('/wago/in/wire_sense_start',     'Wire Sense Start'),
]

# PLC signals that require robot_ready to be set before activation
REQUIRES_ROBOT_READY = {
    '/wago/in/wire_forward',
    '/wago/in/wire_backward',
    '/wago/in/gas_on',
    '/wago/in/welding_simulation',
    '/wago/in/touch_sensing',
    '/wago/in/wire_sense_break',
    '/wago/in/wire_sense_start',
}

WORKING_MODES = {
    0:  'Internal (Synergetic)',
    1:  'Special 2-Step',
    2:  'Job Mode',
    8:  '2-Step Characteristic',
    9:  'MIG/MAG Manual',
    24: 'R/L Measurement',
    25: 'R/L Alignment',
}


# ---------------------------------------------------------------------------
# StatusIndicator — traffic-light indicator label
# ---------------------------------------------------------------------------
class StatusIndicator(QLabel):
    """A small coloured indicator (circle) with a text label."""

    _CSS = (
        "QLabel {{ "
        "  background-color: {bg}; "
        "  color: {fg}; "
        "  border: 1px solid #555; "
        "  border-radius: 4px; "
        "  padding: 2px 8px; "
        "  font-weight: bold; "
        "}}"
    )

    def __init__(self, label_text: str, parent=None):
        super().__init__(label_text, parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(100)
        self._set_state(False)

    def _set_state(self, on: bool):
        if on:
            self.setStyleSheet(self._CSS.format(bg='#4CAF50', fg='white'))
        else:
            self.setStyleSheet(self._CSS.format(bg='#616161', fg='#bbb'))

    def set_value(self, on: bool):
        self._set_state(on)


# ---------------------------------------------------------------------------
# FloatReadout — numeric readout label for float topics
# ---------------------------------------------------------------------------
class FloatReadout(QLabel):
    """Numeric readout label for float topics."""

    _CSS = (
        "QLabel { font-family: monospace; font-size: 14px; padding: 2px 6px; }"
    )

    def __init__(self, parent=None):
        super().__init__('---', parent)
        self.setStyleSheet(self._CSS)
        self.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.setMinimumWidth(90)

    def set_value(self, v: float):
        self.setText(f'{v:.1f}')


# ---------------------------------------------------------------------------
# ToggleServiceButton — toggle button for SetBool services
# ---------------------------------------------------------------------------
class ToggleServiceButton(QPushButton):
    """A push-button that toggles a SetBool service on/off.

    The button sends commands via a SetBool service and can also
    reflect the actual PLC state via ``set_from_readback``.
    """

    def __init__(self, node: Node, service_name: str, label: str,
                 panel=None, parent=None):
        super().__init__(label, parent)
        self._node = node
        self._service_name = service_name
        self._client = node.create_client(SetBool, service_name)
        self._panel = panel
        self._state = False
        self._label = label
        self.setCheckable(True)
        self.setChecked(False)
        self._update_style()
        self.clicked.connect(self._on_click)

    def _on_click(self):
        desired = self.isChecked()

        # Enforce robot_ready prerequisite for certain signals
        if (desired
                and self._service_name in REQUIRES_ROBOT_READY
                and self._panel is not None
                and not self._panel.is_robot_ready()):
            self._node.get_logger().warn(
                f'{self._service_name}: robot_ready is not set — '
                'auto-enabling robot_ready first')
            self._panel.ensure_robot_ready()

        self._state = desired
        self._update_style()
        req = SetBool.Request()
        req.data = self._state
        if self._client.service_is_ready():
            self._client.call_async(req)
        else:
            self._node.get_logger().warn(
                f'Service {self._client.srv_name} not available')

    def set_from_readback(self, value: bool):
        """Update visual state from the actual PLC readback value."""
        if value == self._state:
            return
        self._state = value
        self.setChecked(value)
        self._update_style()

    def _update_style(self):
        if self._state:
            self.setStyleSheet(
                'QPushButton { background-color: #4CAF50; color: white; '
                'font-weight: bold; padding: 6px 12px; }')
            self.setText(f'{self._label}  ● ON')
        else:
            self.setStyleSheet(
                'QPushButton { background-color: #424242; color: #ccc; '
                'padding: 6px 12px; }')
            self.setText(f'{self._label}  ○ OFF')


# ---------------------------------------------------------------------------
# MomentaryServiceButton — active while pressed
# ---------------------------------------------------------------------------
class MomentaryServiceButton(QPushButton):
    """A button that sends True on press and False on release."""

    def __init__(self, node: Node, service_name: str, label: str,
                 panel=None, parent=None):
        super().__init__(label, parent)
        self._node = node
        self._service_name = service_name
        self._client = node.create_client(SetBool, service_name)
        self._panel = panel
        self.setStyleSheet(
            'QPushButton { padding: 6px 12px; } '
            'QPushButton:pressed { background-color: #FFA726; color: black; font-weight: bold; }')
        self.pressed.connect(lambda: self._send(True))
        self.released.connect(lambda: self._send(False))

    def _send(self, val: bool):
        # Enforce robot_ready prerequisite on press
        if (val
                and self._service_name in REQUIRES_ROBOT_READY
                and self._panel is not None
                and not self._panel.is_robot_ready()):
            self._node.get_logger().warn(
                f'{self._service_name}: robot_ready is not set — '
                'auto-enabling robot_ready first')
            self._panel.ensure_robot_ready()

        req = SetBool.Request()
        req.data = val
        if self._client.service_is_ready():
            self._client.call_async(req)
