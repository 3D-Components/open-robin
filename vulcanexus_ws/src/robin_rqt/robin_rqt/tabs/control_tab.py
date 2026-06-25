"""
Control tab — merged Status + Manual Control + Laser ON/OFF.

Compact layout for a quarter-screen touch display (~960×540).
"""

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QDoubleSpinBox, QComboBox, QGroupBox, QTextEdit, QSizePolicy,
)

from std_srvs.srv import Trigger
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State

from ..components.widgets import (
    StatusIndicator,
    ToggleServiceButton, MomentaryServiceButton,
    WAGO_BOOL_TOPICS,
    WORKING_MODES, apply_touch_style,
    TOUCH_MIN_H,
)


# Float items for PLC status grid (short display labels)
_FLOAT_GRID_ITEMS = [
    ('/wago/out/welding_current',  'Current', 'A'),
    ('/wago/out/welding_voltage',  'Voltage', 'V'),
    ('/wago/out/wire_feed_speed',  'WFS', 'm/min'),
]


class _FloatCell(QLabel):
    """Compact float readout for the PLC status grid."""
    _CSS = (
        'QLabel { background-color: #424242; border: 1px solid #555;'
        ' border-radius: 3px; padding: 1px 4px; font-weight: bold;'
        ' font-size: 10px; min-width: 50px; }'
    )

    def __init__(self, name, unit=''):
        super().__init__(f'{name}: ---')
        self._name = name
        self._unit = unit
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(self._CSS)

    def set_value(self, v):
        self.setText(f'{self._name}: {v:.1f} {self._unit}')


def build_control_tab(panel) -> QWidget:
    """Build the unified Control tab and register widgets on *panel*."""
    tab = QWidget()
    outer = QVBoxLayout(tab)
    outer.setSpacing(4)
    outer.setContentsMargins(4, 4, 4, 4)

    # ── Top row: PLC Status (left) + Wire/Laser (right) ──────────────
    top_row = QHBoxLayout()
    top_row.setSpacing(4)

    # -- PLC Status: 3-col × 4-row grid (9 bools + 3 floats, no Warning) --
    status_group = QGroupBox('PLC Status')
    status_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
    status_grid = QGridLayout(status_group)
    status_grid.setSpacing(2)
    status_grid.setContentsMargins(4, 4, 4, 4)

    bool_items = [(t, l) for t, l in WAGO_BOOL_TOPICS
                  if t != '/wago/out/warning']
    for i, (topic, label) in enumerate(bool_items):
        row, col = divmod(i, 3)
        ind = StatusIndicator(label)
        ind.setToolTip(topic)
        panel._bool_indicators[topic] = ind
        status_grid.addWidget(ind, row, col)

    for i_f, (topic, short, unit) in enumerate(_FLOAT_GRID_ITEMS):
        cell = _FloatCell(short, unit)
        cell.setToolTip(topic)
        panel._float_readouts[topic] = cell
        status_grid.addWidget(cell, 3, i_f)

    top_row.addWidget(status_group, stretch=2)

    # -- Wire + Laser stacked on right side --
    right_col = QVBoxLayout()
    right_col.setSpacing(4)

    wire_group = QGroupBox('Wire')
    wire_layout = QHBoxLayout(wire_group)
    wire_layout.setContentsMargins(4, 4, 4, 4)
    wire_fwd = MomentaryServiceButton(
        panel._node, '/wago/in/wire_forward', '⬆ Wire Fwd', panel=panel)
    wire_bwd = MomentaryServiceButton(
        panel._node, '/wago/in/wire_backward', '⬇ Wire Bwd', panel=panel)
    wire_layout.addWidget(wire_fwd)
    wire_layout.addWidget(wire_bwd)
    right_col.addWidget(wire_group)

    laser_group = QGroupBox('Laser Profilometer')
    laser_layout = QHBoxLayout(laser_group)
    laser_layout.setContentsMargins(4, 4, 4, 4)
    panel._sensor_on_btn = QPushButton('Laser ON')
    panel._sensor_on_btn.setStyleSheet(
        f'QPushButton {{ min-height: {TOUCH_MIN_H}px; font-weight: bold; }}')
    panel._sensor_on_btn.clicked.connect(
        lambda: _set_sensor_active(panel, True))
    laser_layout.addWidget(panel._sensor_on_btn)
    panel._sensor_off_btn = QPushButton('Laser OFF')
    panel._sensor_off_btn.setStyleSheet(
        f'QPushButton {{ min-height: {TOUCH_MIN_H}px; font-weight: bold; }}')
    panel._sensor_off_btn.clicked.connect(
        lambda: _set_sensor_active(panel, False))
    laser_layout.addWidget(panel._sensor_off_btn)
    panel._sensor_state_label = QLabel('Laser: —')
    panel._sensor_state_label.setStyleSheet('font-weight: bold;')
    laser_layout.addWidget(panel._sensor_state_label)
    laser_layout.addStretch()
    right_col.addWidget(laser_group)

    right_col.addStretch()
    top_row.addLayout(right_col, stretch=1)

    outer.addLayout(top_row)

    # ── PLC Signals + Parameters side by side ─────────────────────────
    mid_row = QHBoxLayout()
    mid_row.setSpacing(4)

    # -- PLC toggle signals (2-column grid) --
    toggle_group = QGroupBox('PLC Signals')
    toggle_layout = QGridLayout(toggle_group)
    toggle_layout.setSpacing(4)
    toggle_layout.setContentsMargins(4, 4, 4, 4)

    toggle_services = [
        ('/wago/in/robot_ready',         'Robot Ready'),
        ('/wago/in/gas_on',              'Gas'),
        ('/wago/in/error_quit',          'Error Quit'),
        ('/wago/in/teach_mode',          'Teach Mode'),
        ('/wago/in/welding_simulation',  'Weld Sim'),
        ('/wago/in/touch_sensing',       'Touch Sensing'),
    ]
    for i, (srv, label) in enumerate(toggle_services):
        row, col = divmod(i, 2)
        btn = ToggleServiceButton(panel._node, srv, label, panel=panel)
        toggle_layout.addWidget(btn, row, col)
        panel._toggle_buttons[srv] = btn
        if srv == '/wago/in/robot_ready':
            panel._robot_ready_btn = btn
    mid_row.addWidget(toggle_group)

    # -- Parameters --
    param_group = QGroupBox('Parameters')
    param_grid = QGridLayout(param_group)
    param_grid.setSpacing(2)
    param_grid.setContentsMargins(4, 4, 4, 4)

    param_grid.addWidget(QLabel('Mode:'), 0, 0)
    panel._mode_combo = QComboBox()
    for val, name in sorted(WORKING_MODES.items()):
        panel._mode_combo.addItem(f'{val} — {name}', val)
    param_grid.addWidget(panel._mode_combo, 0, 1)
    mode_set_btn = QPushButton('Set')
    mode_set_btn.clicked.connect(lambda: _set_working_mode(panel))
    param_grid.addWidget(mode_set_btn, 0, 2)

    param_grid.addWidget(QLabel('Weld Speed:'), 1, 0)
    panel._weld_speed_spin = QDoubleSpinBox()
    panel._weld_speed_spin.setRange(0.0, 100.0)
    panel._weld_speed_spin.setDecimals(1)
    panel._weld_speed_spin.setSuffix(' mm/s')
    param_grid.addWidget(panel._weld_speed_spin, 1, 1)
    ws_btn = QPushButton('Set')
    ws_btn.clicked.connect(lambda: _set_float_service(
        panel, '/wago/in/welding_speed', panel._weld_speed_spin.value()))
    param_grid.addWidget(ws_btn, 1, 2)

    param_grid.addWidget(QLabel('Wire Len:'), 2, 0)
    panel._wire_len_spin = QDoubleSpinBox()
    panel._wire_len_spin.setRange(0.0, 500.0)
    panel._wire_len_spin.setDecimals(1)
    panel._wire_len_spin.setSuffix(' mm')
    param_grid.addWidget(panel._wire_len_spin, 2, 1)
    wl_btn = QPushButton('Set')
    wl_btn.clicked.connect(lambda: _set_float_service(
        panel, '/wago/in/wire_move_length', panel._wire_len_spin.value()))
    param_grid.addWidget(wl_btn, 2, 2)

    mid_row.addWidget(param_group)

    outer.addLayout(mid_row)

    # ── Service log ───────────────────────────────────────────────────
    log_group = QGroupBox('Log')
    log_layout = QVBoxLayout(log_group)
    log_layout.setContentsMargins(4, 4, 4, 4)
    panel._log_text = QTextEdit()
    panel._log_text.setReadOnly(True)
    log_layout.addWidget(panel._log_text)
    outer.addWidget(log_group, stretch=1)

    apply_touch_style(tab)

    return tab


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _set_working_mode(panel):
    mode_val = panel._mode_combo.currentData()
    def _on_mode_done(f):
        r = f.result()
        _log(panel, f'Working mode → {mode_val}: {r.message if r else "timeout"}')
    started = panel._manual_ctrl.set_working_mode(mode_val, _on_mode_done)
    if not started:
        _log(panel, 'Working mode service not available')


def _set_float_service(panel, service_name: str, value: float):
    def _on_float_done(f):
        r = f.result()
        _log(panel, f'{service_name} → {value}: {r.message if r else "timeout"}')
    started = panel._manual_ctrl.set_float(service_name, value, _on_float_done)
    if not started:
        _log(panel, f'{service_name} not available')


def _set_sensor_active(panel, enable: bool):
    action = 'ON' if enable else 'OFF'

    if not panel._sensor_change_state.service_is_ready():
        _log(panel, 'Sensor lifecycle service not available')
        return

    if enable:
        # Get current state, then send needed transitions
        future = panel._sensor_get_state.call_async(GetState.Request())

        def _got_state(fut):
            result = fut.result()
            if result is None:
                _log(panel, 'Failed to get sensor state')
                return
            sid = result.current_state.id
            transitions = []
            if sid == State.PRIMARY_STATE_UNCONFIGURED:
                transitions = [Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]
            elif sid == State.PRIMARY_STATE_INACTIVE:
                transitions = [Transition.TRANSITION_ACTIVATE]
            elif sid == State.PRIMARY_STATE_ACTIVE:
                _log(panel, 'Sensor already active')
                panel._sensor_state_label.setText("Laser: ON")
                return
            _send_transitions(panel, transitions, action)

        future.add_done_callback(_got_state)
    else:
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        future = panel._sensor_change_state.call_async(req)

        def _done(fut):
            result = fut.result()
            if result and result.success:
                panel._sensor_state_label.setText("Laser: OFF")
                _log(panel, 'Laser OFF')
            else:
                _log(panel, 'Laser OFF failed')

        future.add_done_callback(_done)


def _send_transitions(panel, transitions, action):
    """Send a sequence of lifecycle transitions."""
    if not transitions:
        panel._sensor_state_label.setText(f"Laser: {action}")
        _log(panel, f'Laser {action}')
        return

    tid = transitions[0]
    remaining = transitions[1:]
    req = ChangeState.Request()
    req.transition.id = tid

    future = panel._sensor_change_state.call_async(req)

    def _done(fut):
        result = fut.result()
        if result and result.success:
            _send_transitions(panel, remaining, action)
        else:
            _log(panel, f'Laser {action} failed at transition {tid}')

    future.add_done_callback(_done)


def _log(panel, text: str):
    if panel._log_text:
        from PyQt5.QtCore import QMetaObject, Qt, Q_ARG
        QMetaObject.invokeMethod(
            panel._log_text, "append",
            Qt.QueuedConnection,
            Q_ARG(str, str(text)),
        )


def is_robot_ready(panel) -> bool:
    return panel.state.robot_ready


def ensure_robot_ready(panel):
    started = panel._manual_ctrl.set_robot_ready(True, lambda _: None)
    if started:
        panel.state.robot_ready = True
        panel.state.robot_ready = True
        if panel._robot_ready_btn is not None:
            panel._robot_ready_btn.set_from_readback(True)
        _log(panel, 'Auto-enabled robot_ready')
    else:
        _log(panel, 'WARNING: /wago/in/robot_ready service not available')
