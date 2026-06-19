"""
Manual Control tab — wire jog, gas, error quit, teach mode, welding simulation.
"""

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QDoubleSpinBox, QComboBox, QGroupBox, QTextEdit,
)

from ..components.widgets import (
    ToggleServiceButton, MomentaryServiceButton, WORKING_MODES,
)


def build_manual_tab(panel) -> QWidget:
    """Build the Manual Control tab and register widgets on *panel*.

    Expected panel attributes:
        panel._node, panel._toggle_buttons, panel._robot_ready_btn,
        panel._robot_ready_client, panel._robot_ready
    """
    tab = QWidget()
    outer = QVBoxLayout(tab)

    # ---- Wire control ----
    wire_group = QGroupBox('Wire Control')
    wire_layout = QHBoxLayout(wire_group)
    wire_fwd = MomentaryServiceButton(
        panel._node, '/wago/in/wire_forward', '⬆ Wire Forward',
        panel=panel)
    wire_bwd = MomentaryServiceButton(
        panel._node, '/wago/in/wire_backward', '⬇ Wire Backward',
        panel=panel)
    wire_layout.addWidget(wire_fwd)
    wire_layout.addWidget(wire_bwd)
    outer.addWidget(wire_group)

    # ---- Boolean toggles (gas, teach, error quit, etc.) ----
    toggle_group = QGroupBox('PLC Signals')
    toggle_layout = QGridLayout(toggle_group)

    toggle_services = [
        ('/wago/in/robot_ready',         'Robot Ready'),
        ('/wago/in/gas_on',             'Gas'),
        ('/wago/in/error_quit',          'Error Quit'),
        ('/wago/in/teach_mode',          'Teach Mode'),
        ('/wago/in/torch_blow_out',      'Torch Blow-Out'),
        ('/wago/in/welding_simulation',  'Welding Simulation'),
        ('/wago/in/touch_sensing',       'Touch Sensing'),
    ]
    for i, (srv, label) in enumerate(toggle_services):
        row, col = divmod(i, 3)
        btn = ToggleServiceButton(panel._node, srv, label, panel=panel)
        toggle_layout.addWidget(btn, row, col)
        panel._toggle_buttons[srv] = btn
        if srv == '/wago/in/robot_ready':
            panel._robot_ready_btn = btn
    outer.addWidget(toggle_group)

    # ---- Working Mode ----
    mode_group = QGroupBox('Working Mode')
    mode_layout = QHBoxLayout(mode_group)
    panel._mode_combo = QComboBox()
    for val, name in sorted(WORKING_MODES.items()):
        panel._mode_combo.addItem(f'{val} — {name}', val)
    mode_layout.addWidget(panel._mode_combo)
    mode_set_btn = QPushButton('Set Working Mode')
    mode_set_btn.clicked.connect(lambda: _set_working_mode(panel))
    mode_layout.addWidget(mode_set_btn)
    outer.addWidget(mode_group)

    # ---- Quick parameter set (welding speed, wire move length) ----
    param_group = QGroupBox('Analog Commands')
    param_layout = QGridLayout(param_group)

    param_layout.addWidget(QLabel('Welding Speed:'), 0, 0)
    panel._weld_speed_spin = QDoubleSpinBox()
    panel._weld_speed_spin.setRange(0.0, 100.0)
    panel._weld_speed_spin.setDecimals(1)
    panel._weld_speed_spin.setSuffix(' mm/s')
    param_layout.addWidget(panel._weld_speed_spin, 0, 1)
    ws_btn = QPushButton('Set')
    ws_btn.clicked.connect(lambda: _set_float_service(
        panel, '/wago/in/welding_speed', panel._weld_speed_spin.value()))
    param_layout.addWidget(ws_btn, 0, 2)

    param_layout.addWidget(QLabel('Wire Move Length:'), 1, 0)
    panel._wire_len_spin = QDoubleSpinBox()
    panel._wire_len_spin.setRange(0.0, 500.0)
    panel._wire_len_spin.setDecimals(1)
    panel._wire_len_spin.setSuffix(' mm')
    param_layout.addWidget(panel._wire_len_spin, 1, 1)
    wl_btn = QPushButton('Set')
    wl_btn.clicked.connect(lambda: _set_float_service(
        panel, '/wago/in/wire_move_length', panel._wire_len_spin.value()))
    param_layout.addWidget(wl_btn, 1, 2)

    outer.addWidget(param_group)

    # ---- Log output ----
    log_group = QGroupBox('Service Log')
    log_layout = QVBoxLayout(log_group)
    panel._log_text = QTextEdit()
    panel._log_text.setReadOnly(True)
    panel._log_text.setMaximumHeight(120)
    panel._log_text.setStyleSheet('font-family: monospace; font-size: 11px;')
    log_layout.addWidget(panel._log_text)
    outer.addWidget(log_group)

    outer.addStretch()
    return tab


# ---------------------------------------------------------------------------
# Helper functions (called with panel as first arg)
# ---------------------------------------------------------------------------

def _set_working_mode(panel):
    mode_val = panel._mode_combo.currentData()
    started = panel._manual_ctrl.set_working_mode(
        mode_val,
        lambda f: log(panel, f'Working mode → {mode_val}: '
                              f'{f.result().message if f.result() else "timeout"}'),
    )
    if not started:
        log(panel, 'Working mode service not available')


def _set_float_service(panel, service_name: str, value: float):
    started = panel._manual_ctrl.set_float(
        service_name,
        value,
        lambda f: log(panel, f'{service_name} → {value}: '
                              f'{f.result().message if f.result() else "timeout"}'),
    )
    if not started:
        log(panel, f'{service_name} not available')


def log(panel, text: str):
    """Append text to the manual-tab service log."""
    if panel._log_text:
        panel._log_text.append(text)


def is_robot_ready(panel) -> bool:
    """Return True if robot_ready is currently set."""
    return panel._robot_ready


def ensure_robot_ready(panel):
    """Set robot_ready=True via the WAGO service."""
    started = panel._manual_ctrl.set_robot_ready(True, lambda _: None)
    if started:
        panel.state.robot_ready = True
        panel._robot_ready = True
        if panel._robot_ready_btn is not None:
            panel._robot_ready_btn.set_from_readback(True)
        log(panel, 'Auto-enabled robot_ready (prerequisite for PLC signal)')
    else:
        log(panel, 'WARNING: /wago/in/robot_ready service not available')
