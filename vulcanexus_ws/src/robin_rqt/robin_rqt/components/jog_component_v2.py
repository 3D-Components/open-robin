"""
Cartesian jog component — virtual joystick pads for touch screens.

Translation: XY circular pad + Z vertical single-axis pad.
Orientation: RxRy circular pad + Rz horizontal single-axis pad.
Displacement is proportional: further from centre = faster jog.
Fixed speeds: 50 mm/s translation, 10 mm/s (≈0.175 rad/s) orientation.
"""

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton,
    QGroupBox, QTabWidget, QSizePolicy,
)

from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
from moveit_msgs.srv import ServoCommandType
from controller_manager_msgs.srv import SwitchController, ListControllers

from .joystick_widget import JoystickPad, SingleAxisPad


_JOG_TICK_MS = 10
_TRANS_SPEED = 0.050   # 50 mm/s in m/s
_ROT_SPEED = 0.175     # ~10 °/s in rad/s

# Acceleration limits — controls ramp-up/down time.
# 0 → full speed in ~0.33 s (translation) / ~0.35 s (rotation).
_TRANS_ACCEL = 0.15    # m/s²
_ROT_ACCEL = 0.50      # rad/s²

_ENABLE_BTN_CSS = (
    'QPushButton {'
    '  min-height: 28px; padding: 2px 8px;'
    '  font-weight: bold; font-size: 11px;'
    '}'
    'QPushButton:checked { background-color: #FF9800; color: black; }'
)


def build_jog_group(panel) -> QGroupBox:
    """Build Cartesian jog widget with virtual joystick pads."""
    jog_group = QGroupBox('Jog (MoveIt Servo)')
    main = QVBoxLayout(jog_group)
    main.setSpacing(2)
    main.setContentsMargins(4, 6, 4, 4)

    # ── Enable toggle (single button, base_link frame) ────────────────
    panel._jog_toggle_btn = QPushButton('Jog')
    panel._jog_toggle_btn.setCheckable(True)
    panel._jog_toggle_btn.setChecked(False)
    panel._jog_toggle_btn.setStyleSheet(_ENABLE_BTN_CSS)
    panel._jog_toggle_btn.clicked.connect(lambda: _on_jog_toggle(panel))
    main.addWidget(panel._jog_toggle_btn)

    # Force base_link frame
    panel.state.calibration.jog_frame = 'base_link'
    panel._jog_frame = 'base_link'

    # ── Tabbed pads: Translation (default) | Orientation ──────────────
    pad_tabs = QTabWidget()
    pad_tabs.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
    pad_tabs.setStyleSheet(
        'QTabBar::tab { min-height: 24px; font-size: 11px; '
        'font-weight: bold; padding: 2px 10px; }')

    # -- Translation tab: XY pad + Z single-axis --
    trans_page = QWidget()
    trans_lay = QHBoxLayout(trans_page)
    trans_lay.setSpacing(8)
    trans_lay.setContentsMargins(4, 4, 4, 4)

    # XY joystick
    xy_col = QVBoxLayout()
    xy_col.setSpacing(1)
    panel._jog_xy_pad = JoystickPad(size=150)
    panel._jog_xy_pad.setEnabled(False)
    panel._jog_xy_pad.positionChanged.connect(
        lambda x, y: _on_xy_changed(panel, x, y))
    xy_col.addWidget(panel._jog_xy_pad, alignment=Qt.AlignCenter)
    xy_lbl = QLabel('X  Y')
    xy_lbl.setAlignment(Qt.AlignCenter)
    xy_lbl.setStyleSheet('font-size: 9px; color: #999;')
    xy_col.addWidget(xy_lbl)
    trans_lay.addLayout(xy_col)

    trans_lay.addSpacing(24)

    # Z single-axis (vertical)
    z_col = QVBoxLayout()
    z_col.setSpacing(1)
    panel._jog_z_pad = SingleAxisPad(Qt.Vertical, length=150, thickness=50)
    panel._jog_z_pad.setEnabled(False)
    panel._jog_z_pad.valueChanged.connect(
        lambda v: _on_z_changed(panel, v))
    z_col.addWidget(panel._jog_z_pad, alignment=Qt.AlignCenter)
    z_lbl = QLabel('Z')
    z_lbl.setAlignment(Qt.AlignCenter)
    z_lbl.setStyleSheet('font-size: 9px; color: #999;')
    z_col.addWidget(z_lbl)
    trans_lay.addLayout(z_col)

    trans_lay.addStretch()
    pad_tabs.addTab(trans_page, 'Translation')

    # -- Orientation tab: RxRy pad on top, Rz bar below --
    rot_page = QWidget()
    rot_lay = QVBoxLayout(rot_page)
    rot_lay.setSpacing(8)
    rot_lay.setContentsMargins(4, 4, 4, 4)

    # RxRy joystick (centred)
    rxry_col = QVBoxLayout()
    rxry_col.setSpacing(1)
    panel._jog_rxry_pad = JoystickPad(size=150)
    panel._jog_rxry_pad.setEnabled(False)
    panel._jog_rxry_pad.positionChanged.connect(
        lambda x, y: _on_rxry_changed(panel, x, y))
    rxry_col.addWidget(panel._jog_rxry_pad, alignment=Qt.AlignCenter)
    rxry_lbl = QLabel('Rx  Ry')
    rxry_lbl.setAlignment(Qt.AlignCenter)
    rxry_lbl.setStyleSheet('font-size: 9px; color: #999;')
    rxry_col.addWidget(rxry_lbl)
    rot_lay.addLayout(rxry_col)

    # Rz single-axis (horizontal bar below RxRy)
    rz_col = QVBoxLayout()
    rz_col.setSpacing(1)
    panel._jog_rz_pad = SingleAxisPad(Qt.Horizontal, length=150, thickness=40)
    panel._jog_rz_pad.setEnabled(False)
    panel._jog_rz_pad.valueChanged.connect(
        lambda v: _on_rz_changed(panel, v))
    rz_col.addWidget(panel._jog_rz_pad, alignment=Qt.AlignCenter)
    rz_lbl = QLabel('Rz')
    rz_lbl.setAlignment(Qt.AlignCenter)
    rz_lbl.setStyleSheet('font-size: 9px; color: #999;')
    rz_col.addWidget(rz_lbl)
    rot_lay.addLayout(rz_col)

    pad_tabs.addTab(rot_page, 'Orientation')

    main.addWidget(pad_tabs)

    # Store current joystick output (proportional, normalised)
    panel._jog_xy = (0.0, 0.0)
    panel._jog_z = 0.0
    panel._jog_rxry = (0.0, 0.0)
    panel._jog_rz = 0.0

    # Smoothed (ramped) velocity state: [lx, ly, lz, ax, ay, az]
    panel._jog_smooth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    return jog_group


# ---------------------------------------------------------------------------
# Joystick signal handlers
# ---------------------------------------------------------------------------

def _on_xy_changed(panel, x, y):
    panel._jog_xy = (x, y)
    _update_jog(panel)


def _on_z_changed(panel, v):
    panel._jog_z = v
    _update_jog(panel)


def _on_rxry_changed(panel, x, y):
    panel._jog_rxry = (x, y)
    _update_jog(panel)


def _on_rz_changed(panel, v):
    panel._jog_rz = v
    _update_jog(panel)


def _is_all_zero(panel) -> bool:
    return (panel._jog_xy == (0.0, 0.0)
            and panel._jog_z == 0.0
            and panel._jog_rxry == (0.0, 0.0)
            and panel._jog_rz == 0.0)


def _update_jog(panel):
    """Start or stop the jog timer based on current pad state."""
    smoothed_zero = all(v == 0.0 for v in panel._jog_smooth)
    if _is_all_zero(panel) and smoothed_zero:
        _destroy_jog_timer(panel)
        _publish_twist(panel, 0, 0, 0, 0, 0, 0)
    else:
        # Keep timer alive for ramp-down even after pads return to zero.
        _ensure_jog_timer(panel)


def disable_servo(panel):
    _disable_servo(panel)


# ---------------------------------------------------------------------------
# Internal helpers — servo lifecycle (unchanged logic)
# ---------------------------------------------------------------------------

def _on_jog_toggle(panel):
    if panel._jog_toggle_btn.isChecked():
        _enable_servo(panel)
    else:
        _disable_servo(panel)


def _enable_servo(panel):
    _get_log(panel).append('Enabling Cartesian jog (MoveIt Servo)...')

    if not panel._switch_ctrl_client.service_is_ready():
        _get_log(panel).append(
            'ERROR: controller_manager switch service not available')
        panel._jog_toggle_btn.setChecked(False)
        return

    req = SwitchController.Request()
    req.activate_controllers = ['forward_velocity_controller']

    traj_ctrl = _get_trajectory_controller_name(panel)
    panel._traj_ctrl_name = traj_ctrl  # stash for restore
    req.deactivate_controllers = [traj_ctrl]
    
    req.strictness = SwitchController.Request.BEST_EFFORT
    future = panel._switch_ctrl_client.call_async(req)
    future.add_done_callback(
        lambda f: _on_ctrl_switch_for_servo(panel, f))


def _on_ctrl_switch_for_servo(panel, future):
    r = future.result()
    if not (r and r.ok):
        _get_log(panel).append(
            'ERROR: Controller switch to forward_velocity_controller failed — aborting jog enable')
        panel._jog_toggle_btn.setChecked(False)
        return

    _get_log(panel).append(
        'Controller switched to forward_velocity_controller')

    # Unpause servo — MUST complete before switching command type
    if not panel._servo_pause_client.service_is_ready():
        _get_log(panel).append(
            'ERROR: /servo_node/pause_servo not available — aborting jog enable')
        panel._jog_toggle_btn.setChecked(False)
        return

    pause_req = SetBool.Request()
    pause_req.data = False
    pause_future = panel._servo_pause_client.call_async(pause_req)
    pause_future.add_done_callback(
        lambda f: _on_servo_unpaused(panel, f))


def _on_servo_unpaused(panel, future):
    r = future.result()
    if not (r and r.success):
        _get_log(panel).append(
            'ERROR: Failed to unpause servo — aborting jog enable')
        panel._jog_toggle_btn.setChecked(False)
        return

    _get_log(panel).append('Servo unpaused')

    # Now switch command type to TWIST
    if not panel._servo_cmd_type_client.service_is_ready():
        _get_log(panel).append(
            'ERROR: /servo_node/switch_command_type not available — '
            'launch with: launch_servo:=true')
        panel._jog_toggle_btn.setChecked(False)
        return

    req = ServoCommandType.Request()
    req.command_type = ServoCommandType.Request.TWIST
    f2 = panel._servo_cmd_type_client.call_async(req)
    f2.add_done_callback(lambda f: _on_servo_started(panel, f))


def _on_servo_started(panel, future):
    r = future.result()
    if not (r and r.success):
        _get_log(panel).append(
            'ERROR: Servo switch_command_type(TWIST) failed — jog NOT enabled')
        panel._jog_toggle_btn.setChecked(False)
        return

    _get_log(panel).append('MoveIt Servo TWIST mode active — jog enabled')
    _set_jog_pads_enabled(panel, True)
    panel.state.calibration.servo_active = True
    panel._servo_active = True


def _disable_servo(panel):
    _set_jog_pads_enabled(panel, False)
    panel.state.calibration.servo_active = False
    panel._servo_active = False
    _get_log(panel).append('Disabling Cartesian jog...')

    _destroy_jog_timer(panel)

    # Zero all pads and smoothed state
    panel._jog_xy = (0.0, 0.0)
    panel._jog_z = 0.0
    panel._jog_rxry = (0.0, 0.0)
    panel._jog_rz = 0.0
    panel._jog_smooth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    _publish_twist(panel, 0, 0, 0, 0, 0, 0)

    if panel._servo_pause_client.service_is_ready():
        req = SetBool.Request()
        req.data = True
        panel._servo_pause_client.call_async(req)

    if panel._switch_ctrl_client.service_is_ready():
        req = SwitchController.Request()
        traj_ctrl = getattr(panel, '_traj_ctrl_name', 'joint_trajectory_controller')
        req.activate_controllers = [traj_ctrl]
        req.deactivate_controllers = ['forward_velocity_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = panel._switch_ctrl_client.call_async(req)
        future.add_done_callback(
            lambda f: _on_ctrl_restored(panel, f))
    panel._jog_toggle_btn.setChecked(False)
    _get_log(panel).append('Cartesian jog disabled')


def _set_jog_pads_enabled(panel, enabled: bool):
    for pad in (panel._jog_xy_pad, panel._jog_z_pad,
                panel._jog_rxry_pad, panel._jog_rz_pad):
        pad.setEnabled(enabled)


def _on_ctrl_restored(panel, future):
    r = future.result()
    if r and r.ok:
        _get_log(panel).append(
                        f'Controller restored to {getattr(panel, "_traj_ctrl_name", "trajectory_controller")}')
    else:
        _get_log(panel).append(
            'WARNING: Controller restore may have failed')


def _destroy_jog_timer(panel):
    """Cancel and destroy the jog timer so it can be cleanly recreated."""
    if panel._jog_ros_timer is not None:
        panel._jog_ros_timer.cancel()
        panel._node.destroy_timer(panel._jog_ros_timer)
        panel._jog_ros_timer = None


def _ensure_jog_timer(panel):
    if panel._jog_ros_timer is None:
        panel._jog_ros_timer = panel._node.create_timer(
            _JOG_TICK_MS / 1000.0,
            lambda: _jog_timer_tick(panel),
        )


def _slew(current, target, max_step):
    """Move *current* toward *target* by at most *max_step*."""
    diff = target - current
    if abs(diff) <= max_step:
        return target
    return current + max_step * (1.0 if diff > 0 else -1.0)


def _jog_timer_tick(panel):
    """Publish ramped twist from current joystick pad state."""
    dt = _JOG_TICK_MS / 1000.0
    trans_step = _TRANS_ACCEL * dt   # max velocity change per tick
    rot_step = _ROT_ACCEL * dt

    # Target velocities from joystick pads
    targets = [
        panel._jog_xy[0] * _TRANS_SPEED,
        panel._jog_xy[1] * _TRANS_SPEED,
        panel._jog_z * _TRANS_SPEED,
        panel._jog_rxry[0] * _ROT_SPEED,
        panel._jog_rxry[1] * _ROT_SPEED,
        panel._jog_rz * _ROT_SPEED,
    ]
    steps = [trans_step, trans_step, trans_step,
             rot_step, rot_step, rot_step]

    s = panel._jog_smooth
    for i in range(6):
        s[i] = _slew(s[i], targets[i], steps[i])

    _publish_twist(panel, s[0], s[1], s[2], s[3], s[4], s[5])

    # Auto-stop timer once fully ramped down to zero
    if _is_all_zero(panel) and all(v == 0.0 for v in s):
        _destroy_jog_timer(panel)


def _publish_twist(panel, lx, ly, lz, ax, ay, az):
    msg = TwistStamped()
    msg.header.stamp = panel._node.get_clock().now().to_msg()
    msg.header.frame_id = panel.state.calibration.jog_frame
    msg.twist.linear.x = float(lx)
    msg.twist.linear.y = float(ly)
    msg.twist.linear.z = float(lz)
    msg.twist.angular.x = float(ax)
    msg.twist.angular.y = float(ay)
    msg.twist.angular.z = float(az)
    panel._jog_publisher.publish(msg)


class _ThreadSafeLog:
    """Wraps a QTextEdit so .append() is safe from any thread."""

    def __init__(self, widget):
        self._widget = widget

    def append(self, text):
        from PyQt5.QtCore import QMetaObject, Qt, Q_ARG
        QMetaObject.invokeMethod(
            self._widget, "append",
            Qt.QueuedConnection,
            Q_ARG(str, str(text)),
        )


def _get_log(panel):
    """Return a thread-safe log wrapper for the best available log widget."""
    for attr in ('_cal_log', '_log_text'):
        widget = getattr(panel, attr, None)
        if widget is not None:
            return _ThreadSafeLog(widget)

    # Fallback: a no-op object so callers can always call .append()
    class _Noop:
        def append(self, _text):
            pass
    return _Noop()

def _get_trajectory_controller_name(panel) -> str:
    """Query controller_manager to find the active/loaded trajectory controller."""
    list_client = getattr(panel, '_list_ctrl_client', None)
    if list_client is None:
        panel._list_ctrl_client = panel._node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        list_client = panel._list_ctrl_client
    if not list_client.service_is_ready():
        return 'joint_trajectory_controller'
    future = list_client.call_async(ListControllers.Request())
    # Spin briefly to get the result
    import time
    deadline = time.monotonic() + 2.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done():
        return 'joint_trajectory_controller'
    result = future.result()
    # Prefer scaled_ if it exists, otherwise plain
    names = {c.name for c in result.controller}
    if 'scaled_joint_trajectory_controller' in names:
        return 'scaled_joint_trajectory_controller'
    return 'joint_trajectory_controller'