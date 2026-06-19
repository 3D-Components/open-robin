"""
Cartesian jog component for the calibration UI.
"""

from functools import partial

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QWidget, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QSlider, QRadioButton, QButtonGroup,
)

from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
from moveit_msgs.srv import ServoCommandType
from controller_manager_msgs.srv import SwitchController


_JOG_TICK_MS = 10


def build_jog_group(panel) -> QGroupBox:
    """Build Cartesian jog widget group and register handlers on panel."""
    jog_group = QGroupBox('Cartesian Jog (MoveIt Servo)')
    jog_layout = QGridLayout(jog_group)

    panel._jog_toggle_btn = QPushButton('Enable Jog')
    panel._jog_toggle_btn.setCheckable(True)
    panel._jog_toggle_btn.setChecked(False)
    panel._jog_toggle_btn.setStyleSheet(
        'QPushButton { padding: 8px 16px; font-weight: bold; font-size: 13px; }'
        'QPushButton:checked { background-color: #FF9800; color: black; }')
    panel._jog_toggle_btn.clicked.connect(lambda: _on_jog_toggle(panel))
    jog_layout.addWidget(panel._jog_toggle_btn, 0, 0, 1, 2)

    frame_group_box = QGroupBox('Jog Frame')
    frame_lay = QHBoxLayout(frame_group_box)
    panel._jog_frame_group = QButtonGroup()
    panel._jog_base_radio = QRadioButton('Base Frame')
    panel._jog_base_radio.setChecked(True)
    panel._jog_tool_radio = QRadioButton('Tool Frame')
    panel._jog_frame_group.addButton(panel._jog_base_radio, 0)
    panel._jog_frame_group.addButton(panel._jog_tool_radio, 1)
    panel._jog_frame_group.buttonClicked.connect(lambda: _on_jog_frame_changed(panel))
    frame_lay.addWidget(panel._jog_base_radio)
    frame_lay.addWidget(panel._jog_tool_radio)
    jog_layout.addWidget(frame_group_box, 0, 2, 1, 2)

    jog_layout.addWidget(QLabel('Jog Speed:'), 1, 0)
    panel._jog_speed_slider = QSlider(Qt.Horizontal)
    panel._jog_speed_slider.setRange(2, 50)
    panel._jog_speed_slider.setValue(10)
    panel._jog_speed_slider.setTickInterval(5)
    panel._jog_speed_slider.setTickPosition(QSlider.TicksBelow)
    panel._jog_speed_slider.valueChanged.connect(lambda v: _on_jog_speed_changed(panel, v))
    jog_layout.addWidget(panel._jog_speed_slider, 1, 1, 1, 2)
    panel._jog_speed_label = QLabel('10.0 mm/s')
    panel._jog_speed_label.setStyleSheet('font-family: monospace; font-weight: bold;')
    jog_layout.addWidget(panel._jog_speed_label, 1, 3)

    lin_label = QLabel('Linear:')
    lin_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    jog_layout.addWidget(lin_label, 2, 0)
    jog_btn_names = [
        ('+X', (1, 0, 0, 0, 0, 0)),  ('-X', (-1, 0, 0, 0, 0, 0)),
        ('+Y', (0, 1, 0, 0, 0, 0)),  ('-Y', (0, -1, 0, 0, 0, 0)),
        ('+Z', (0, 0, 1, 0, 0, 0)),  ('-Z', (0, 0, -1, 0, 0, 0)),
    ]
    panel._jog_buttons = []
    btn_widget = QWidget()
    btn_grid = QGridLayout(btn_widget)
    btn_grid.setContentsMargins(0, 0, 0, 0)
    for i, (name, twist_dir) in enumerate(jog_btn_names):
        btn = QPushButton(name)
        btn.setEnabled(False)
        btn.setAutoRepeat(False)
        btn.setMinimumWidth(50)
        btn.setStyleSheet(
            'QPushButton { padding: 6px 10px; font-weight: bold; }'
            'QPushButton:pressed { background-color: #FFA726; color: black; }')
        btn.pressed.connect(partial(_on_jog_press, panel, twist_dir))
        btn.released.connect(lambda: _on_jog_release(panel))
        row, col = divmod(i, 6)
        btn_grid.addWidget(btn, row, col)
        panel._jog_buttons.append(btn)
    jog_layout.addWidget(btn_widget, 2, 1, 1, 3)

    rot_label = QLabel('Rotation:')
    rot_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    jog_layout.addWidget(rot_label, 3, 0)
    rot_btn_names = [
        ('+Rx', (0, 0, 0, 1, 0, 0)),  ('-Rx', (0, 0, 0, -1, 0, 0)),
        ('+Ry', (0, 0, 0, 0, 1, 0)),  ('-Ry', (0, 0, 0, 0, -1, 0)),
        ('+Rz', (0, 0, 0, 0, 0, 1)),  ('-Rz', (0, 0, 0, 0, 0, -1)),
    ]
    rot_widget = QWidget()
    rot_grid = QGridLayout(rot_widget)
    rot_grid.setContentsMargins(0, 0, 0, 0)
    for i, (name, twist_dir) in enumerate(rot_btn_names):
        btn = QPushButton(name)
        btn.setEnabled(False)
        btn.setAutoRepeat(False)
        btn.setMinimumWidth(50)
        btn.setStyleSheet(
            'QPushButton { padding: 6px 10px; font-weight: bold; }'
            'QPushButton:pressed { background-color: #FFA726; color: black; }')
        btn.pressed.connect(partial(_on_jog_press, panel, twist_dir))
        btn.released.connect(lambda: _on_jog_release(panel))
        row, col = divmod(i, 6)
        rot_grid.addWidget(btn, row, col)
        panel._jog_buttons.append(btn)
    jog_layout.addWidget(rot_widget, 3, 1, 1, 3)

    return jog_group


def disable_servo(panel):
    _disable_servo(panel)


def _on_jog_toggle(panel):
    if panel._jog_toggle_btn.isChecked():
        _enable_servo(panel)
    else:
        _disable_servo(panel)


def _enable_servo(panel):
    panel._cal_log.append('Enabling Cartesian jog (MoveIt Servo)...')

    if panel._switch_ctrl_client.service_is_ready():
        req = SwitchController.Request()
        req.activate_controllers = ['forward_velocity_controller']
        req.deactivate_controllers = ['scaled_joint_trajectory_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = panel._switch_ctrl_client.call_async(req)
        future.add_done_callback(lambda f: _on_ctrl_switch_for_servo(panel, f))
    else:
        panel._cal_log.append(
            'WARNING: controller_manager switch service not available — '
            'jog buttons enabled but controller may not accept commands')
        _set_jog_buttons_enabled(panel, True)
        panel.state.calibration.servo_active = True
        panel._servo_active = True


def _on_ctrl_switch_for_servo(panel, future):
    result = future.result()
    if result and result.ok:
        panel._cal_log.append(
            'Controller switched to forward_velocity_controller')
    else:
        panel._cal_log.append(
            'WARNING: Controller switch returned not-ok — jog may not work')

    if panel._servo_pause_client.service_is_ready():
        pause_req = SetBool.Request()
        pause_req.data = False
        panel._servo_pause_client.call_async(pause_req)

    if panel._servo_cmd_type_client.service_is_ready():
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        f2 = panel._servo_cmd_type_client.call_async(req)
        f2.add_done_callback(lambda f: _on_servo_started(panel, f))
    else:
        panel._cal_log.append(
            'WARNING: /servo_node/switch_command_type not available — '
            'launch with: launch_servo:=true')
        _set_jog_buttons_enabled(panel, True)
        panel.state.calibration.servo_active = True
        panel._servo_active = True


def _on_servo_started(panel, future):
    result = future.result()
    if result and result.success:
        panel._cal_log.append('MoveIt Servo TWIST mode active — jog enabled')
    else:
        panel._cal_log.append('Servo switch_command_type failed')
    _set_jog_buttons_enabled(panel, True)
    panel.state.calibration.servo_active = True
    panel._servo_active = True


def _disable_servo(panel):
    _set_jog_buttons_enabled(panel, False)
    panel.state.calibration.servo_active = False
    panel._servo_active = False
    panel._cal_log.append('Disabling Cartesian jog...')

    if panel._jog_ros_timer is not None:
        panel._jog_ros_timer.cancel()
    panel.state.calibration.jog_twist_dir = (0, 0, 0, 0, 0, 0)
    panel._jog_twist_dir = panel.state.calibration.jog_twist_dir

    _publish_twist(panel, 0, 0, 0, 0, 0, 0)

    if panel._servo_pause_client.service_is_ready():
        req = SetBool.Request()
        req.data = True
        panel._servo_pause_client.call_async(req)

    if panel._switch_ctrl_client.service_is_ready():
        req = SwitchController.Request()
        req.activate_controllers = ['scaled_joint_trajectory_controller']
        req.deactivate_controllers = ['forward_velocity_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = panel._switch_ctrl_client.call_async(req)
        future.add_done_callback(
            lambda f: panel._cal_log.append(
                'Controller restored to scaled_joint_trajectory_controller'
                if f.result() and f.result().ok
                else 'WARNING: Controller restore may have failed'))
    panel._jog_toggle_btn.setChecked(False)
    panel._cal_log.append('Cartesian jog disabled')


def _set_jog_buttons_enabled(panel, enabled: bool):
    for btn in panel._jog_buttons:
        btn.setEnabled(enabled)


def _on_jog_frame_changed(panel):
    if panel._jog_base_radio.isChecked():
        panel.state.calibration.jog_frame = 'base_link'
    else:
        panel.state.calibration.jog_frame = panel._tcp_active_frame or 'wire_tip'
    panel._jog_frame = panel.state.calibration.jog_frame


def _on_jog_speed_changed(panel, value: int):
    panel.state.calibration.jog_speed = value * 0.001
    panel._jog_speed = panel.state.calibration.jog_speed
    panel._jog_speed_label.setText(f'{value:.1f} mm/s')


def _on_jog_press(panel, twist_dir: tuple):
    panel.state.calibration.jog_twist_dir = twist_dir
    panel._jog_twist_dir = panel.state.calibration.jog_twist_dir
    _ensure_jog_timer(panel)
    panel._jog_ros_timer.reset()


def _on_jog_release(panel):
    panel.state.calibration.jog_twist_dir = (0, 0, 0, 0, 0, 0)
    panel._jog_twist_dir = panel.state.calibration.jog_twist_dir
    if panel._jog_ros_timer is not None:
        panel._jog_ros_timer.cancel()
    _publish_twist(panel, 0, 0, 0, 0, 0, 0)


def _ensure_jog_timer(panel):
    if panel._jog_ros_timer is None:
        panel._jog_ros_timer = panel._node.create_timer(
            _JOG_TICK_MS / 1000.0,
            lambda: _jog_timer_tick(panel),
        )
        panel._jog_ros_timer.cancel()


def _jog_timer_tick(panel):
    lx, ly, lz, ax, ay, az = panel.state.calibration.jog_twist_dir
    speed = panel.state.calibration.jog_speed
    rot_speed = speed * 5.0
    _publish_twist(panel,
                   lx * speed, ly * speed, lz * speed,
                   ax * rot_speed, ay * rot_speed, az * rot_speed)


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
