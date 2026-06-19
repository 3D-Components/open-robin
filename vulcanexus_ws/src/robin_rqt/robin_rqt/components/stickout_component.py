"""
Stickout/TCP component for the calibration UI.
"""

import math

import rclpy
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QLabel, QPushButton, QDoubleSpinBox, QGroupBox,
    QWidget, QGridLayout,
)
from tf_transformations import euler_from_quaternion


def build_tcp_stickout_group(panel) -> QGroupBox:
    """Build TCP mode + stickout + live TCP widget group."""
    tcp_group = QGroupBox('TCP Mode & Stickout')
    tcp_layout = QGridLayout(tcp_group)

    tcp_layout.addWidget(QLabel('TCP Mode:'), 0, 0)
    panel._tcp_welding_btn = QPushButton('Welding (wire_tip)')
    panel._tcp_welding_btn.setCheckable(True)
    panel._tcp_welding_btn.setChecked(True)
    panel._tcp_welding_btn.clicked.connect(lambda: _switch_tcp_mode(panel, 'welding'))
    tcp_layout.addWidget(panel._tcp_welding_btn, 0, 1)

    panel._tcp_scanning_btn = QPushButton('Scanning (laser_frame)')
    panel._tcp_scanning_btn.setCheckable(True)
    panel._tcp_scanning_btn.setChecked(False)
    panel._tcp_scanning_btn.clicked.connect(lambda: _switch_tcp_mode(panel, 'scanning'))
    tcp_layout.addWidget(panel._tcp_scanning_btn, 0, 2)

    panel._tcp_contact_tip_btn = QPushButton('Contact Tip (contact_tip)')
    panel._tcp_contact_tip_btn.setCheckable(True)
    panel._tcp_contact_tip_btn.setChecked(False)
    panel._tcp_contact_tip_btn.clicked.connect(lambda: _switch_tcp_mode(panel, 'contact_tip'))
    tcp_layout.addWidget(panel._tcp_contact_tip_btn, 0, 3)

    panel._tcp_frame_label = QLabel('Active TCP: wire_tip')
    panel._tcp_frame_label.setStyleSheet(
        'font-size: 13px; font-weight: bold; padding: 4px; color: #4CAF50;')
    tcp_layout.addWidget(panel._tcp_frame_label, 0, 4)

    tcp_layout.addWidget(QLabel('Stickout:'), 1, 0)
    panel._stickout_spin = QDoubleSpinBox()
    panel._stickout_spin.setRange(0.0, 50.0)
    panel._stickout_spin.setDecimals(2)
    panel._stickout_spin.setSingleStep(0.5)
    panel._stickout_spin.setValue(15.0)
    panel._stickout_spin.setSuffix(' mm')
    panel._stickout_user_editing = False
    panel.state.calibration.stickout_user_editing = False
    line_edit = panel._stickout_spin.lineEdit()
    if line_edit is not None:
        line_edit.textEdited.connect(lambda _: _set_stickout_editing(panel, True))
    panel._stickout_spin.editingFinished.connect(
        lambda: _set_stickout_editing(panel, False))
    tcp_layout.addWidget(panel._stickout_spin, 1, 1)

    stickout_set_btn = QPushButton('Set Stickout')
    stickout_set_btn.setStyleSheet('font-weight: bold; padding: 4px 10px;')
    stickout_set_btn.clicked.connect(lambda: _set_stickout_from_spin(panel))
    tcp_layout.addWidget(stickout_set_btn, 1, 2)

    panel._stickout_readout = QLabel('15.00 mm')
    panel._stickout_readout.setStyleSheet(
        'font-family: monospace; font-size: 14px; font-weight: bold; padding: 4px;')
    tcp_layout.addWidget(panel._stickout_readout, 1, 3)

    panel._stickout_cal_label = QLabel('✗ Not Calibrated')
    panel._stickout_cal_label.setStyleSheet(
        'font-size: 13px; font-weight: bold; padding: 4px; color: #F44336;')
    tcp_layout.addWidget(panel._stickout_cal_label, 1, 4)

    tcp_layout.addWidget(QLabel('Live TCP:'), 2, 0)
    panel._tcp_pos_labels = {}
    tcp_pos_widget = QWidget()
    tcp_pos_grid = QGridLayout(tcp_pos_widget)
    tcp_pos_grid.setContentsMargins(0, 0, 0, 0)
    for i, axis in enumerate(['X', 'Y', 'Z']):
        lbl = QLabel(f'{axis}:')
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lbl.setStyleSheet('font-weight: bold;')
        val = QLabel('---.- mm')
        val.setStyleSheet('font-family: monospace; font-size: 12px;')
        val.setMinimumWidth(80)
        tcp_pos_grid.addWidget(lbl, 0, i * 2)
        tcp_pos_grid.addWidget(val, 0, i * 2 + 1)
        panel._tcp_pos_labels[axis] = val
    for i, axis in enumerate(['Rx', 'Ry', 'Rz']):
        lbl = QLabel(f'{axis}:')
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lbl.setStyleSheet('font-weight: bold;')
        val = QLabel('---.- °')
        val.setStyleSheet('font-family: monospace; font-size: 12px;')
        val.setMinimumWidth(80)
        tcp_pos_grid.addWidget(lbl, 1, i * 2)
        tcp_pos_grid.addWidget(val, 1, i * 2 + 1)
        panel._tcp_pos_labels[axis] = val
    tcp_layout.addWidget(tcp_pos_widget, 2, 1, 1, 4)

    panel._tcp_pos_timer = QTimer()
    panel._tcp_pos_timer.timeout.connect(lambda: _update_live_tcp(panel))
    panel._tcp_pos_timer.start(200)

    return tcp_group


def update_stickout_calibration_status(panel, value: bool):
    panel.state.calibration.tcp_stickout_calibrated = value
    panel._tcp_stickout_calibrated = value
    if hasattr(panel, '_stickout_cal_label'):
        if value:
            panel._stickout_cal_label.setText('✓ Calibrated')
            panel._stickout_cal_label.setStyleSheet(
                'font-size: 13px; font-weight: bold; padding: 4px; '
                'color: #4CAF50;')
        else:
            panel._stickout_cal_label.setText('✗ Not Calibrated')
            panel._stickout_cal_label.setStyleSheet(
                'font-size: 13px; font-weight: bold; padding: 4px; '
                'color: #F44336;')


def update_stickout_value_from_topic(panel, value: float):
    panel.state.calibration.tcp_stickout = value
    panel._tcp_stickout = value
    if hasattr(panel, '_stickout_readout'):
        panel._stickout_readout.setText(f'{value * 1000:.2f} mm')

    if not hasattr(panel, '_stickout_spin'):
        return

    if panel.state.calibration.stickout_user_editing:
        return
    if panel._stickout_spin.hasFocus():
        return

    panel._stickout_spin.blockSignals(True)
    panel._stickout_spin.setValue(value * 1000.0)
    panel._stickout_spin.blockSignals(False)


def update_active_tcp_frame(panel, value: str):
    panel.state.calibration.tcp_active_frame = value
    panel._tcp_active_frame = value
    if hasattr(panel, '_tcp_frame_label'):
        panel._tcp_frame_label.setText(f'Active TCP: {value}')


def _set_stickout_editing(panel, editing: bool):
    panel.state.calibration.stickout_user_editing = editing
    panel._stickout_user_editing = editing


def _set_stickout_from_spin(panel):
    """Call /tcp/set_stickout with the spinbox value (mm → m)."""
    _set_stickout_editing(panel, False)
    value_m = panel._stickout_spin.value() / 1000.0
    panel._cal_log.append(f'Setting stickout to {value_m * 1000:.2f} mm...')

    def on_done(f):
        panel._cal_log.append(
            f'Stickout set: {f.result().message}' if f.result()
            else 'Stickout set: timeout')

    panel._calibration_ctrl.set_stickout(value_m, on_done)


def _update_live_tcp(panel):
    """Look up the active TCP frame in base_link and update the display."""
    target_frame = panel._tcp_active_frame or 'wire_tip'
    try:
        if not panel._tf_buffer.can_transform(
                'base_link', target_frame, rclpy.time.Time()):
            return
        trans = panel._tf_buffer.lookup_transform(
            'base_link', target_frame,
            rclpy.time.Time())
        t = trans.transform.translation
        q = trans.transform.rotation
        panel._tcp_pos_labels['X'].setText(f'{t.x * 1000:.1f} mm')
        panel._tcp_pos_labels['Y'].setText(f'{t.y * 1000:.1f} mm')
        panel._tcp_pos_labels['Z'].setText(f'{t.z * 1000:.1f} mm')
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        panel._tcp_pos_labels['Rx'].setText(f'{math.degrees(roll):.1f} °')
        panel._tcp_pos_labels['Ry'].setText(f'{math.degrees(pitch):.1f} °')
        panel._tcp_pos_labels['Rz'].setText(f'{math.degrees(yaw):.1f} °')
    except Exception:
        pass


def _switch_tcp_mode(panel, mode: str):
    panel._cal_log.append(f'Switching TCP mode to: {mode}...')

    mode_colors = {
        'welding': '#4CAF50',
        'scanning': '#2196F3',
        'contact_tip': '#FF9800',
    }

    def on_result(future, mode=mode):
        res = future.result()
        if res and res.success:
            panel._cal_log.append(f'TCP mode → {mode} ({res.active_frame})')
            panel._tcp_welding_btn.setChecked(mode == 'welding')
            panel._tcp_scanning_btn.setChecked(mode == 'scanning')
            panel._tcp_contact_tip_btn.setChecked(mode == 'contact_tip')
            panel._tcp_frame_label.setText(f'Active TCP: {res.active_frame}')
            color = mode_colors.get(mode, '#4CAF50')
            panel._tcp_frame_label.setStyleSheet(
                f'font-size: 13px; font-weight: bold; padding: 4px; color: {color};')
        else:
            msg = res.message if res else 'timeout'
            panel._cal_log.append(f'TCP mode switch FAILED: {msg}')

    panel._calibration_ctrl.set_tcp_mode(mode, on_result)
