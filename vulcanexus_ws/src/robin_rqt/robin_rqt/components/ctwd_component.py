"""
CTWD/TCP component — live TCP display and TCP mode switching.

The TCP mode buttons have been removed from the UI; switching is done
programmatically by the experiment workflow via ``switch_tcp_mode()``.
"""

import math

import rclpy
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QLabel, QWidget, QGridLayout,
)
from tf_transformations import euler_from_quaternion


def build_live_tcp_widget(panel) -> QWidget:
    """Build a compact Live TCP readout widget with RGB-coloured axes."""
    panel._tcp_pos_labels = {}
    tcp_pos_widget = QWidget()
    tcp_pos_grid = QGridLayout(tcp_pos_widget)
    tcp_pos_grid.setContentsMargins(4, 2, 4, 2)
    tcp_pos_grid.setHorizontalSpacing(2)
    tcp_pos_grid.setVerticalSpacing(1)
    tcp_pos_grid.setColumnStretch(1, 1)
    tcp_pos_grid.setColumnStretch(3, 1)
    tcp_pos_grid.setColumnStretch(5, 1)

    _val_css = 'font-family: monospace; font-size: 10px; padding: 0 2px;'

    # RGB colour convention for XYZ / RxRyRz
    _axis_colors = {
        'X': '#E53935', 'Y': '#43A047', 'Z': '#1E88E5',
        'Rx': '#E53935', 'Ry': '#43A047', 'Rz': '#1E88E5',
    }

    for i, axis in enumerate(['X', 'Y', 'Z']):
        color = _axis_colors[axis]
        lbl = QLabel(f'{axis}:')
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lbl.setStyleSheet(
            f'font-weight: bold; font-size: 10px; color: {color}; padding: 0 1px;')
        val = QLabel('---.- mm')
        val.setStyleSheet(_val_css)
        tcp_pos_grid.addWidget(lbl, 0, i * 2)
        tcp_pos_grid.addWidget(val, 0, i * 2 + 1)
        panel._tcp_pos_labels[axis] = val
    for i, axis in enumerate(['Rx', 'Ry', 'Rz']):
        color = _axis_colors[axis]
        lbl = QLabel(f'{axis}:')
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lbl.setStyleSheet(
            f'font-weight: bold; font-size: 10px; color: {color}; padding: 0 1px;')
        val = QLabel('---.- °')
        val.setStyleSheet(_val_css)
        tcp_pos_grid.addWidget(lbl, 1, i * 2)
        tcp_pos_grid.addWidget(val, 1, i * 2 + 1)
        panel._tcp_pos_labels[axis] = val

    panel._tcp_pos_timer = QTimer()
    panel._tcp_pos_timer.timeout.connect(lambda: _update_live_tcp(panel))
    panel._tcp_pos_timer.start(200)

    return tcp_pos_widget


# ---------------------------------------------------------------------------
# Public helpers (called from operator_panel / experiment workflow)
# ---------------------------------------------------------------------------

def update_ctwd_calibration_status(panel, value: bool):
    panel.state.calibration.tcp_wire_tip_calibrated = value
    if hasattr(panel, '_ctwd_cal_label'):
        if value:
            panel._ctwd_cal_label.setText('✓ Calibrated')
            panel._ctwd_cal_label.setStyleSheet(
                'font-size: 12px; font-weight: bold; padding: 2px; '
                'color: #4CAF50;')
        else:
            panel._ctwd_cal_label.setText('✗ Not Calibrated')
            panel._ctwd_cal_label.setStyleSheet(
                'font-size: 12px; font-weight: bold; padding: 2px; '
                'color: #F44336;')


def update_ctwd_value_from_topic(panel, value: float):
    panel.state.calibration.tcp_wire_tip = value
    if hasattr(panel, '_ctwd_readout'):
        panel._ctwd_readout.setText(f'{value * 1000:.2f} mm')

    if not hasattr(panel, '_ctwd_spin'):
        return

    if panel.state.calibration.wire_tip_user_editing:
        return
    if panel._ctwd_spin.hasFocus():
        return

    panel._ctwd_spin.blockSignals(True)
    panel._ctwd_spin.setValue(value * 1000.0)
    panel._ctwd_spin.blockSignals(False)


def update_active_tcp_frame(panel, value: str):
    panel.state.calibration.tcp_active_frame = value


def switch_tcp_mode(panel, mode: str):
    """Programmatically switch TCP mode (used by experiment workflow)."""
    _switch_tcp_mode(panel, mode)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _set_ctwd_editing(panel, editing: bool):
    panel.state.calibration.wire_tip_user_editing = editing
    panel._ctwd_user_editing = editing


def _set_ctwd_from_spin(panel):
    """Call /tcp/set_ctwd with the spinbox value (mm → m)."""
    _set_ctwd_editing(panel, False)
    value_m = panel._ctwd_spin.value() / 1000.0
    panel._cal_log.append(f'Setting wire tip to {value_m * 1000:.2f} mm...')

    def on_done(f):
        panel._cal_log.append(
            f'Wire tip set: {f.result().message}' if f.result()
            else 'Wire tip set: timeout')

    panel._calibration_ctrl.set_wire_tip(value_m, on_done)


def _update_live_tcp(panel):
    """Look up the active TCP frame in base_link and update the display."""
    target_frame = panel.state.calibration.tcp_active_frame or 'wire_tip'
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

    def on_result(future, mode=mode):
        res = future.result()
        if res and res.success:
            panel._cal_log.append(f'TCP mode → {mode} ({res.active_frame})')
        else:
            msg = res.message if res else 'timeout'
            panel._cal_log.append(f'TCP mode switch FAILED: {msg}')

    panel._calibration_ctrl.set_tcp_mode(mode, on_result)
