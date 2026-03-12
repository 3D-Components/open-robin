"""
Stickout calibration component for the calibration UI.
"""

from python_qt_binding.QtWidgets import (
    QGroupBox, QGridLayout, QLabel, QPushButton, QDoubleSpinBox, QComboBox,
)

import rclpy
from robin_interfaces.srv import CalibrateStickout

from ..tabs.plates_tab import auto_save_plates
from .jog_component import disable_servo


def build_stickout_calibration_group(panel) -> QGroupBox:
    sc_group = QGroupBox('Stickout Calibration (Guided)')
    sc_layout = QGridLayout(sc_group)

    sc_layout.addWidget(QLabel('Auto-retract length:'), 0, 0)
    panel._sc_retract = QDoubleSpinBox(); panel._sc_retract.setRange(5.0, 80.0)
    panel._sc_retract.setDecimals(1); panel._sc_retract.setValue(30.0)
    panel._sc_retract.setSuffix(' mm')
    panel._sc_retract.setToolTip('Automatically applied when calibration starts')
    sc_layout.addWidget(panel._sc_retract, 0, 1, 1, 3)

    sc_layout.addWidget(QLabel('Known contact point:'), 1, 0)
    panel._sc_contact_point_label = QLabel('Not saved')
    panel._sc_contact_point_label.setStyleSheet('font-family: monospace; font-size: 11px;')
    sc_layout.addWidget(panel._sc_contact_point_label, 1, 1, 1, 2)
    save_contact_btn = QPushButton('Save Current contact_tip Pose')
    save_contact_btn.clicked.connect(lambda: _save_current_contact_point(panel))
    sc_layout.addWidget(save_contact_btn, 1, 3)

    sc_layout.addWidget(QLabel('Calibration source:'), 2, 0)
    panel._sc_source_combo = QComboBox()
    sc_layout.addWidget(panel._sc_source_combo, 2, 1, 1, 2)
    refresh_sources_btn = QPushButton('Refresh Sources')
    refresh_sources_btn.clicked.connect(lambda: _refresh_stickout_sources(panel))
    sc_layout.addWidget(refresh_sources_btn, 2, 3)

    sc_cal_btn = QPushButton('Run Stickout Calibration')
    sc_cal_btn.setStyleSheet(
        'QPushButton { padding: 8px 16px; font-weight: bold; font-size: 13px; }')
    sc_cal_btn.clicked.connect(lambda: _calibrate_stickout(panel))
    sc_layout.addWidget(sc_cal_btn, 3, 0, 1, 3)

    sc_abort_btn = QPushButton('Abort Calibration')
    sc_abort_btn.setStyleSheet(
        'QPushButton { padding: 8px 16px; font-weight: bold; font-size: 13px; '
        'background-color: #cc3333; color: white; }')
    sc_abort_btn.clicked.connect(lambda: _abort_calibration(panel))
    sc_layout.addWidget(sc_abort_btn, 3, 3)

    panel._sc_result = QLabel('Result: —')
    panel._sc_result.setStyleSheet(
        'font-size: 14px; font-weight: bold; padding: 4px;')
    sc_layout.addWidget(panel._sc_result, 4, 0, 1, 4)

    panel._refresh_stickout_sources = lambda: _refresh_stickout_sources(panel)
    panel._update_contact_point_label = lambda: _update_contact_point_label(panel)
    _refresh_stickout_sources(panel)
    _update_contact_point_label(panel)

    return sc_group


def _save_current_contact_point(panel):
    try:
        tf = panel._tf_buffer.lookup_transform(
            'base_link', 'contact_tip', rclpy.time.Time())
        t = tf.transform.translation
        panel._calibration_point = {
            'x': float(t.x),
            'y': float(t.y),
            'z': float(t.z),
            'frame': 'base_link',
            'source': 'contact_tip',
        }
        auto_save_plates(panel)
        _update_contact_point_label(panel)
        _refresh_stickout_sources(panel)
        panel._cal_log.append(
            f'Saved contact point: x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f} (base_link)')
    except Exception as e:
        panel._cal_log.append(f'ERROR: Failed to read contact_tip pose: {e}')


def _update_contact_point_label(panel):
    cp = getattr(panel, '_calibration_point', None)
    if not cp:
        panel._sc_contact_point_label.setText('Not saved')
        return
    panel._sc_contact_point_label.setText(
        f"x={cp['x']:.4f}, y={cp['y']:.4f}, z={cp['z']:.4f}")


def _refresh_stickout_sources(panel):
    panel._sc_source_combo.clear()
    cp = getattr(panel, '_calibration_point', None)
    if cp:
        panel._sc_source_combo.addItem(
            'Known contact point', ('contact_point', None))
    for p in panel._plates:
        if p.get('is_calibrated', False):
            panel._sc_source_combo.addItem(
                f"Calibrated plate: {p['plate_id']}", ('plate', p['plate_id']))


def _calibrate_stickout(panel):
    if not panel.is_robot_ready():
        panel._sc_result.setText('Result: Robot Ready is FALSE')
        panel._sc_result.setStyleSheet(
            'font-size: 14px; font-weight: bold; padding: 4px; color: #F44336;')
        panel._cal_log.append('ERROR: Refusing calibration because robot_ready is FALSE')
        return

    if getattr(panel, '_servo_active', False):
        panel._cal_log.append('Jog mode active — disabling jog before calibration')
        disable_servo(panel)

    source_data = panel._sc_source_combo.currentData()
    if source_data is None:
        panel._sc_result.setText('Result: No calibration source selected!')
        panel._cal_log.append('ERROR: No stickout calibration source available')
        return

    source_kind, source_id = source_data
    if source_kind == 'contact_point':
        cp = getattr(panel, '_calibration_point', None)
        if not cp:
            panel._sc_result.setText('Result: Contact point not set!')
            panel._cal_log.append('ERROR: Known contact point not set')
            return
        req_x, req_y, req_surface_z = cp['x'], cp['y'], cp['z']
    else:
        plate = next((p for p in panel._plates if p['plate_id'] == source_id), None)
        if plate is None:
            panel._sc_result.setText('Result: Plate source not found!')
            panel._cal_log.append(f'ERROR: Plate source not found: {source_id}')
            return
        req_x, req_y, req_surface_z = plate['origin_x'], plate['origin_y'], plate['surface_z']

    req = CalibrateStickout.Request()
    req.x = req_x
    req.y = req_y
    req.surface_z = req_surface_z
    req.nozzle_height = 0.0
    req.retract_length = panel._sc_retract.value() / 1000.0

    panel._sc_result.setText('Result: Calibrating...')
    panel._sc_result.setStyleSheet(
        'font-size: 14px; font-weight: bold; padding: 4px; color: #FFA726;')
    panel._cal_log.append(
        f'Stickout cal at ({req.x:.4f}, {req.y:.4f}), '
        f'surface_z={req.surface_z:.4f}, auto-retract={req.retract_length:.3f}m '
        f'(source={source_kind}{":" + source_id if source_id else ""})')

    started = panel._calibration_ctrl.calibrate_stickout(
        req, lambda f: _on_stickout_cal_result(panel, f))
    if not started:
        panel._sc_result.setText('Result: Service not available!')


def _on_stickout_cal_result(panel, future):
    result = future.result()
    if result is None:
        panel._sc_result.setText('Result: Service call failed!')
        panel._sc_result.setStyleSheet(
            'font-size: 14px; font-weight: bold; padding: 4px; color: #F44336;')
        panel._cal_log.append('ERROR: Stickout calibration returned None')
        return

    if result.success:
        panel._sc_result.setText(
            f'Result: Stickout = {result.measured_stickout * 1000:.2f} mm')
        panel._sc_result.setStyleSheet(
            'font-size: 14px; font-weight: bold; padding: 4px; color: #4CAF50;')
        panel._cal_log.append(f'SUCCESS: {result.message}')
    else:
        panel._sc_result.setText(f'Result: FAILED — {result.message}')
        panel._sc_result.setStyleSheet(
            'font-size: 14px; font-weight: bold; padding: 4px; color: #F44336;')
        panel._cal_log.append(f'FAILED: {result.message}')


def _abort_calibration(panel):
    """Send abort request to /calibration/abort service."""
    client = getattr(panel, '_abort_calibration_client', None)
    if client is None or not client.service_is_ready():
        panel._cal_log.append('WARN: /calibration/abort service not available')
        return
    from std_srvs.srv import Trigger
    req = Trigger.Request()
    future = client.call_async(req)
    future.add_done_callback(lambda f: _on_abort_result(panel, f))
    panel._cal_log.append('Abort calibration requested…')


def _on_abort_result(panel, future):
    try:
        result = future.result()
        if result and result.success:
            panel._sc_result.setText('Result: Calibration aborted')
            panel._sc_result.setStyleSheet(
                'font-size: 14px; font-weight: bold; padding: 4px; color: #FF9800;')
            panel._cal_log.append(f'Abort: {result.message}')
        else:
            msg = result.message if result else 'No response'
            panel._cal_log.append(f'Abort failed: {msg}')
    except Exception as e:
        panel._cal_log.append(f'Abort error: {e}')
