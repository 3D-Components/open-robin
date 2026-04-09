"""Sensor tab — laser ON/OFF controls and live profile plot."""

from python_qt_binding.QtWidgets import (
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from robin_interfaces.srv import SensorCommand

from ..components.laser_profile_widget import LaserProfileWidget


def build_sensor_tab(panel) -> QWidget:
    tab = QWidget()
    outer = QVBoxLayout(tab)

    control_group = QGroupBox("Laser Control")
    control_layout = QHBoxLayout(control_group)

    control_layout.addWidget(QLabel("FPS:"))
    panel._sensor_fps_spin = QDoubleSpinBox()
    panel._sensor_fps_spin.setRange(1.0, 200.0)
    panel._sensor_fps_spin.setDecimals(0)
    panel._sensor_fps_spin.setValue(42.0)
    panel._sensor_fps_spin.setSuffix(" Hz")
    control_layout.addWidget(panel._sensor_fps_spin)

    panel._sensor_on_btn = QPushButton("Laser ON")
    panel._sensor_on_btn.clicked.connect(lambda: _set_sensor_active(panel, True))
    control_layout.addWidget(panel._sensor_on_btn)

    panel._sensor_off_btn = QPushButton("Laser OFF")
    panel._sensor_off_btn.clicked.connect(lambda: _set_sensor_active(panel, False))
    control_layout.addWidget(panel._sensor_off_btn)

    panel._sensor_state_label = QLabel("State: unknown")
    control_layout.addWidget(panel._sensor_state_label)
    control_layout.addStretch()
    outer.addWidget(control_group)

    plot_group = QGroupBox("Live Laser Cross-Section")
    plot_layout = QVBoxLayout(plot_group)

    controls_row = QHBoxLayout()
    controls_row.addWidget(QLabel("Plane: YZ (fixed)"))

    panel._sensor_freeze_btn = QPushButton("Freeze")
    panel._sensor_freeze_btn.setCheckable(True)
    controls_row.addWidget(panel._sensor_freeze_btn)

    clear_btn = QPushButton("Clear")
    clear_btn.clicked.connect(lambda: panel._sensor_profile_plot.clear())
    controls_row.addWidget(clear_btn)

    capture_ref_btn = QPushButton("Capture Ref")
    capture_ref_btn.setToolTip("Snapshot the current profile as a ghost reference overlay (amber)")
    capture_ref_btn.clicked.connect(lambda: panel._sensor_profile_plot.set_reference())
    controls_row.addWidget(capture_ref_btn)

    clear_ref_btn = QPushButton("Clear Ref")
    clear_ref_btn.clicked.connect(lambda: panel._sensor_profile_plot.clear_reference())
    controls_row.addWidget(clear_ref_btn)

    reload_cfg_btn = QPushButton("Reload Config")
    reload_cfg_btn.setToolTip(
        "Re-read share/robin_rqt/config/laser_profile.yaml and update axis ranges "
        "without restarting rqt"
    )
    reload_cfg_btn.clicked.connect(lambda: _reload_plot_config(panel))
    controls_row.addWidget(reload_cfg_btn)

    panel._sensor_points_label = QLabel("Points: 0")
    panel._sensor_last_point_count = -1  # cache to avoid redundant setText
    controls_row.addWidget(panel._sensor_points_label)
    controls_row.addStretch()
    plot_layout.addLayout(controls_row)

    panel._sensor_profile_plot = LaserProfileWidget()
    plot_layout.addWidget(panel._sensor_profile_plot)
    outer.addWidget(plot_group, stretch=2)

    log_group = QGroupBox("Sensor Log")
    log_layout = QVBoxLayout(log_group)
    panel._sensor_log = QTextEdit()
    panel._sensor_log.setReadOnly(True)
    panel._sensor_log.setMaximumHeight(120)
    panel._sensor_log.setStyleSheet("font-family: monospace; font-size: 11px;")
    log_layout.addWidget(panel._sensor_log)
    outer.addWidget(log_group)

    outer.addStretch()
    return tab


def _set_sensor_active(panel, enable: bool):
    client = panel._sensor_activate_client if enable else panel._sensor_deactivate_client
    service_name = "/profilometer_activate" if enable else "/profilometer_deactivate"
    action = "ON" if enable else "OFF"

    if not client.wait_for_service(timeout_sec=1.0):
        _log(panel, f"{service_name} not available")
        return

    req = SensorCommand.Request()
    req.sensor_ip = ""
    req.ctrl_port = 0
    req.data_port = 0
    req.cmd = ""
    req.fps = int(panel._sensor_fps_spin.value())

    future = client.call_async(req)

    def _done(fut):
        result = fut.result()
        if result is None:
            _log(panel, f"Laser {action} request failed")
            return
        if result.success:
            panel._sensor_state_label.setText(f"State: {'ON' if enable else 'OFF'}")
            _log(panel, f"Laser {action} succeeded: {result.message}")
        else:
            _log(panel, f"Laser {action} failed: {result.message}")

    future.add_done_callback(_done)


def update_sensor_pointcloud(panel, pointcloud_msg):
    if getattr(panel, "_sensor_profile_plot", None) is None:
        return
    if getattr(panel, "_sensor_freeze_btn", None) is not None and panel._sensor_freeze_btn.isChecked():
        return

    panel._sensor_profile_plot.update_from_pointcloud(pointcloud_msg)

    points = int(pointcloud_msg.width) * int(pointcloud_msg.height)
    if getattr(panel, "_sensor_points_label", None) is not None:
        if points != getattr(panel, "_sensor_last_point_count", -1):
            panel._sensor_last_point_count = points
            panel._sensor_points_label.setText(f"Points: {points}")


def _reload_plot_config(panel):
    """Re-read laser_profile.yaml and redraw the profile widget."""
    if getattr(panel, "_sensor_profile_plot", None) is None:
        return
    panel._sensor_profile_plot.reload_config()
    plot = panel._sensor_profile_plot
    _log(panel,
         f"Config reloaded: Y=[{plot.Y_MIN_MM:.0f},{plot.Y_MAX_MM:.0f}] mm  "
         f"Z=[{plot.Z_MIN_MM:.0f},{plot.Z_MAX_MM:.0f}] mm")


def _log(panel, text: str):
    if getattr(panel, "_sensor_log", None) is not None:
        panel._sensor_log.append(text)
