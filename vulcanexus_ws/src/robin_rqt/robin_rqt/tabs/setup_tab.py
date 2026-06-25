"""
Setup tab — Plates + Calibration on left, Jog on right side panel.

Left column: stacked plate table, compact form, calibration sections.
Right column: Fixed-width jog controls with joystick pads.
No QToolBox, no scroll — all sections always visible.
"""

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QDoubleSpinBox, QLineEdit, QGroupBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QTextEdit, QComboBox, QMessageBox,
)
from python_qt_binding.QtCore import Qt

from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from ..components.widgets import apply_touch_style, TOUCH_MIN_H
from ..components.jog_component_v2 import build_jog_group
from ..components.ctwd_component import build_live_tcp_widget
from ..components.ctwd_calibration_component import build_ctwd_calibration_group
from ..components.plate_orientation_widget import RotatingPlateWidget

# Re-use plate helper functions from plates_tab (kept as utility module)
from .plates_tab import (
    DEFAULT_MARGIN_M,
    load_plates_from_config,
    auto_save_plates,
    refresh_plate_table,
    add_or_update_plate,
    delete_selected_plate,
    toggle_selected_plates,
    load_plate_to_form,
    load_plates_json,
    save_plates_json,
    capture_corner_pose,
    calibrate_selected_plate,
    plates_to_msgs,
    _set_corner,
)


def build_setup_tab(panel) -> QWidget:
    """Build the unified Setup tab: plates/calibration left, jog right."""
    tab = QWidget()
    root = QHBoxLayout(tab)
    root.setSpacing(4)
    root.setContentsMargins(4, 4, 4, 4)

    # ── Left column: everything stacked, no toolbox ───────────────────
    left = QWidget()
    left_lay = QVBoxLayout(left)
    left_lay.setSpacing(2)
    left_lay.setContentsMargins(0, 0, 0, 0)

    _build_plate_section(panel, left_lay)
    _build_calibration_section(panel, left_lay)

    # -- Shared log --
    log_group = QGroupBox('Setup Log')
    log_l = QVBoxLayout(log_group)
    log_l.setContentsMargins(4, 6, 4, 4)
    panel._cal_log = QTextEdit()
    panel._cal_log.setReadOnly(True)
    panel._cal_log.setMaximumHeight(48)
    log_l.addWidget(panel._cal_log)
    left_lay.addWidget(log_group)

    root.addWidget(left, stretch=1)

    # ── Right column: Live TCP + Jog (fixed width) ───────────────────
    right = QWidget()
    right_lay = QVBoxLayout(right)
    right_lay.setSpacing(4)
    right_lay.setContentsMargins(0, 0, 0, 0)

    # TCP position inside a thin group box for visual consistency
    tcp_group = QGroupBox('TCP Position')
    tcp_group_lay = QVBoxLayout(tcp_group)
    tcp_group_lay.setContentsMargins(2, 2, 2, 2)
    tcp_group_lay.addWidget(build_live_tcp_widget(panel))
    right_lay.addWidget(tcp_group)

    jog_widget = build_jog_group(panel)
    right_lay.addWidget(jog_widget)

    right_lay.addStretch(1)

    right.setMaximumWidth(280)
    root.addWidget(right)

    apply_touch_style(tab)
    return tab


# ---------------------------------------------------------------------------
# Plate Management section  (all widgets added directly to *parent_layout*)
# ---------------------------------------------------------------------------

def _build_plate_section(panel, parent_layout: QVBoxLayout):
    """Build plate table, compact form, calibration probe — no wrapper."""

    if not hasattr(panel, '_plate_defaults'):
        panel._plate_defaults = {
            'margin_x': DEFAULT_MARGIN_M,
            'margin_y': DEFAULT_MARGIN_M,
        }
    # ── Plate table (stretches to fill available space) ───────────────
    panel._plate_table = QTableWidget(0, 11)
    panel._plate_table.setHorizontalHeaderLabels([
        'On', 'Plate ID', 'Corner', 'Corner X', 'Corner Y',
        'W (mm)', 'L (mm)', 'Mx (mm)', 'My (mm)',
        'Surface Z', 'Cal',
    ])
    panel._plate_table.horizontalHeader().setSectionResizeMode(
        QHeaderView.Stretch)
    panel._plate_table.horizontalHeader().setSectionResizeMode(
        0, QHeaderView.ResizeToContents)
    panel._plate_table.setSelectionBehavior(QTableWidget.SelectRows)
    panel._plate_table.setMinimumHeight(120)
    parent_layout.addWidget(panel._plate_table, stretch=4)

    # ── Action buttons (right below the table) ────────────────────────
    btn_row = QHBoxLayout()
    btn_row.setSpacing(4)

    add_btn = QPushButton('Add / Update')
    add_btn.clicked.connect(lambda: add_or_update_plate(panel))
    btn_row.addWidget(add_btn)

    del_btn = QPushButton('Delete')
    del_btn.clicked.connect(lambda: delete_selected_plate(panel))
    btn_row.addWidget(del_btn)

    toggle_btn = QPushButton('Enable / Disable')
    toggle_btn.setToolTip('Toggle enabled state for selected plates')
    toggle_btn.clicked.connect(lambda: toggle_selected_plates(panel))
    btn_row.addWidget(toggle_btn)

    load_btn = QPushButton('Load…')
    load_btn.clicked.connect(lambda: load_plates_json(panel))
    btn_row.addWidget(load_btn)

    save_btn = QPushButton('Save…')
    save_btn.clicked.connect(lambda: save_plates_json(panel))
    btn_row.addWidget(save_btn)

    clear_beads_btn = QPushButton('Clear Beads (Sim)')
    clear_beads_btn.setToolTip('Remove all spawned bead entities from Gazebo')
    clear_beads_btn.clicked.connect(lambda: _clear_beads_in_sim(panel))
    btn_row.addWidget(clear_beads_btn)

    parent_layout.addLayout(btn_row)

    # ── Compact add/edit form ─────────────────────────────────────────
    form_group = QGroupBox('Add / Edit Plate')
    form_outer = QHBoxLayout(form_group)
    form_outer.setSpacing(6)
    form_outer.setContentsMargins(4, 6, 4, 4)

    # Left side: fields in a tight grid
    form = QGridLayout()
    form.setSpacing(4)
    form.setContentsMargins(0, 0, 0, 0)

    # Row 0: ID + Size (W × L) in mm
    form.addWidget(QLabel('ID:'), 0, 0)
    panel._plate_id_edit = QLineEdit()
    panel._plate_id_edit.setPlaceholderText('plate_A')
    form.addWidget(panel._plate_id_edit, 0, 1)

    form.addWidget(QLabel('W:'), 0, 2)
    panel._plate_w = QDoubleSpinBox()
    panel._plate_w.setRange(10, 1000)
    panel._plate_w.setDecimals(0)
    panel._plate_w.setSingleStep(10)
    panel._plate_w.setValue(300)
    panel._plate_w.setSuffix(' mm')
    panel._plate_w.valueChanged.connect(
        lambda: _sync_plate_widget(panel))
    form.addWidget(panel._plate_w, 0, 3)

    form.addWidget(QLabel('L:'), 0, 4)
    panel._plate_l = QDoubleSpinBox()
    panel._plate_l.setRange(10, 1000)
    panel._plate_l.setDecimals(0)
    panel._plate_l.setSingleStep(10)
    panel._plate_l.setValue(300)
    panel._plate_l.setSuffix(' mm')
    panel._plate_l.valueChanged.connect(
        lambda: _sync_plate_widget(panel))
    form.addWidget(panel._plate_l, 0, 5)

    # Row 1: Corner position (auto-filled by Capture TF) + corner selector
    form.addWidget(QLabel('Corner:'), 1, 0)
    panel._corner_combo = QComboBox()
    panel._corner_combo.addItems(['front_left', 'front_right', 'rear_left', 'rear_right'])
    panel._corner_combo.currentTextChanged.connect(
        lambda c: _set_corner(panel, c))
    form.addWidget(panel._corner_combo, 1, 1)

    form.addWidget(QLabel('X:'), 1, 2)
    panel._plate_ox = QDoubleSpinBox()
    panel._plate_ox.setRange(-2.0, 2.0)
    panel._plate_ox.setDecimals(4)
    panel._plate_ox.setSuffix(' m')
    form.addWidget(panel._plate_ox, 1, 3)

    form.addWidget(QLabel('Y:'), 1, 4)
    panel._plate_oy = QDoubleSpinBox()
    panel._plate_oy.setRange(-2.0, 2.0)
    panel._plate_oy.setDecimals(4)
    panel._plate_oy.setSuffix(' m')
    form.addWidget(panel._plate_oy, 1, 5)

    # Row 2: Z + Capture button
    form.addWidget(QLabel('Z:'), 2, 0)
    panel._plate_oz = QDoubleSpinBox()
    panel._plate_oz.setRange(-1.0, 1.0)
    panel._plate_oz.setDecimals(4)
    panel._plate_oz.setSuffix(' m')
    form.addWidget(panel._plate_oz, 2, 1)

    capture_btn = QPushButton('Capture Corner TF')
    capture_btn.setToolTip('Capture corner position from wire_tip TF')
    capture_btn.clicked.connect(lambda: capture_corner_pose(panel))
    form.addWidget(capture_btn, 2, 2, 1, 2)

    # Row 3: Margins (mm) + Yaw
    form.addWidget(QLabel('MarginX:'), 3, 0)
    panel._plate_mx = QDoubleSpinBox()
    panel._plate_mx.setRange(0, 200)
    panel._plate_mx.setDecimals(0)
    panel._plate_mx.setSingleStep(10)
    panel._plate_mx.setValue(DEFAULT_MARGIN_M * 1000)
    panel._plate_mx.setSuffix(' mm')
    panel._plate_mx.valueChanged.connect(
        lambda: _sync_plate_widget(panel))
    form.addWidget(panel._plate_mx, 3, 1)

    form.addWidget(QLabel('MarginY:'), 3, 2)
    panel._plate_my = QDoubleSpinBox()
    panel._plate_my.setRange(0, 200)
    panel._plate_my.setDecimals(0)
    panel._plate_my.setSingleStep(10)
    panel._plate_my.setValue(DEFAULT_MARGIN_M * 1000)
    panel._plate_my.setSuffix(' mm')
    panel._plate_my.valueChanged.connect(
        lambda: _sync_plate_widget(panel))
    form.addWidget(panel._plate_my, 3, 3)

    form.addWidget(QLabel('Yaw:'), 3, 4)
    panel._plate_yaw = QDoubleSpinBox()
    panel._plate_yaw.setRange(-180.0, 180.0)
    panel._plate_yaw.setDecimals(1)
    panel._plate_yaw.setSuffix(' °')
    panel._plate_yaw.valueChanged.connect(
        lambda v: panel._plate_orient_widget.set_yaw(v))
    form.addWidget(panel._plate_yaw, 3, 5)

    # Global margins alias — single margin set
    panel._global_margin_x = panel._plate_mx
    panel._global_margin_y = panel._plate_my

    form_outer.addLayout(form, stretch=1)

    # Right side: rotating plate widget
    panel._corner_id = 'front_left'
    panel._plate_orient_widget = RotatingPlateWidget()
    panel._plate_orient_widget.setMinimumSize(120, 120)
    panel._plate_orient_widget.setMaximumSize(160, 160)
    panel._plate_orient_widget.yawChanged.connect(
        lambda deg: _on_plate_yaw_changed(panel, deg))
    _sync_plate_widget(panel)

    form_outer.addWidget(panel._plate_orient_widget)

    parent_layout.addWidget(form_group)

    # ── Plate calibration (4-point probe) — single row ────────────────
    probe_group = QGroupBox('Plate Calibration')
    probe_lay = QHBoxLayout(probe_group)
    probe_lay.setSpacing(4)
    probe_lay.setContentsMargins(4, 6, 4, 4)

    probe_lay.addWidget(QLabel('Speed:'))
    panel._probe_speed = QDoubleSpinBox()
    panel._probe_speed.setRange(0.001, 0.10)
    panel._probe_speed.setDecimals(3)
    panel._probe_speed.setValue(0.015)
    panel._probe_speed.setSuffix(' m/s')
    panel._probe_speed.setToolTip(
        'Probe approach speed (m/s). Heights auto-computed from corner Z.')
    probe_lay.addWidget(panel._probe_speed)

    calibrate_btn = QPushButton('Calibrate')
    calibrate_btn.clicked.connect(lambda: calibrate_selected_plate(panel))
    probe_lay.addWidget(calibrate_btn)

    panel._plate_cal_result = QLabel('Result: —')
    panel._plate_cal_result.setStyleSheet(
        'font-family: monospace; font-size: 11px;')
    probe_lay.addWidget(panel._plate_cal_result, stretch=1)

    parent_layout.addWidget(probe_group)

    # Click table row → load into form (also sync plate widget)
    panel._plate_table.cellClicked.connect(
        lambda r, c: _load_and_sync(panel, r))


# ---------------------------------------------------------------------------
# Plate orientation widget helpers
# ---------------------------------------------------------------------------

def _sync_plate_widget(panel):
    """Push current spinbox values (mm) into the RotatingPlateWidget (m)."""
    w = panel._plate_orient_widget
    w.set_plate_dims(panel._plate_w.value() / 1000.0,
                     panel._plate_l.value() / 1000.0)
    w.set_margins(panel._plate_mx.value() / 1000.0,
                  panel._plate_my.value() / 1000.0)
    w.set_yaw(panel._plate_yaw.value())


def _on_plate_yaw_changed(panel, deg: float):
    """Called when the RotatingPlateWidget drag changes yaw."""
    panel._plate_yaw.blockSignals(True)
    panel._plate_yaw.setValue(deg)
    panel._plate_yaw.blockSignals(False)


def _load_and_sync(panel, row):
    """Load plate from table row, then sync the orientation widget."""
    load_plate_to_form(panel, row)
    _sync_plate_widget(panel)


# ---------------------------------------------------------------------------
# Calibration section (CTWD / TCP)
# ---------------------------------------------------------------------------

def _build_calibration_section(panel, parent_layout: QVBoxLayout):
    """Add stickout calibration widget directly to *parent_layout*."""
    parent_layout.addWidget(build_ctwd_calibration_group(panel))


# ---------------------------------------------------------------------------
# Simulation helpers
# ---------------------------------------------------------------------------

def _clear_beads_in_sim(panel):
    """Remove all bead entities from Gazebo AND clear bead persistence JSON."""
    log = getattr(panel, '_cal_log', None)

    # 1. Publish to Gazebo to remove visual bead entities
    if not hasattr(panel, '_clear_beads_pub'):
        panel._clear_beads_pub = panel._node.create_publisher(
            Bool, '/robin/clear_beads', 10)
    msg = Bool()
    msg.data = True
    panel._clear_beads_pub.publish(msg)
    if log:
        log.append('Sent clear-beads command to Gazebo')

    # 2. Call service to clear all bead records from beads.json
    if not hasattr(panel, '_clear_all_beads_client'):
        panel._clear_all_beads_client = panel._node.create_client(
            Trigger, '/plate/clear_all_beads')
    client = panel._clear_all_beads_client
    if client.service_is_ready():
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: _on_clear_beads_done(panel, f))
    else:
        if log:
            log.append('clear_all_beads service not available — JSON not cleared')


def _on_clear_beads_done(panel, future):
    """Handle the async service response for clear_all_beads."""
    log = getattr(panel, '_cal_log', None)
    try:
        result = future.result()
        if log:
            log.append(f'Beads JSON: {result.message}')
    except Exception as e:
        if log:
            log.append(f'clear_all_beads failed: {e}')
