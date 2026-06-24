"""Plates tab — define/capture/calibrate plates by corner position and plane."""

import json
import math
import os

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QDoubleSpinBox, QLineEdit, QGroupBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QMessageBox, QFileDialog,
)
from python_qt_binding.QtCore import Qt, QPointF
from python_qt_binding.QtGui import QPainter, QPen, QBrush, QColor, QPolygonF

import rclpy
from geometry_msgs.msg import Point
from robin_interfaces.srv import CalibratePlatePlane
from robin_core.plate_geometry import corner_to_center


DEFAULT_MARGIN_M = 0.040


class CornerSelectorWidget(QWidget):
    """Clickable square corner selector with simple base_link axis hint."""

    _KEYS = ('front_left', 'front_right', 'rear_left', 'rear_right')

    def __init__(self, on_corner_selected, parent=None):
        super().__init__(parent)
        self._on_corner_selected = on_corner_selected
        self._selected = 'front_left'
        self.setMinimumSize(70, 70)

    def set_selected_corner(self, corner_id: str):
        if corner_id in self._KEYS:
            self._selected = corner_id
            self.update()

    def _corner_points(self):
        pad = 24
        w = max(80, self.width() - 2 * pad)
        h = max(80, self.height() - 2 * pad)
        x0 = (self.width() - w) / 2.0
        y0 = (self.height() - h) / 2.0
        # Operator-view map (top-down) rotated 180° vs world map:
        # screen top = robot rear (+X), screen bottom = robot front (-X).
        return {
            'rear_right': QPointF(x0, y0),
            'rear_left': QPointF(x0 + w, y0),
            'front_right': QPointF(x0, y0 + h),
            'front_left': QPointF(x0 + w, y0 + h),
        }

    def mousePressEvent(self, event):
        pts = self._corner_points()
        if hasattr(event, 'position'):
            p = event.position()
            px = float(p.x())
            py = float(p.y())
        elif hasattr(event, 'pos'):
            p = event.pos()
            px = float(p.x())
            py = float(p.y())
        else:
            return
        best_key = self._selected
        best_dist = None
        for key, q in pts.items():
            dx = float(px - q.x())
            dy = float(py - q.y())
            d2 = dx * dx + dy * dy
            if best_dist is None or d2 < best_dist:
                best_dist = d2
                best_key = key
        self._selected = best_key
        self.update()
        if self._on_corner_selected is not None:
            self._on_corner_selected(best_key)

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        pts = self._corner_points()
        order = ['front_left', 'front_right', 'rear_right', 'rear_left', 'front_left']

        # Plate fill for better visual readability.
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor('#ECEFF1')))
        poly = QPolygonF([pts[k] for k in ['front_left', 'front_right', 'rear_right', 'rear_left']])
        painter.drawPolygon(poly)

        painter.setPen(QPen(QColor('#607D8B'), 2))
        for i in range(len(order) - 1):
            a = pts[order[i]]
            b = pts[order[i + 1]]
            painter.drawLine(a, b)

        for key, q in pts.items():
            is_sel = (key == self._selected)
            painter.setPen(QPen(QColor('#263238'), 1.5 if is_sel else 1))
            painter.setBrush(QBrush(QColor('#43A047') if is_sel else QColor('#B0BEC5')))
            painter.drawEllipse(q, 8 if is_sel else 6, 8 if is_sel else 6)

        labels = {
            'front_left': 'FL', 'front_right': 'FR',
            'rear_left': 'RL', 'rear_right': 'RR',
        }
        painter.setPen(QPen(QColor('#263238'), 1))
        for key, q in pts.items():
            painter.drawText(QPointF(q.x() + 8, q.y() - 8), labels[key])

        # base_link axis hint (operator-view schematic)
        origin = QPointF(18, self.height() - 18)
        painter.setPen(QPen(QColor('#E53935'), 2))
        painter.drawLine(origin, QPointF(origin.x(), origin.y() - 28))
        painter.drawText(QPointF(origin.x() + 4, origin.y() - 32), '+X')
        painter.setPen(QPen(QColor('#1E88E5'), 2))
        painter.drawLine(origin, QPointF(origin.x() + 28, origin.y()))
        painter.drawText(QPointF(origin.x() + 30, origin.y() + 4), '+Y')

        painter.setPen(QPen(QColor('#455A64'), 1))
        painter.drawText(QPointF(self.width() * 0.5 - 46, self.height() - 6), 'Operator side')


def build_plates_tab(panel) -> QWidget:
    """Build the Plates tab widget and register table/form widgets on *panel*."""
    tab = QWidget()
    outer = QVBoxLayout(tab)

    if not hasattr(panel, '_plate_defaults'):
        panel._plate_defaults = {'margin_x': DEFAULT_MARGIN_M, 'margin_y': DEFAULT_MARGIN_M}

    # ---- Plate table ----
    panel._plate_table = QTableWidget(0, 11)
    panel._plate_table.setHorizontalHeaderLabels([
        'On', 'Plate ID', 'Corner', 'Corner X', 'Corner Y',
        'W (m)', 'L (m)', 'Mx (m)', 'My (m)',
        'Surface Z', 'Cal',
    ])
    panel._plate_table.horizontalHeader().setSectionResizeMode(
        QHeaderView.Stretch)
    panel._plate_table.setSelectionBehavior(QTableWidget.SelectRows)
    outer.addWidget(panel._plate_table, stretch=3)

    # ---- Add / edit form ----
    form_group = QGroupBox('Add / Edit Plate')
    form = QGridLayout(form_group)

    form.addWidget(QLabel('Global Margin X:'), 0, 0)
    panel._global_margin_x = QDoubleSpinBox()
    panel._global_margin_x.setRange(0.0, 0.2)
    panel._global_margin_x.setDecimals(4)
    panel._global_margin_x.setValue(float(panel._plate_defaults.get('margin_x', DEFAULT_MARGIN_M)))
    panel._global_margin_x.setSuffix(' m')
    form.addWidget(panel._global_margin_x, 0, 1)

    form.addWidget(QLabel('Global Margin Y:'), 0, 2)
    panel._global_margin_y = QDoubleSpinBox()
    panel._global_margin_y.setRange(0.0, 0.2)
    panel._global_margin_y.setDecimals(4)
    panel._global_margin_y.setValue(float(panel._plate_defaults.get('margin_y', DEFAULT_MARGIN_M)))
    panel._global_margin_y.setSuffix(' m')
    form.addWidget(panel._global_margin_y, 0, 3)

    use_global_btn = QPushButton('Use Global Margins')
    use_global_btn.clicked.connect(lambda: _apply_global_margins_to_plate(panel))
    form.addWidget(use_global_btn, 0, 4)

    panel._plate_id_edit = QLineEdit()
    panel._plate_id_edit.setPlaceholderText('e.g. plate_A')
    form.addWidget(QLabel('ID:'), 1, 0)
    form.addWidget(panel._plate_id_edit, 1, 1)

    panel._corner_id = 'front_left'
    form.addWidget(QLabel('Touched Corner:'), 1, 2)
    panel._corner_selector = CornerSelectorWidget(lambda c: _set_corner(panel, c))
    form.addWidget(panel._corner_selector, 1, 3, 2, 2)
    _set_corner(panel, panel._corner_id)

    form.addWidget(QLabel('Corner select:'), 2, 0)
    form.addWidget(QLabel('Click one corner on graphic (base_link axes shown).'), 2, 1, 1, 2)

    panel._plate_ox = QDoubleSpinBox(); panel._plate_ox.setRange(-2.0, 2.0)
    panel._plate_ox.setDecimals(4); panel._plate_ox.setSuffix(' m')
    panel._plate_oy = QDoubleSpinBox(); panel._plate_oy.setRange(-2.0, 2.0)
    panel._plate_oy.setDecimals(4); panel._plate_oy.setSuffix(' m')
    panel._plate_oz = QDoubleSpinBox(); panel._plate_oz.setRange(-1.0, 1.0)
    panel._plate_oz.setDecimals(4); panel._plate_oz.setSuffix(' m')
    form.addWidget(QLabel('Corner X:'), 3, 0); form.addWidget(panel._plate_ox, 3, 1)
    form.addWidget(QLabel('Corner Y:'), 3, 2); form.addWidget(panel._plate_oy, 3, 3)
    form.addWidget(QLabel('Corner Z:'), 3, 4); form.addWidget(panel._plate_oz, 3, 5)

    panel._plate_w = QDoubleSpinBox(); panel._plate_w.setRange(0.01, 1.0)
    panel._plate_w.setDecimals(4); panel._plate_w.setValue(0.15); panel._plate_w.setSuffix(' m')
    panel._plate_l = QDoubleSpinBox(); panel._plate_l.setRange(0.01, 1.0)
    panel._plate_l.setDecimals(4); panel._plate_l.setValue(0.20); panel._plate_l.setSuffix(' m')
    form.addWidget(QLabel('Width:'), 4, 0); form.addWidget(panel._plate_w, 4, 1)
    form.addWidget(QLabel('Length:'), 4, 2); form.addWidget(panel._plate_l, 4, 3)

    panel._plate_yaw = QDoubleSpinBox(); panel._plate_yaw.setRange(-180.0, 180.0)
    panel._plate_yaw.setDecimals(1); panel._plate_yaw.setSuffix(' °')
    panel._plate_mx = QDoubleSpinBox(); panel._plate_mx.setRange(0.0, 0.2)
    panel._plate_mx.setDecimals(4); panel._plate_mx.setValue(DEFAULT_MARGIN_M); panel._plate_mx.setSuffix(' m')
    panel._plate_my = QDoubleSpinBox(); panel._plate_my.setRange(0.0, 0.2)
    panel._plate_my.setDecimals(4); panel._plate_my.setValue(DEFAULT_MARGIN_M); panel._plate_my.setSuffix(' m')
    form.addWidget(QLabel('Yaw:'), 5, 0); form.addWidget(panel._plate_yaw, 5, 1)
    form.addWidget(QLabel('Margin X:'), 5, 2); form.addWidget(panel._plate_mx, 5, 3)
    form.addWidget(QLabel('Margin Y:'), 5, 4); form.addWidget(panel._plate_my, 5, 5)

    probe_group = QGroupBox('Plate Calibration (4-point probe + plane fit)')
    probe_form = QGridLayout(probe_group)
    # Probe height is auto-computed (corner_z +30mm start / table-floor limit) — no user input needed.
    panel._probe_speed = QDoubleSpinBox(); panel._probe_speed.setRange(0.001, 0.10)
    panel._probe_speed.setDecimals(3); panel._probe_speed.setValue(0.015); panel._probe_speed.setSuffix(' m/s')
    panel._probe_speed.setToolTip('Probe approach speed (m/s). Heights are auto-computed: start = corner_Z +30 mm, limit = configurable table Z floor (default −0.200 m).')
    probe_form.addWidget(QLabel('Probe Speed:'), 0, 0); probe_form.addWidget(panel._probe_speed, 0, 1)
    calibrate_btn = QPushButton('Calibrate Selected Plate (4 points)')
    calibrate_btn.clicked.connect(lambda: calibrate_selected_plate(panel))
    probe_form.addWidget(calibrate_btn, 0, 2, 1, 3)
    panel._plate_cal_result = QLabel('Result: —')
    panel._plate_cal_result.setStyleSheet('font-family: monospace; font-size: 11px;')
    probe_form.addWidget(panel._plate_cal_result, 1, 0, 1, 6)

    btn_row = QHBoxLayout()
    add_btn = QPushButton('Add / Update')
    add_btn.clicked.connect(lambda: add_or_update_plate(panel))
    btn_row.addWidget(add_btn)

    del_btn = QPushButton('Delete Selected')
    del_btn.clicked.connect(lambda: delete_selected_plate(panel))
    btn_row.addWidget(del_btn)

    load_btn = QPushButton('Load from JSON...')
    load_btn.clicked.connect(lambda: load_plates_json(panel))
    btn_row.addWidget(load_btn)

    save_btn = QPushButton('Save to JSON...')
    save_btn.clicked.connect(lambda: save_plates_json(panel))
    btn_row.addWidget(save_btn)

    capture_btn = QPushButton('Capture Corner from wire_tip TF')
    capture_btn.clicked.connect(lambda: capture_corner_pose(panel))
    btn_row.addWidget(capture_btn)

    form.addLayout(btn_row, 6, 0, 1, 6)
    outer.addWidget(form_group, stretch=2)
    outer.addWidget(probe_group)

    # Load selected row into form on click
    panel._plate_table.cellClicked.connect(lambda r, c: load_plate_to_form(panel, r))

    return tab


# ---------------------------------------------------------------------------
# Plate config location
# ---------------------------------------------------------------------------

def locate_plates_config() -> str:
    """Find the plates.json config file.

    Search order:
    1. $ROBIN_PLATES_CONFIG  (env-var override)
    2. robin_bringup share dir (installed package)
    3. Source-tree default (developer convenience)
    """
    env_path = os.environ.get('ROBIN_PLATES_CONFIG', '')
    if env_path and os.path.isfile(env_path):
        return env_path

    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('robin_bringup')
        pkg_path = os.path.join(share, 'config', 'plates.json')
        if os.path.isfile(pkg_path):
            return pkg_path
    except Exception:
        pass

    src_path = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..',
        'robin_bringup', 'config', 'plates.json')
    src_path = os.path.normpath(src_path)
    if os.path.isfile(src_path):
        return src_path

    return ''


def load_plates_from_config(panel):
    """Load plates from the first config file found on disk."""
    path = locate_plates_config()
    if not path:
        panel._node.get_logger().info(
            'No plates config found — starting with empty plate list')
        return
    try:
        with open(path) as f:
            data = json.load(f)
        if isinstance(data, list):
            panel._plates = [_normalize_plate_entry(p) for p in data]
            panel._calibration_point = None
            panel._plate_defaults = {'margin_x': DEFAULT_MARGIN_M, 'margin_y': DEFAULT_MARGIN_M}
        else:
            panel._plates = [_normalize_plate_entry(p) for p in data.get('plates', [])]
            panel._calibration_point = data.get('calibration_point')
            panel._plate_defaults = data.get(
                'plate_defaults', {'margin_x': DEFAULT_MARGIN_M, 'margin_y': DEFAULT_MARGIN_M})
        panel._plates_path = path
        if hasattr(panel, '_global_margin_x'):
            panel._global_margin_x.setValue(
                float(panel._plate_defaults.get('margin_x', DEFAULT_MARGIN_M)) * 1000.0)
        if hasattr(panel, '_global_margin_y'):
            panel._global_margin_y.setValue(
                float(panel._plate_defaults.get('margin_y', DEFAULT_MARGIN_M)) * 1000.0)
        refresh_plate_table(panel)
        panel._node.get_logger().info(
            f'Loaded {len(panel._plates)} plate(s) from {path}')
    except Exception as e:
        panel._node.get_logger().warn(f'Failed to load plates config: {e}')


def auto_save_plates(panel):
    """Persist plates back to the config file (if we have a path)."""
    if not panel._plates_path:
        return
    try:
        with open(panel._plates_path, 'w') as f:
            payload = {
                'plates': panel._plates,
                'calibration_point': getattr(panel, '_calibration_point', None),
                'plate_defaults': getattr(panel, '_plate_defaults', {
                    'margin_x': DEFAULT_MARGIN_M, 'margin_y': DEFAULT_MARGIN_M,
                }),
            }
            json.dump(payload, f, indent=2)
            f.write('\n')
    except Exception as e:
        panel._node.get_logger().warn(f'Auto-save plates failed: {e}')


def toggle_selected_plates(panel):
    """Toggle enabled/disabled state for selected plates."""
    rows = set(idx.row() for idx in panel._plate_table.selectedIndexes())
    if not rows:
        return
    col = 1 if panel._plate_table.columnCount() == 11 else 0
    for r in rows:
        pid = panel._plate_table.item(r, col).text()
        for p in panel._plates:
            if p['plate_id'] == pid:
                p['enabled'] = not p.get('enabled', True)
                break
    refresh_plate_table(panel)
    auto_save_plates(panel)


def refresh_plate_table(panel):
    """Re-populate the plate table from panel._plates."""
    panel._plate_table.setRowCount(0)
    has_on_col = panel._plate_table.columnCount() == 11
    disabled_fg = QColor('#999999')
    for p in panel._plates:
        row = panel._plate_table.rowCount()
        panel._plate_table.insertRow(row)
        enabled = p.get('enabled', True)
        if has_on_col:
            on_item = QTableWidgetItem('\u2713' if enabled else '')
            on_item.setTextAlignment(Qt.AlignCenter)
            panel._plate_table.setItem(row, 0, on_item)
        off = 1 if has_on_col else 0
        panel._plate_table.setItem(row, off + 0, QTableWidgetItem(p['plate_id']))
        panel._plate_table.setItem(row, off + 1, QTableWidgetItem(p.get('corner_id', 'front_left')))
        panel._plate_table.setItem(row, off + 2, QTableWidgetItem(f"{p['corner_x']:.4f}"))
        panel._plate_table.setItem(row, off + 3, QTableWidgetItem(f"{p['corner_y']:.4f}"))
        panel._plate_table.setItem(row, off + 4, QTableWidgetItem(f"{p['width'] * 1000:.0f}"))
        panel._plate_table.setItem(row, off + 5, QTableWidgetItem(f"{p['length'] * 1000:.0f}"))
        panel._plate_table.setItem(row, off + 6, QTableWidgetItem(f"{p['margin_x'] * 1000:.0f}"))
        panel._plate_table.setItem(row, off + 7, QTableWidgetItem(f"{p['margin_y'] * 1000:.0f}"))
        panel._plate_table.setItem(row, off + 8, QTableWidgetItem(f"{p.get('surface_z', 0.0):.4f}"))
        panel._plate_table.setItem(row, off + 9, QTableWidgetItem('Yes' if p.get('is_calibrated') else 'No'))
        if not enabled:
            for c in range(panel._plate_table.columnCount()):
                item = panel._plate_table.item(row, c)
                if item:
                    item.setForeground(disabled_fg)

    if hasattr(panel, '_cal_plate_combo'):
        panel._cal_plate_combo.clear()
        for p in panel._plates:
            panel._cal_plate_combo.addItem(p['plate_id'])

    if hasattr(panel, '_refresh_ctwd_sources'):
        panel._refresh_ctwd_sources()

    if hasattr(panel, '_update_contact_point_label'):
        panel._update_contact_point_label()


def add_or_update_plate(panel):
    pid = panel._plate_id_edit.text().strip()
    if not pid:
        QMessageBox.warning(panel._widget, 'Error', 'Plate ID cannot be empty.')
        return

    panel._plate_defaults = {
        'margin_x': panel._global_margin_x.value() / 1000.0,
        'margin_y': panel._global_margin_y.value() / 1000.0,
    }

    # Read form values — W/L/margins are in mm, position in m
    corner_x = panel._plate_ox.value()
    corner_y = panel._plate_oy.value()
    corner_z = panel._plate_oz.value() if hasattr(panel, '_plate_oz') else 0.0
    corner_id = panel._corner_id
    width_m = panel._plate_w.value() / 1000.0
    length_m = panel._plate_l.value() / 1000.0
    yaw_deg = panel._plate_yaw.value()
    yaw_rad = math.radians(yaw_deg)
    mx_m = panel._plate_mx.value() / 1000.0
    my_m = panel._plate_my.value() / 1000.0

    center_x, center_y = corner_to_center(
        corner_x, corner_y, yaw_rad, corner_id, length_m, width_m)

    # Preserve calibration data / surface_z from existing plate
    surface_z = corner_z
    existing = next((p for p in panel._plates if p['plate_id'] == pid), None)
    cal_fields = {}
    if existing:
        surface_z = existing.get('surface_z', surface_z)
        for k in ('is_calibrated', 'plane_calibrated', 'plane_a', 'plane_b',
                  'plane_c', 'probe_points'):
            cal_fields[k] = existing.get(k)

    entry = {
        'plate_id': pid,
        'enabled': existing.get('enabled', True) if existing else True,
        'corner_id': corner_id,
        'corner_x': corner_x,
        'corner_y': corner_y,
        'corner_z': corner_z,
        'center_x': center_x,
        'center_y': center_y,
        'width': width_m,
        'length': length_m,
        'yaw_deg': yaw_deg,
        'margin_x': mx_m,
        'margin_y': my_m,
        'surface_z': surface_z,
        'is_calibrated': cal_fields.get('is_calibrated', False),
        'plane_calibrated': cal_fields.get('plane_calibrated', False),
        'plane_a': cal_fields.get('plane_a', 0.0),
        'plane_b': cal_fields.get('plane_b', 0.0),
        'plane_c': cal_fields.get('plane_c', surface_z),
        'probe_points': cal_fields.get('probe_points', []),
    }

    for i, p in enumerate(panel._plates):
        if p['plate_id'] == pid:
            panel._plates[i] = entry
            break
    else:
        panel._plates.append(entry)

    refresh_plate_table(panel)
    auto_save_plates(panel)


def delete_selected_plate(panel):
    rows = set(idx.row() for idx in panel._plate_table.selectedIndexes())
    if not rows:
        return
    col = 1 if panel._plate_table.columnCount() == 11 else 0
    ids_to_remove = {
        panel._plate_table.item(r, col).text() for r in rows
    }
    panel._plates = [p for p in panel._plates if p['plate_id'] not in ids_to_remove]
    refresh_plate_table(panel)
    auto_save_plates(panel)


def load_plate_to_form(panel, row):
    col = 1 if panel._plate_table.columnCount() == 11 else 0
    pid = panel._plate_table.item(row, col).text()
    for p in panel._plates:
        if p['plate_id'] == pid:
            panel._plate_id_edit.setText(p['plate_id'])
            _set_corner(panel, p.get('corner_id', 'front_left'))
            panel._plate_ox.setValue(p.get('corner_x', 0.0))
            panel._plate_oy.setValue(p.get('corner_y', 0.0))
            if hasattr(panel, '_plate_oz'):
                panel._plate_oz.setValue(p.get('corner_z', p.get('surface_z', 0.0)))
            panel._plate_w.setValue(p['width'] * 1000.0)   # m → mm
            panel._plate_l.setValue(p['length'] * 1000.0)
            panel._plate_yaw.setValue(p['yaw_deg'])
            panel._plate_mx.setValue(p.get('margin_x', DEFAULT_MARGIN_M) * 1000.0)
            panel._plate_my.setValue(p.get('margin_y', DEFAULT_MARGIN_M) * 1000.0)
            break


def load_plates_json(panel):
    path, _ = QFileDialog.getOpenFileName(
        panel._widget, 'Load Plates JSON', '', 'JSON (*.json)')
    if not path:
        return
    try:
        with open(path) as f:
            data = json.load(f)
        if isinstance(data, list):
            panel._plates = [_normalize_plate_entry(p) for p in data]
            panel._calibration_point = None
            panel._plate_defaults = {'margin_x': DEFAULT_MARGIN_M, 'margin_y': DEFAULT_MARGIN_M}
        else:
            panel._plates = [_normalize_plate_entry(p) for p in data.get('plates', [])]
            panel._calibration_point = data.get('calibration_point')
            panel._plate_defaults = data.get(
                'plate_defaults', {'margin_x': DEFAULT_MARGIN_M, 'margin_y': DEFAULT_MARGIN_M})
        panel._plates_path = path
        if hasattr(panel, '_global_margin_x'):
            panel._global_margin_x.setValue(
                float(panel._plate_defaults.get('margin_x', DEFAULT_MARGIN_M)) * 1000.0)
        if hasattr(panel, '_global_margin_y'):
            panel._global_margin_y.setValue(
                float(panel._plate_defaults.get('margin_y', DEFAULT_MARGIN_M)) * 1000.0)
        refresh_plate_table(panel)
    except Exception as e:
        QMessageBox.warning(panel._widget, 'Load Error', str(e))


def save_plates_json(panel):
    default_name = panel._plates_path if panel._plates_path else 'plates.json'
    path, _ = QFileDialog.getSaveFileName(
        panel._widget, 'Save Plates JSON', default_name, 'JSON (*.json)')
    if not path:
        return
    try:
        with open(path, 'w') as f:
            payload = {
                'plates': panel._plates,
                'calibration_point': getattr(panel, '_calibration_point', None),
                'plate_defaults': getattr(panel, '_plate_defaults', {
                    'margin_x': DEFAULT_MARGIN_M, 'margin_y': DEFAULT_MARGIN_M,
                }),
            }
            json.dump(payload, f, indent=2)
            f.write('\n')
        panel._plates_path = path
    except Exception as e:
        QMessageBox.warning(panel._widget, 'Save Error', str(e))


def plates_to_msgs(panel) -> list:
    """Convert enabled plate dicts to robin_interfaces/WeldPlate msgs."""
    from robin_interfaces.msg import WeldPlate
    msgs = []
    for p in panel._plates:
        if not p.get('enabled', True):
            continue
        wp = WeldPlate()
        wp.plate_id = p['plate_id']
        corner_x = float(p.get('corner_x', p.get('origin_x', 0.0)))
        corner_y = float(p.get('corner_y', p.get('origin_y', 0.0)))
        corner_z = float(p.get('corner_z', p.get('surface_z', 0.0)))
        wp.origin = Point(
            x=corner_x, y=corner_y, z=corner_z)
        wp.width = float(p['width'])
        wp.length = float(p['length'])
        wp.orientation = float(math.radians(p['yaw_deg']))
        wp.surface_z = float(p['surface_z'])
        wp.is_calibrated = bool(p.get('is_calibrated', False))
        wp.corner_id = str(p.get('corner_id', 'front_left'))
        wp.margin_x = float(p.get('margin_x', DEFAULT_MARGIN_M))
        wp.margin_y = float(p.get('margin_y', DEFAULT_MARGIN_M))
        wp.plane_calibrated = bool(p.get('plane_calibrated', False))
        wp.plane_a = float(p.get('plane_a', 0.0))
        wp.plane_b = float(p.get('plane_b', 0.0))
        wp.plane_c = float(p.get('plane_c', wp.surface_z))
        wp.probe_points = []
        for q in p.get('probe_points', []):
            wp.probe_points.append(Point(
                x=float(q.get('x', 0.0)),
                y=float(q.get('y', 0.0)),
                z=float(q.get('z', 0.0)),
            ))
        msgs.append(wp)
    return msgs


def _set_corner(panel, corner_id: str):
    panel._corner_id = corner_id
    if hasattr(panel, '_corner_selector'):
        panel._corner_selector.set_selected_corner(corner_id)
    if hasattr(panel, '_corner_combo'):
        panel._corner_combo.blockSignals(True)
        idx = panel._corner_combo.findText(corner_id)
        if idx >= 0:
            panel._corner_combo.setCurrentIndex(idx)
        panel._corner_combo.blockSignals(False)


def _apply_global_margins_to_plate(panel):
    panel._plate_mx.setValue(panel._global_margin_x.value())
    panel._plate_my.setValue(panel._global_margin_y.value())


def _normalize_plate_entry(raw: dict) -> dict:
    corner_x = float(raw.get('corner_x', raw.get('origin_x', 0.0)))
    corner_y = float(raw.get('corner_y', raw.get('origin_y', 0.0)))
    corner_z = float(raw.get('corner_z', raw.get('surface_z', 0.0)))
    corner_id = str(raw.get('corner_id', 'front_left'))
    surface_z = float(raw.get('surface_z', corner_z))
    width = float(raw.get('width', 0.30))
    length = float(raw.get('length', 0.30))
    yaw_deg = float(raw.get('yaw_deg', 0.0))
    yaw_rad = math.radians(yaw_deg)
    # Compute center from corner (backward-compat for old JSON), or use stored
    if 'center_x' in raw and 'center_y' in raw:
        center_x = float(raw['center_x'])
        center_y = float(raw['center_y'])
    else:
        center_x, center_y = corner_to_center(
            corner_x, corner_y, yaw_rad, corner_id, length, width)
    return {
        'plate_id': str(raw.get('plate_id', 'plate')),
        'enabled': bool(raw.get('enabled', True)),
        'corner_id': corner_id,
        'corner_x': corner_x,
        'corner_y': corner_y,
        'corner_z': corner_z,
        'center_x': center_x,
        'center_y': center_y,
        'width': width,
        'length': length,
        'yaw_deg': yaw_deg,
        'margin_x': float(raw.get('margin_x', DEFAULT_MARGIN_M)),
        'margin_y': float(raw.get('margin_y', DEFAULT_MARGIN_M)),
        'surface_z': surface_z,
        'is_calibrated': bool(raw.get('is_calibrated', False)),
        'plane_calibrated': bool(raw.get('plane_calibrated', False)),
        'plane_a': float(raw.get('plane_a', 0.0)),
        'plane_b': float(raw.get('plane_b', 0.0)),
        'plane_c': float(raw.get('plane_c', surface_z)),
        'probe_points': [
            {
                'x': float(p.get('x', 0.0)),
                'y': float(p.get('y', 0.0)),
                'z': float(p.get('z', 0.0)),
            }
            for p in raw.get('probe_points', [])
        ],
    }


def capture_corner_pose(panel):
    """Capture wire_tip TF as the plate corner position."""
    try:
        tf = panel._tf_buffer.lookup_transform(
            'base_link', 'wire_tip', rclpy.time.Time())
        t = tf.transform.translation
        panel._plate_ox.setValue(float(t.x))
        panel._plate_oy.setValue(float(t.y))
        if hasattr(panel, '_plate_oz'):
            panel._plate_oz.setValue(float(t.z))
    except Exception as e:
        QMessageBox.warning(panel._widget, 'Capture Error', str(e))


def calibrate_selected_plate(panel):
    rows = sorted(set(idx.row() for idx in panel._plate_table.selectedIndexes()))
    if not rows:
        QMessageBox.warning(panel._widget, 'No Selection', 'Select one plate row first.')
        return

    col = 1 if panel._plate_table.columnCount() == 11 else 0
    pid = panel._plate_table.item(rows[0], col).text()
    plate = next((p for p in panel._plates if p['plate_id'] == pid), None)
    if plate is None:
        QMessageBox.warning(panel._widget, 'Error', f'Plate not found: {pid}')
        return

    req = CalibratePlatePlane.Request()
    req.plate_id = plate['plate_id']
    req.corner_x = float(plate['corner_x'])
    req.corner_y = float(plate['corner_y'])
    req.corner_z = float(plate['corner_z'])
    req.corner_id = str(plate.get('corner_id', 'front_left'))
    req.width = float(plate['width'])
    req.length = float(plate['length'])
    req.yaw = float(math.radians(plate['yaw_deg']))
    req.margin_x = float(plate.get('margin_x', panel._global_margin_x.value()))
    req.margin_y = float(plate.get('margin_y', panel._global_margin_y.value()))
    req.probe_speed = float(panel._probe_speed.value())
    # z_start_offset / z_probe_depth intentionally left at 0.0 — server ignores them
    # and auto-computes safe heights from corner_z.

    panel._plate_cal_result.setText('Result: Calibrating...')

    def _done(future):
        res = future.result()
        if res is None:
            panel._plate_cal_result.setText('Result: Service call failed')
            return
        if not res.success:
            panel._plate_cal_result.setText(f'Result: FAILED - {res.message}')
            return

        plate['surface_z'] = float(res.surface_z)
        plate['is_calibrated'] = True
        plate['plane_calibrated'] = True
        plate['plane_a'] = float(res.plane_a)
        plate['plane_b'] = float(res.plane_b)
        plate['plane_c'] = float(res.plane_c)
        plate['probe_points'] = [
            {'x': float(p.x), 'y': float(p.y), 'z': float(p.z)}
            for p in res.probe_points
        ]
        auto_save_plates(panel)
        refresh_plate_table(panel)
        panel._plate_cal_result.setText(
            f"Result: OK z={res.plane_a:.4f}x+{res.plane_b:.4f}y+{res.plane_c:.4f}")

    started = panel._calibration_ctrl.calibrate_plate_plane(req, _done)
    if not started:
        panel._plate_cal_result.setText('Result: Service unavailable')
