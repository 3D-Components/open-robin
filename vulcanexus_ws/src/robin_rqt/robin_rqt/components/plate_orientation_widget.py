"""
Rotating plate orientation widget for the Setup tab.

Draws a top-down plate rectangle that the operator drags to set yaw.

Operator-view convention (matches RViz top-down default):
  +X axis → left   (robot forward / rear direction)
  +Y axis → down   (robot left / right direction)

Dragging rotates the plate; snap to 15° increments.
"""

import math

from python_qt_binding.QtCore import Qt, Signal, QPointF
from python_qt_binding.QtGui import (
    QPainter, QPen, QBrush, QColor, QPolygonF, QFont, QFontMetrics,
)
from python_qt_binding.QtWidgets import QWidget, QSizePolicy


_SNAP_DEG = 15

# Plate colors
_PLATE_FILL = QColor('#ECEFF1')
_PLATE_BORDER = QColor('#607D8B')
_AXIS_X = QColor('#E53935')
_AXIS_Y = QColor('#43A047')
_AXIS_Z = QColor('#1E88E5')
_BG = QColor('#2B2B2B')
_MARGIN_COLOR = QColor(255, 152, 0, 90)  # translucent orange


class RotatingPlateWidget(QWidget):
    """Interactive plate orientation widget with draggable rotation.

    Signals
    -------
    yawChanged(float)
        Emitted when the yaw angle (degrees) changes via drag.
    """

    yawChanged = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._yaw_deg = 0.0
        self._plate_w = 0.15      # m, synced from spinboxes
        self._plate_l = 0.20      # m
        self._margin_x = 0.04     # m
        self._margin_y = 0.04     # m
        self._dragging = False
        self._drag_start_angle = 0.0
        self._drag_start_yaw = 0.0
        self.setMinimumSize(100, 100)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    # -- public API --------------------------------------------------------

    def set_yaw(self, deg: float):
        self._yaw_deg = deg
        self.update()

    def set_plate_dims(self, w: float, l: float):
        self._plate_w = max(w, 0.01)
        self._plate_l = max(l, 0.01)
        self.update()

    def set_margins(self, mx: float, my: float):
        self._margin_x = mx
        self._margin_y = my
        self.update()

    # -- geometry ----------------------------------------------------------

    def _center(self) -> QPointF:
        return QPointF(self.width() / 2.0, self.height() / 2.0)

    def _scale(self) -> float:
        """Pixels per metre — fit plate with margin inside widget."""
        max_dim = max(self._plate_w, self._plate_l)
        avail = min(self.width(), self.height()) - 30
        return avail / max_dim if max_dim > 0 else 1.0

    def _plate_corners_local(self):
        """Return plate corners in local coords (m) relative to plate center.

        The plate center is at (L/2, W/2) from front_left corner.
        Corner layout (before rotation):
            front_left  = (-L/2, -W/2)    front_right = (-L/2, +W/2)
            rear_left   = (+L/2, -W/2)    rear_right  = (+L/2, +W/2)

        Where X_local = along plate length, Y_local = along plate width.
        """
        hw = self._plate_w / 2.0
        hl = self._plate_l / 2.0
        return {
            'front_left':  (-hl, -hw),
            'front_right': (-hl, +hw),
            'rear_left':   (+hl, -hw),
            'rear_right':  (+hl, +hw),
        }

    def _local_to_screen(self, lx: float, ly: float) -> QPointF:
        """Transform local plate coords (m) to screen coords.

        Operator view: +X_world → screen left, +Y_world → screen down.
        So screen_x = center_x - lx * scale, screen_y = center_y + ly * scale.
        Rotation applied about plate center.
        """
        s = self._scale()
        yaw = math.radians(self._yaw_deg)
        c, sn = math.cos(yaw), math.sin(yaw)
        # Rotate local coords by yaw
        rx = lx * c - ly * sn
        ry = lx * sn + ly * c
        # Map to screen: +X→left, +Y→down
        cx = self._center()
        return QPointF(cx.x() - rx * s, cx.y() + ry * s)

    def _screen_to_angle(self, pos) -> float:
        """Angle (deg) from widget center to screen point, in plate convention."""
        cx = self._center()
        # Screen to world direction: dx_screen→left = +X, dy_screen→down = +Y
        dx = -(pos.x() - cx.x())
        dy = pos.y() - cx.y()
        return math.degrees(math.atan2(dy, dx))

    def _margin_corners_local(self):
        """Return margin-inset rectangle corners in local coords."""
        hw = self._plate_w / 2.0
        hl = self._plate_l / 2.0
        mx = self._margin_x
        my = self._margin_y
        return [
            (-hl + mx, -hw + my),
            (-hl + mx, +hw - my),
            (+hl - mx, +hw - my),
            (+hl - mx, -hw + my),
        ]

    # -- painting ----------------------------------------------------------

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # Background
        p.fillRect(self.rect(), QBrush(_BG))

        # Axis indicator — top-right area
        ax_x = self.width() - 12
        ax_y = 14
        ax_len = 22
        p.setPen(QPen(_AXIS_X, 2))
        p.drawLine(QPointF(ax_x, ax_y), QPointF(ax_x - ax_len, ax_y))
        p.setPen(QPen(_AXIS_Y, 2))
        p.drawLine(QPointF(ax_x, ax_y), QPointF(ax_x, ax_y + ax_len))
        small_font = QFont()
        small_font.setPixelSize(9)
        small_font.setBold(True)
        p.setFont(small_font)
        p.setPen(_AXIS_X)
        p.drawText(QPointF(ax_x - ax_len - 12, ax_y + 4), '+X')
        p.setPen(_AXIS_Y)
        p.drawText(QPointF(ax_x - 8, ax_y + ax_len + 11), '+Y')

        corners = self._plate_corners_local()
        screen = {k: self._local_to_screen(*v) for k, v in corners.items()}

        # Plate fill
        order = ['front_left', 'front_right', 'rear_right', 'rear_left']
        poly = QPolygonF([screen[k] for k in order])
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(_PLATE_FILL))
        p.drawPolygon(poly)

        # Margin inset rectangle (if margins > 0)
        if self._margin_x > 0 or self._margin_y > 0:
            m_pts = self._margin_corners_local()
            m_screen = [self._local_to_screen(*pt) for pt in m_pts]
            m_poly = QPolygonF(m_screen)
            p.setPen(QPen(QColor('#FF9800'), 1, Qt.DashLine))
            p.setBrush(QBrush(_MARGIN_COLOR))
            p.drawPolygon(m_poly)

        # Plate outline
        p.setPen(QPen(_PLATE_BORDER, 2))
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(poly)

        # Single reference corner (front_left) — red
        fl = screen['front_left']
        _RED = QColor('#E53935')
        p.setPen(QPen(_RED.darker(130), 1.5))
        p.setBrush(QBrush(_RED))
        p.drawEllipse(fl, 6, 6)

        # Yaw readout at bottom-center
        p.setPen(QPen(QColor('#AAAAAA'), 1))
        yaw_font = QFont()
        yaw_font.setPixelSize(10)
        p.setFont(yaw_font)
        yaw_text = f'Yaw: {self._yaw_deg:.0f}°'
        ytw = QFontMetrics(yaw_font).horizontalAdvance(yaw_text)
        p.drawText(QPointF((self.width() - ytw) / 2, self.height() - 4), yaw_text)

        p.end()

    # -- mouse interaction -------------------------------------------------

    def mousePressEvent(self, event):
        pos = event.position() if hasattr(event, 'position') else event.pos()
        pos = QPointF(float(pos.x()), float(pos.y()))

        # Start rotation drag
        self._dragging = True
        self._drag_start_angle = self._screen_to_angle(pos)
        self._drag_start_yaw = self._yaw_deg

    def mouseMoveEvent(self, event):
        if not self._dragging:
            return
        pos = event.position() if hasattr(event, 'position') else event.pos()
        pos = QPointF(float(pos.x()), float(pos.y()))
        current_angle = self._screen_to_angle(pos)
        delta = current_angle - self._drag_start_angle
        raw_yaw = self._drag_start_yaw + delta
        # Snap to 15° increments
        snapped = round(raw_yaw / _SNAP_DEG) * _SNAP_DEG
        # Normalize to [-180, 180]
        while snapped > 180:
            snapped -= 360
        while snapped <= -180:
            snapped += 360
        if snapped != self._yaw_deg:
            self._yaw_deg = snapped
            self.update()
            self.yawChanged.emit(self._yaw_deg)

    def mouseReleaseEvent(self, _event):
        self._dragging = False
