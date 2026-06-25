"""
Virtual joystick widgets for touch-screen robot jog control.

JoystickPad — 2-axis circular pad with draggable thumb (XY or RxRy).
SingleAxisPad — 1-axis pad, vertical (Z) or horizontal (Rz).

All pads emit normalised [-1, 1] values.  Displacement is proportional:
the further from centre, the higher the output magnitude.  A 5 % dead-zone
prevents unintentional drift from imprecise touch.
"""

import math

from python_qt_binding.QtCore import Qt, Signal, QPointF, QRectF
from python_qt_binding.QtGui import QPainter, QPen, QBrush, QColor, QRadialGradient
from python_qt_binding.QtWidgets import QWidget, QSizePolicy


_DEAD_ZONE = 0.05          # fraction of radius — ignore displacement < 5 %
_BG_COLOR = QColor('#3A3A3A')
_BORDER_COLOR = QColor('#666666')
_GUIDE_COLOR = QColor('#555555')
_THUMB_IDLE = QColor('#90A4AE')
_THUMB_ACTIVE = QColor('#FFA726')


class JoystickPad(QWidget):
    """Circular 2-axis virtual joystick.

    Emits *positionChanged(x, y)* with values in [-1, 1].
    (0, 0) means centred / released.
    """

    positionChanged = Signal(float, float)

    def __init__(self, size: int = 120, parent=None):
        super().__init__(parent)
        self.setFixedSize(size, size)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self._pressed = False
        self._nx = 0.0      # normalised output X
        self._ny = 0.0      # normalised output Y

    # -- geometry helpers --------------------------------------------------

    def _center(self) -> QPointF:
        return QPointF(self.width() / 2.0, self.height() / 2.0)

    def _radius(self) -> float:
        return min(self.width(), self.height()) / 2.0 - 2

    def _thumb_radius(self) -> float:
        return self._radius() * 0.28

    # -- painting ----------------------------------------------------------

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c = self._center()
        r = self._radius()

        # Background circle
        p.setPen(QPen(_BORDER_COLOR, 1.5))
        p.setBrush(QBrush(_BG_COLOR))
        p.drawEllipse(c, r, r)

        # Cross-hair guides
        p.setPen(QPen(_GUIDE_COLOR, 1, Qt.DashLine))
        p.drawLine(QPointF(c.x() - r, c.y()), QPointF(c.x() + r, c.y()))
        p.drawLine(QPointF(c.x(), c.y() - r), QPointF(c.x(), c.y() + r))

        # Thumb
        tx = c.x() + self._nx * r
        ty = c.y() - self._ny * r        # Qt Y is inverted
        tr = self._thumb_radius()
        color = _THUMB_ACTIVE if self._pressed else _THUMB_IDLE
        grad = QRadialGradient(tx, ty, tr)
        grad.setColorAt(0, color.lighter(130))
        grad.setColorAt(1, color)
        p.setPen(QPen(color.darker(140), 1))
        p.setBrush(QBrush(grad))
        p.drawEllipse(QPointF(tx, ty), tr, tr)

        p.end()

    # -- mouse interaction -------------------------------------------------

    def mousePressEvent(self, event):
        self._pressed = True
        self._update_from_mouse(event)

    def mouseMoveEvent(self, event):
        if self._pressed:
            self._update_from_mouse(event)

    def mouseReleaseEvent(self, _event):
        self._pressed = False
        self._nx = 0.0
        self._ny = 0.0
        self.update()
        self.positionChanged.emit(0.0, 0.0)

    def _update_from_mouse(self, event):
        c = self._center()
        r = self._radius()
        if hasattr(event, 'position'):
            pos = event.position()
        else:
            pos = event.pos()
        dx = float(pos.x()) - c.x()
        dy = -(float(pos.y()) - c.y())     # invert Y so up = +
        dist = math.hypot(dx, dy)
        if dist > r:
            dx *= r / dist
            dy *= r / dist
            dist = r
        nx = dx / r
        ny = dy / r
        mag = math.hypot(nx, ny)
        if mag < _DEAD_ZONE:
            nx = 0.0
            ny = 0.0
        self._nx = nx
        self._ny = ny
        self.update()
        self.positionChanged.emit(self._nx, self._ny)


class SingleAxisPad(QWidget):
    """Single-axis virtual joystick (vertical or horizontal).

    Emits *valueChanged(v)* with v in [-1, 1].
    ``orientation=Qt.Vertical``  →  up = +1, down = -1
    ``orientation=Qt.Horizontal``→  right = +1, left = -1
    """

    valueChanged = Signal(float)

    def __init__(self, orientation=Qt.Vertical, length: int = 120,
                 thickness: int = 40, parent=None):
        super().__init__(parent)
        self._orient = orientation
        if orientation == Qt.Vertical:
            self.setFixedSize(thickness, length)
        else:
            self.setFixedSize(length, thickness)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self._pressed = False
        self._nv = 0.0       # normalised output

    # -- geometry helpers --------------------------------------------------

    def _half_travel(self) -> float:
        if self._orient == Qt.Vertical:
            return self.height() / 2.0 - 2
        return self.width() / 2.0 - 2

    def _thumb_radius(self) -> float:
        if self._orient == Qt.Vertical:
            return min(self.width(), 28) / 2.0
        return min(self.height(), 28) / 2.0

    # -- painting ----------------------------------------------------------

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # Track (rounded rect)
        p.setPen(QPen(_BORDER_COLOR, 1.5))
        p.setBrush(QBrush(_BG_COLOR))
        rr = min(w, h) / 2.0
        p.drawRoundedRect(QRectF(1, 1, w - 2, h - 2), rr, rr)

        # Centre guide line
        p.setPen(QPen(_GUIDE_COLOR, 1, Qt.DashLine))
        if self._orient == Qt.Vertical:
            p.drawLine(QPointF(w / 2, 4), QPointF(w / 2, h - 4))
        else:
            p.drawLine(QPointF(4, h / 2), QPointF(w - 4, h / 2))

        # Thumb
        ht = self._half_travel()
        tr = self._thumb_radius()
        if self._orient == Qt.Vertical:
            tx = w / 2.0
            ty = h / 2.0 - self._nv * ht   # invert: up = +
        else:
            tx = w / 2.0 + self._nv * ht
            ty = h / 2.0
        color = _THUMB_ACTIVE if self._pressed else _THUMB_IDLE
        grad = QRadialGradient(tx, ty, tr)
        grad.setColorAt(0, color.lighter(130))
        grad.setColorAt(1, color)
        p.setPen(QPen(color.darker(140), 1))
        p.setBrush(QBrush(grad))
        p.drawEllipse(QPointF(tx, ty), tr, tr)

        p.end()

    # -- mouse interaction -------------------------------------------------

    def mousePressEvent(self, event):
        self._pressed = True
        self._update_from_mouse(event)

    def mouseMoveEvent(self, event):
        if self._pressed:
            self._update_from_mouse(event)

    def mouseReleaseEvent(self, _event):
        self._pressed = False
        self._nv = 0.0
        self.update()
        self.valueChanged.emit(0.0)

    def _update_from_mouse(self, event):
        if hasattr(event, 'position'):
            pos = event.position()
        else:
            pos = event.pos()
        ht = self._half_travel()
        if self._orient == Qt.Vertical:
            raw = -(float(pos.y()) - self.height() / 2.0)   # up = +
        else:
            raw = float(pos.x()) - self.width() / 2.0
        nv = max(-1.0, min(1.0, raw / ht)) if ht > 0 else 0.0
        if abs(nv) < _DEAD_ZONE:
            nv = 0.0
        self._nv = nv
        self.update()
        self.valueChanged.emit(self._nv)
