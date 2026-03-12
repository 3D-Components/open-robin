"""Lightweight live process plots for experiment monitoring.

Plots current, voltage, and wire feed speed against normalized progression.
"""

from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPainter, QPen, QColor, QFont


class ProcessPlotWidget(QWidget):
    """Custom Qt painter-based 3-panel plot widget.

    X-axis: normalized progression [0, 1]
    Y-axes: per-signal auto-scaled independently.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(280)
        self._bead_id = ""
        self._prog = []
        self._current = []
        self._voltage = []
        self._wfs = []
        self._max_points = 1200

    def clear(self):
        self._prog.clear()
        self._current.clear()
        self._voltage.clear()
        self._wfs.clear()
        self.update()

    def add_sample(self, bead_id: str, progression: float,
                   current: float, voltage: float, wfs: float):
        p = max(0.0, min(1.0, float(progression)))
        bid = bead_id or ""

        # New bead or progression reset => clear traces for readability.
        if self._bead_id and bid and bid != self._bead_id:
            self.clear()
        elif self._prog and p < self._prog[-1] - 0.05:
            self.clear()

        self._bead_id = bid
        self._prog.append(p)
        self._current.append(float(current))
        self._voltage.append(float(voltage))
        self._wfs.append(float(wfs))

        if len(self._prog) > self._max_points:
            trim = len(self._prog) - self._max_points
            self._prog = self._prog[trim:]
            self._current = self._current[trim:]
            self._voltage = self._voltage[trim:]
            self._wfs = self._wfs[trim:]

        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect()
        palette = self.palette()
        bg = palette.color(palette.Window)
        fg = palette.color(palette.WindowText)
        grid = palette.color(palette.Mid)
        muted = palette.color(palette.Midlight)
        painter.fillRect(rect, bg)

        margin_l = 60
        margin_r = 12
        margin_t = 22
        margin_b = 22
        plot_w = max(10, rect.width() - margin_l - margin_r)
        plot_h_total = max(30, rect.height() - margin_t - margin_b)
        panel_h = plot_h_total / 3.0

        title_font = QFont()
        title_font.setPointSize(9)
        title_font.setBold(True)
        painter.setFont(title_font)
        painter.setPen(fg)
        bead_txt = self._bead_id if self._bead_id else "-"
        painter.drawText(8, 14, f"Bead: {bead_txt} | X = normalized progression")

        self._draw_panel(
            painter, margin_l, margin_t + 0 * panel_h, plot_w, panel_h,
            self._prog, self._current,
            label="Current [A]", color=palette.color(palette.Highlight),
            fg=fg, grid=grid, muted=muted,
        )
        self._draw_panel(
            painter, margin_l, margin_t + 1 * panel_h, plot_w, panel_h,
            self._prog, self._voltage,
            label="Voltage [V]", color=palette.color(palette.Link),
            fg=fg, grid=grid, muted=muted,
        )
        self._draw_panel(
            painter, margin_l, margin_t + 2 * panel_h, plot_w, panel_h,
            self._prog, self._wfs,
            label="Wire Feed [m/min]", color=palette.color(palette.Text),
            fg=fg, grid=grid, muted=muted,
            draw_x_axis=True,
        )

    def _draw_panel(self, painter, x0, y0, w, h, xs, ys, label, color,
                    fg, grid, muted, draw_x_axis=False):
        x0 = float(x0)
        y0 = float(y0)
        w = float(w)
        h = float(h)
        pad_top = 8.0
        pad_bot = 18.0 if draw_x_axis else 8.0
        pad_inner = 8.0

        px0 = x0
        py0 = y0 + pad_top
        pw = max(2.0, w)
        ph = max(2.0, h - pad_top - pad_bot)

        painter.setPen(QPen(grid, 1))
        painter.drawRect(int(px0), int(py0), int(pw), int(ph))

        painter.setPen(fg)
        painter.drawText(int(px0 - 54), int(py0 + 12), label)

        if not xs or not ys:
            painter.setPen(muted)
            painter.drawText(int(px0 + 8), int(py0 + ph / 2), "No data")
            return

        y_min = min(ys)
        y_max = max(ys)
        if abs(y_max - y_min) < 1e-6:
            y_min -= 0.5
            y_max += 0.5

        painter.setPen(muted)
        painter.drawText(int(px0 - 54), int(py0 + 24), f"{y_max:.1f}")
        painter.drawText(int(px0 - 54), int(py0 + ph), f"{y_min:.1f}")

        painter.setPen(QPen(color, 2))
        last = None
        for x_val, y_val in zip(xs, ys):
            xx = px0 + pad_inner + max(0.0, min(1.0, float(x_val))) * (pw - 2 * pad_inner)
            yy = py0 + (1.0 - (float(y_val) - y_min) / (y_max - y_min)) * ph
            if last is not None:
                painter.drawLine(int(last[0]), int(last[1]), int(xx), int(yy))
            last = (xx, yy)

        if draw_x_axis:
            painter.setPen(muted)
            painter.drawText(int(px0 + pad_inner), int(py0 + ph + 14), "0.0")
            painter.drawText(int(px0 + pw / 2 - 10), int(py0 + ph + 14), "0.5")
            painter.drawText(int(px0 + pw - pad_inner - 16), int(py0 + ph + 14), "1.0")
