"""Live laser profile plot widget for PointCloud2 snapshots.

Both Y and Z axes are fixed to the values loaded from
``share/robin_rqt/config/laser_profile.yaml`` (editable at runtime via the
"Reload Config" button in the Sensor tab).  This means the profile is always
drawn at the same spatial position — data outside the configured window is
clamped so nothing escapes the box.

Equal aspect ratio: 1 mm spans the same pixel count on both axes.
Parsing uses numpy strided reads (~0.1 ms at 1200 pts vs ~4 ms for a struct loop).
Repaints are capped at 30 fps.
"""

import os
import time

import numpy as np
import yaml

from python_qt_binding.QtCore import QPointF
from python_qt_binding.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from python_qt_binding.QtWidgets import QWidget

# Throttle Qt repaint calls to this interval (30 fps max)
_PAINT_INTERVAL_S = 1.0 / 30.0

# Default axis range used when the config file cannot be found
_DEFAULTS = dict(y_min_mm=-35.0, y_max_mm=35.0, z_min_mm=-5.0, z_max_mm=15.0)


def _find_config_path() -> str:
    """Return the path to laser_profile.yaml from the installed package share.

    Falls back to the source-tree ``config/`` sibling of this file so that
    the widget also works when run directly from the source checkout without
    a colcon install step.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("robin_rqt"), "config", "laser_profile.yaml"
        )
    except Exception:
        pass
    # Fallback: <package_root>/config/laser_profile.yaml
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "..", "config", "laser_profile.yaml")


def _load_config(path: str) -> dict:
    """Read YAML config.  Returns defaults on any error."""
    cfg = dict(_DEFAULTS)
    try:
        with open(path) as fh:
            data = yaml.safe_load(fh) or {}
        for key in _DEFAULTS:
            if key in data:
                cfg[key] = float(data[key])
    except Exception:
        pass
    return cfg


class LaserProfileWidget(QWidget):
    """2D YZ cross-section plot with fixed, configurable axis ranges.

    Horizontal axis: Y [mm]  — fixed range loaded from laser_profile.yaml (default ±35 mm).
    Vertical   axis: Z [mm]  — fixed range loaded from laser_profile.yaml (default −5…+15 mm).
    Aspect ratio is always 1 : 1 (same mm/pixel on both axes).
    A ghost reference line (amber) can be captured and drawn behind the live
    profile (blue).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(260)
        self._config_path = _find_config_path()
        self._h_values: list = []
        self._z_values: list = []
        self._ref_h_values: list = []
        self._ref_z_values: list = []
        self._point_count: int = 0
        self._last_paint_time: float = 0.0
        self._max_points: int = 1200
        # Axis extents — populated by _apply_config()
        self.Y_MIN_MM: float = _DEFAULTS["y_min_mm"]
        self.Y_MAX_MM: float = _DEFAULTS["y_max_mm"]
        self.Z_MIN_MM: float = _DEFAULTS["z_min_mm"]
        self.Z_MAX_MM: float = _DEFAULTS["z_max_mm"]
        self._apply_config(_load_config(self._config_path))

    # ── public API ────────────────────────────────────────────────────

    def reload_config(self):
        """Re-read laser_profile.yaml and redraw. Safe to call from any thread."""
        self._apply_config(_load_config(self._config_path))
        self.update()

    def config_path(self) -> str:
        return self._config_path

    def clear(self):
        self._h_values = []
        self._z_values = []
        self._point_count = 0
        self.update()

    def set_reference(self):
        """Snapshot the current live profile as the amber ghost reference."""
        self._ref_h_values = list(self._h_values)
        self._ref_z_values = list(self._z_values)
        self.update()

    def clear_reference(self):
        """Remove the ghost reference overlay."""
        self._ref_h_values = []
        self._ref_z_values = []
        self.update()

    def update_from_pointcloud(self, cloud_msg):
        parsed = self._extract_profile(cloud_msg)
        if parsed is None:
            self.clear()
            return
        self._h_values, self._z_values = parsed
        self._point_count = len(self._h_values)
        now = time.monotonic()
        if now - self._last_paint_time >= _PAINT_INTERVAL_S:
            self._last_paint_time = now
            self.update()

    # ── internals ─────────────────────────────────────────────────────

    def _apply_config(self, cfg: dict):
        self.Y_MIN_MM = cfg["y_min_mm"]
        self.Y_MAX_MM = cfg["y_max_mm"]
        self.Z_MIN_MM = cfg["z_min_mm"]
        self.Z_MAX_MM = cfg["z_max_mm"]
        self._Y_RANGE_MM = max(self.Y_MAX_MM - self.Y_MIN_MM, 1.0)
        self._Z_RANGE_MM = max(self.Z_MAX_MM - self.Z_MIN_MM, 1.0)

    # ── numpy parsing ─────────────────────────────────────────────────

    def _extract_profile(self, cloud_msg):
        fields = {f.name: f for f in cloud_msg.fields}
        if "y" not in fields or "z" not in fields:
            return None

        step = int(cloud_msg.point_step)
        if step < 8:
            return None

        y_off = int(fields["y"].offset)
        z_off = int(fields["z"].offset)
        n = int(cloud_msg.width) * int(cloud_msg.height)
        raw = bytes(cloud_msg.data)
        n = min(n, len(raw) // step)
        if n <= 0:
            return None

        stride = max(1, n // self._max_points)
        dt = np.dtype("<f4") if not cloud_msg.is_bigendian else np.dtype(">f4")
        try:
            row_arr = np.frombuffer(raw[:n * step], dtype=np.uint8).reshape(n, step)
            if stride > 1:
                row_arr = row_arr[::stride]
            y_vals = row_arr[:, y_off:y_off + 4].copy().view(dt).ravel() * 1000.0
            z_vals = row_arr[:, z_off:z_off + 4].copy().view(dt).ravel() * 1000.0
        except Exception:
            return None

        mask = np.isfinite(y_vals) & np.isfinite(z_vals)
        y_vals = y_vals[mask]
        z_vals = z_vals[mask]
        if len(y_vals) == 0:
            return None

        order = np.argsort(y_vals)
        return y_vals[order].tolist(), z_vals[order].tolist()

    # ── painting ──────────────────────────────────────────────────────

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect()
        palette = self.palette()
        bg = palette.color(palette.Window)
        fg = palette.color(palette.WindowText)
        grid_col = palette.color(palette.Mid)
        muted = palette.color(palette.Midlight)
        live_col = palette.color(palette.Highlight)
        ref_col = QColor(210, 140, 30, 200)  # amber ghost
        painter.fillRect(rect, bg)

        # Fixed outer margins
        margin_l, margin_r, margin_t, margin_b = 56, 14, 28, 36
        avail_w = float(rect.width() - margin_l - margin_r)
        avail_h = float(rect.height() - margin_t - margin_b)

        # Title
        title_font = QFont()
        title_font.setPointSize(9)
        title_font.setBold(True)
        painter.setFont(title_font)
        painter.setPen(fg)
        ref_tag = "  [REF]" if self._ref_h_values else ""
        painter.drawText(
            8, 18,
            f"Laser Profile (YZ)  pts={self._point_count}"
            f"  Y:[{self.Y_MIN_MM:.0f},{self.Y_MAX_MM:.0f}]"
            f"  Z:[{self.Z_MIN_MM:.0f},{self.Z_MAX_MM:.0f}] mm{ref_tag}")

        # ── equal-aspect layout using fixed axis ranges ───────────────
        ppm = min(avail_w / self._Y_RANGE_MM, avail_h / self._Z_RANGE_MM)
        plot_w = ppm * self._Y_RANGE_MM
        plot_h = ppm * self._Z_RANGE_MM
        x0 = float(margin_l) + (avail_w - plot_w) / 2.0
        y0 = float(margin_t) + (avail_h - plot_h) / 2.0

        # Axes box
        painter.setPen(QPen(grid_col, 1))
        painter.drawRect(int(x0), int(y0), int(plot_w), int(plot_h))

        # Draw zero-lines inside the plot box (dashed, subtle)
        zero_pen = QPen(muted, 1)
        zero_pen.setStyle(0x4)  # Qt::DashLine
        painter.setPen(zero_pen)
        if self.Y_MIN_MM < 0.0 < self.Y_MAX_MM:
            zx = x0 + (-self.Y_MIN_MM) / self._Y_RANGE_MM * plot_w
            painter.drawLine(int(zx), int(y0), int(zx), int(y0 + plot_h))
        if self.Z_MIN_MM < 0.0 < self.Z_MAX_MM:
            zy = y0 + (1.0 - (0.0 - self.Z_MIN_MM) / self._Z_RANGE_MM) * plot_h
            painter.drawLine(int(x0), int(zy), int(x0 + plot_w), int(zy))

        # Axis labels
        small_font = QFont()
        small_font.setPointSize(8)
        painter.setFont(small_font)
        painter.setPen(fg)
        painter.drawText(int(x0 - 44), int(y0 + 14), "Z [mm]")
        painter.drawText(int(x0 + plot_w / 2 - 10), int(y0 + plot_h + 22), "Y [mm]")

        # Tick labels (fixed from config)
        painter.setPen(muted)
        painter.drawText(int(x0 - 48), int(y0 + 10),         f"{self.Z_MAX_MM:.0f}")
        painter.drawText(int(x0 - 48), int(y0 + plot_h),     f"{self.Z_MIN_MM:.0f}")
        painter.drawText(int(x0),                             int(y0 + plot_h + 18), f"{self.Y_MIN_MM:.0f}")
        painter.drawText(int(x0 + plot_w - 28),               int(y0 + plot_h + 18), f"{self.Y_MAX_MM:.0f}")

        if not self._h_values:
            painter.setPen(muted)
            painter.drawText(int(x0 + 8), int(y0 + plot_h / 2), "No pointcloud data")
            return

        # Helper: world mm → widget pixel, clamped to viewport
        def to_poly(h_list, z_list):
            pts = []
            for h, z in zip(h_list, z_list):
                h_c = max(self.Y_MIN_MM, min(self.Y_MAX_MM, h))
                z_c = max(self.Z_MIN_MM, min(self.Z_MAX_MM, z))
                px = x0 + (h_c - self.Y_MIN_MM) / self._Y_RANGE_MM * plot_w
                py = y0 + (1.0 - (z_c - self.Z_MIN_MM) / self._Z_RANGE_MM) * plot_h
                pts.append(QPointF(px, py))
            return QPolygonF(pts)

        # Ghost reference (amber, thin)
        if self._ref_h_values and self._ref_z_values:
            painter.setPen(QPen(ref_col, 1))
            ref_poly = to_poly(self._ref_h_values, self._ref_z_values)
            if len(ref_poly) > 1:
                painter.drawPolyline(ref_poly)

        # Live profile (highlight, thick)
        painter.setPen(QPen(live_col, 2))
        live_poly = to_poly(self._h_values, self._z_values)
        if len(live_poly) > 1:
            painter.drawPolyline(live_poly)

