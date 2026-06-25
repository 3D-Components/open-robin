"""
Matplotlib-based 4×2 subplot grid for live welding process data.

All X-axes show normalised progression [0, 1].
Redraws are debounced via QTimer (200 ms) to avoid thrashing at 10 Hz.
"""

from collections import defaultdict

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QVBoxLayout, QWidget

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

# Series definitions: key → (row, col, title, y-label)
_SERIES = [
    ('wfs',       0, 0, 'Wire Feed Speed', 'm/min'),
    ('current',   0, 1, 'Current',         'A'),
    ('voltage',   1, 0, 'Voltage',         'V'),
    ('energy',    1, 1, 'Energy',          'J'),
    ('power',     2, 0, 'Power',           'W'),
    ('width',     2, 1, 'Bead Width',      'mm'),
    ('height',    3, 0, 'Bead Height',     'mm'),
    ('toe_angle', 3, 1, 'Toe Angle',       'rad'),
]

_ROWS = 4
_COLS = 2
_DEBOUNCE_MS = 200


class PlotGridWidget(QWidget):
    """7-panel matplotlib canvas with debounced redraw."""

    def __init__(self, parent=None):
        super().__init__(parent)

        # --- data stores --------------------------------------------------
        self._xs: dict[str, list[float]] = defaultdict(list)
        self._ys: dict[str, list[float]] = defaultdict(list)

        # --- matplotlib figure --------------------------------------------
        self._fig = Figure(tight_layout=True)
        self._fig.set_facecolor('white')
        self._canvas = FigureCanvasQTAgg(self._fig)

        self._axes: dict[str, matplotlib.axes.Axes] = {}
        self._lines: dict[str, matplotlib.lines.Line2D] = {}

        for key, row, col, title, ylabel in _SERIES:
            ax = self._fig.add_subplot(_ROWS, _COLS, row * _COLS + col + 1)
            ax.set_title(title, fontsize=8, pad=2)
            ax.set_ylabel(ylabel, fontsize=7)
            ax.set_xlim(0.0, 1.0)
            ax.tick_params(labelsize=6)
            ax.grid(True, linewidth=0.3, alpha=0.6)
            (line,) = ax.plot([], [], linewidth=1.0)
            self._axes[key] = ax
            self._lines[key] = line

        self._fig.tight_layout(pad=1.0, h_pad=0.8, w_pad=0.6)

        # --- layout -------------------------------------------------------
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._canvas)

        # --- debounce timer -----------------------------------------------
        self._dirty = False
        self._timer = QTimer(self)
        self._timer.setInterval(_DEBOUNCE_MS)
        self._timer.timeout.connect(self._redraw)
        self._timer.start()

    # ----- public API ------------------------------------------------------

    def add_point(self, key: str, x: float, y: float) -> None:
        """Append a data point to *key* series; actual redraw is debounced."""
        if key not in self._axes:
            return
        self._xs[key].append(x)
        self._ys[key].append(y)
        self._dirty = True

    def clear_all(self) -> None:
        """Reset every series."""
        for key in list(self._axes):
            self.clear_series(key)

    def clear_series(self, key: str) -> None:
        """Reset a single series."""
        self._xs[key].clear()
        self._ys[key].clear()
        if key in self._lines:
            self._lines[key].set_data([], [])
            self._axes[key].relim()
            self._axes[key].autoscale_view(scaley=True, scalex=False)
        self._dirty = True

    # ----- internal --------------------------------------------------------

    def _redraw(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        for key, line in self._lines.items():
            xs = self._xs[key]
            ys = self._ys[key]
            if xs:
                line.set_data(xs, ys)
                ax = self._axes[key]
                ax.relim()
                ax.autoscale_view(scaley=True, scalex=False)
        self._canvas.draw_idle()
