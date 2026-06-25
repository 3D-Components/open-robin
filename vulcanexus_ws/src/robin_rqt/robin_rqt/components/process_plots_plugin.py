"""
ROBIN Process Plots — RQT plugin showing live welding signals vs progression.

Eight sub-plots in a 4×2 grid:
  Wire Feed Speed (m/min), Current (A), Voltage (V), Energy (J),
  Power (W), Bead Width (mm), Bead Height (mm), Toe Angle (rad)

Data sources:
  /robin/data/fronius    (WelderData)   — current, voltage, power, wfs, progression
  /robin/weld_dimensions (BeadGeometry) — height_mm, width_mm, toe_angle_rad, progression
"""

from qt_gui.plugin import Plugin

from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from robin_interfaces.msg import WelderData, BeadGeometry

from .plot_grid_widget import PlotGridWidget


class ProcessPlots(Plugin):
    """RQT plugin — 8-panel live welding process plots vs progression."""

    _sig_fronius = Signal(str, float, float, float, float, float, float)
    _sig_geometry = Signal(str, float, float, float, float)

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('ProcessPlots')

        self._node: Node = context.node

        # ── UI ────────────────────────────────────────────────────────
        self._widget = QWidget()
        self._widget.setWindowTitle('ROBIN Process Plots')
        layout = QVBoxLayout(self._widget)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        toolbar = QHBoxLayout()
        clear_btn = QPushButton('Clear All')
        clear_btn.setStyleSheet(
            'QPushButton { min-height: 32px; font-weight: bold; }')
        clear_btn.clicked.connect(self._clear_all)
        toolbar.addWidget(clear_btn)

        self._bead_label = QLabel('Bead: \u2014')
        self._bead_label.setStyleSheet(
            'font-weight: bold; font-size: 13px; padding: 0 8px;')
        toolbar.addWidget(self._bead_label)
        toolbar.addStretch()
        layout.addLayout(toolbar)

        self._plot_grid = PlotGridWidget()
        layout.addWidget(self._plot_grid)

        context.add_widget(self._widget)

        # ── ROS subscriptions ─────────────────────────────────────────
        self._subscriptions = []

        self._fronius_sub = self._node.create_subscription(
            WelderData, '/robin/data/fronius',
            self._on_fronius, qos_profile_sensor_data)
        self._subscriptions.append(self._fronius_sub)

        self._geometry_sub = self._node.create_subscription(
            BeadGeometry, '/robin/weld_dimensions',
            self._on_geometry, qos_profile_sensor_data)
        self._subscriptions.append(self._geometry_sub)

        # Thread-safe signals
        self._sig_fronius.connect(self._handle_fronius)
        self._sig_geometry.connect(self._handle_geometry)

        self._current_bead = ''
        self._last_progression = -1.0

    def shutdown_plugin(self):
        for sub in self._subscriptions:
            self._node.destroy_subscription(sub)
        self._subscriptions.clear()

    # ── ROS callbacks → Qt signals ────────────────────────────────────

    def _on_fronius(self, msg: WelderData):
        self._sig_fronius.emit(
            msg.bead_id,
            float(msg.progression),
            float(msg.current),
            float(msg.voltage),
            float(msg.wire_feed_speed),
            float(msg.power),
            float(msg.energy),
        )

    def _on_geometry(self, msg: BeadGeometry):
        self._sig_geometry.emit(
            msg.bead_id,
            float(msg.progression),
            float(msg.width_mm),
            float(msg.height_mm),
            float(msg.toe_angle_rad),
        )

    # ── Qt signal handlers ────────────────────────────────────────────

    def _handle_fronius(self, bead_id: str, progression: float,
                        current: float, voltage: float,
                        wfs: float, power: float, energy: float):
        if not bead_id:
            return

        # New bead → clear traces
        if self._current_bead and bead_id != self._current_bead:
            self._plot_grid.clear_all()
            self._last_progression = -1.0
        # Progression jumped backwards → new pass
        elif (self._last_progression > 0
              and progression < self._last_progression - 0.05):
            self._plot_grid.clear_all()
            self._last_progression = -1.0

        self._current_bead = bead_id
        self._bead_label.setText(f'Bead: {bead_id}')

        self._last_progression = progression

        p = max(0.0, min(1.0, progression))
        self._plot_grid.add_point('wfs', p, wfs)
        self._plot_grid.add_point('current', p, current)
        self._plot_grid.add_point('voltage', p, voltage)
        self._plot_grid.add_point('energy', p, energy)
        self._plot_grid.add_point('power', p, power)

    def _handle_geometry(self, bead_id: str, progression: float,
                         width_mm: float, height_mm: float,
                         toe_angle_rad: float):
        if not bead_id:
            return
        p = max(0.0, min(1.0, progression))
        self._plot_grid.add_point('width', p, width_mm)
        self._plot_grid.add_point('height', p, height_mm)
        self._plot_grid.add_point('toe_angle', p, toe_angle_rad)

    def _clear_all(self):
        self._plot_grid.clear_all()
        self._last_progression = -1.0
        self._current_bead = ''
        self._bead_label.setText('Bead: \u2014')
