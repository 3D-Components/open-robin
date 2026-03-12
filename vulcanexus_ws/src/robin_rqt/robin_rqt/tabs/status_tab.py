"""
Status tab — live WAGO PLC & Fronius feedback (read-only indicators).
"""

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QGroupBox,
)

from ..components.widgets import (
    StatusIndicator, FloatReadout,
    WAGO_BOOL_TOPICS, WAGO_FLOAT_TOPICS, FRONIUS_FLOAT_TOPICS,
)


def build_status_tab(panel) -> QWidget:
    """Build the Status tab widget and register indicators on *panel*.

    Populates ``panel._bool_indicators`` and ``panel._float_readouts``.
    """
    tab = QWidget()
    outer = QVBoxLayout(tab)

    # WAGO bool indicators
    wago_group = QGroupBox('WAGO PLC Feedback')
    grid = QGridLayout(wago_group)
    for i, (topic, label) in enumerate(WAGO_BOOL_TOPICS):
        row, col = divmod(i, 3)
        lbl = QLabel(label + ':')
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        ind = StatusIndicator(label)
        panel._bool_indicators[topic] = ind
        grid.addWidget(lbl, row, col * 2)
        grid.addWidget(ind, row, col * 2 + 1)
    outer.addWidget(wago_group)

    # Float readouts (WAGO + Fronius side-by-side)
    float_row = QHBoxLayout()

    wago_float_group = QGroupBox('WAGO Analog Readouts')
    wfg = QGridLayout(wago_float_group)
    for i, (topic, label) in enumerate(WAGO_FLOAT_TOPICS):
        lbl = QLabel(label + ':')
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        ro = FloatReadout()
        panel._float_readouts[topic] = ro
        wfg.addWidget(lbl, i, 0)
        wfg.addWidget(ro, i, 1)
    float_row.addWidget(wago_float_group)

    fronius_group = QGroupBox('Fronius OPC UA Display')
    fg = QGridLayout(fronius_group)
    for i, (topic, label) in enumerate(FRONIUS_FLOAT_TOPICS):
        lbl = QLabel(label + ':')
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        ro = FloatReadout()
        panel._float_readouts[topic] = ro
        fg.addWidget(lbl, i, 0)
        fg.addWidget(ro, i, 1)
    float_row.addWidget(fronius_group)

    outer.addLayout(float_row)
    outer.addStretch()
    return tab
