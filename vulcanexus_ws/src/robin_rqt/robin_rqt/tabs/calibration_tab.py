"""
Calibration tab — composes TCP/stickout and guided stickout calibration.
"""

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QTextEdit,
)

from ..components.stickout_component import build_tcp_stickout_group
from ..components.jog_component import disable_servo as _disable_servo
from ..components.stickout_calibration_component import build_stickout_calibration_group


def build_calibration_tab(panel) -> QWidget:
    """Build the Calibration tab and register all sub-widgets on *panel*.

    Expected panel attributes (set in __init__):
        panel._node, panel._tf_buffer, panel._tcp_active_frame,
        panel._set_stickout_client, panel._set_tcp_mode_client,
        panel._calibrate_stickout_client,
        panel._plates, panel._cal_log (set here)
    """
    tab = QWidget()
    outer = QVBoxLayout(tab)

    # ---- TCP Mode, Stickout & Live TCP ----
    outer.addWidget(build_tcp_stickout_group(panel))

    # Guided stickout flow (includes known contact-point handling)
    outer.addWidget(build_stickout_calibration_group(panel))

    # ---- Calibration log ----
    cal_log_group = QGroupBox('Calibration Log')
    cal_log_layout = QVBoxLayout(cal_log_group)
    panel._cal_log = QTextEdit()
    panel._cal_log.setReadOnly(True)
    panel._cal_log.setStyleSheet('font-family: monospace; font-size: 11px;')
    cal_log_layout.addWidget(panel._cal_log)
    outer.addWidget(cal_log_group, stretch=2)

    outer.addStretch()
    return tab
