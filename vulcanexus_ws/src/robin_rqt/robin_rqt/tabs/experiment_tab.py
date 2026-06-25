"""
Experiment tab — schema planning, approval, and execution workflow.

Process plots are handled by a separate RQT widget; this tab contains
only the workflow controls and experiment log.
"""

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QTextEdit,
)

from ..components.experiment_workflow_component import build_experiment_workflow_group
from ..components.widgets import apply_touch_style


def build_experiment_tab(panel) -> QWidget:
    """Build the Experiment tab (no process plot)."""
    tab = QWidget()
    outer = QVBoxLayout(tab)
    outer.setSpacing(4)
    outer.setContentsMargins(4, 4, 4, 4)

    outer.addWidget(build_experiment_workflow_group(panel))

    exp_log_group = QGroupBox('Experiment Log')
    exp_log_layout = QVBoxLayout(exp_log_group)
    exp_log_layout.setContentsMargins(4, 4, 4, 4)
    panel._exp_log = QTextEdit()
    panel._exp_log.setReadOnly(True)
    exp_log_layout.addWidget(panel._exp_log)
    outer.addWidget(exp_log_group, stretch=1)

    apply_touch_style(tab)
    return tab
