"""
Experiment tab — schema planning, approval, and execution workflow.
"""

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QTextEdit,
)

from ..components.experiment_workflow_component import build_experiment_workflow_group
from ..components.process_plot_widget import ProcessPlotWidget


def build_experiment_tab(panel) -> QWidget:
    """Build the Experiment tab with dedicated workflow log."""
    tab = QWidget()
    outer = QVBoxLayout(tab)

    outer.addWidget(build_experiment_workflow_group(panel))

    process_group = QGroupBox('Live Process Signals')
    process_layout = QVBoxLayout(process_group)
    panel._exp_process_plot = ProcessPlotWidget()
    process_layout.addWidget(panel._exp_process_plot)
    outer.addWidget(process_group, stretch=2)

    exp_log_group = QGroupBox('Experiment Log')
    exp_log_layout = QVBoxLayout(exp_log_group)
    panel._exp_log = QTextEdit()
    panel._exp_log.setReadOnly(True)
    panel._exp_log.setStyleSheet('font-family: monospace; font-size: 11px;')
    exp_log_layout.addWidget(panel._exp_log)
    outer.addWidget(exp_log_group, stretch=2)

    outer.addStretch()
    return tab
