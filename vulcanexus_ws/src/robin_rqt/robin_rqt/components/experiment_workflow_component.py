"""
Experiment workflow UI component:
- Plan experiment from system-agnostic schema
- Approve generated plan
- Execute approved plan
"""

import json

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from std_srvs.srv import Trigger

from robin_interfaces.action import WeldExperiment
from robin_interfaces.msg import ExperimentBeadSpec, PlateLayout
from robin_interfaces.srv import PlanExperiment, ApproveExperimentPlan

from ..tabs.plates_tab import plates_to_msgs


def _log(panel, text: str):
    target = getattr(panel, '_exp_log', None) or getattr(panel, '_cal_log', None)
    if target is not None:
        target.append(text)


def build_experiment_workflow_group(panel) -> QGroupBox:
    group = QGroupBox('Welding Experiment Workflow')
    layout = QVBoxLayout(group)

    top = QGridLayout()
    top.addWidget(QLabel('Experiment ID:'), 0, 0)
    panel._exp_id_edit = QLineEdit()
    panel._exp_id_edit.setPlaceholderText('e.g. weld-exp-2026-02-18')
    top.addWidget(panel._exp_id_edit, 0, 1)

    top.addWidget(QLabel('Spacing X:'), 0, 2)
    panel._exp_spacing_x = QDoubleSpinBox()
    panel._exp_spacing_x.setRange(0.0, 200.0)
    panel._exp_spacing_x.setDecimals(1)
    panel._exp_spacing_x.setValue(30.0)
    panel._exp_spacing_x.setSuffix(' mm')
    top.addWidget(panel._exp_spacing_x, 0, 3)

    top.addWidget(QLabel('Spacing Y:'), 0, 4)
    panel._exp_spacing_y = QDoubleSpinBox()
    panel._exp_spacing_y.setRange(0.0, 200.0)
    panel._exp_spacing_y.setDecimals(1)
    panel._exp_spacing_y.setValue(30.0)
    panel._exp_spacing_y.setSuffix(' mm')
    top.addWidget(panel._exp_spacing_y, 0, 5)

    top.addWidget(QLabel('Margin X:'), 1, 0)
    panel._exp_margin_x = QDoubleSpinBox()
    panel._exp_margin_x.setRange(0.0, 200.0)
    panel._exp_margin_x.setDecimals(1)
    panel._exp_margin_x.setValue(40.0)
    panel._exp_margin_x.setSuffix(' mm')
    top.addWidget(panel._exp_margin_x, 1, 1)

    top.addWidget(QLabel('Margin Y:'), 1, 2)
    panel._exp_margin_y = QDoubleSpinBox()
    panel._exp_margin_y.setRange(0.0, 200.0)
    panel._exp_margin_y.setDecimals(1)
    panel._exp_margin_y.setValue(40.0)
    panel._exp_margin_y.setSuffix(' mm')
    top.addWidget(panel._exp_margin_y, 1, 3)

    panel._exp_staggered = QCheckBox('Staggered placement')
    panel._exp_staggered.setChecked(True)
    top.addWidget(panel._exp_staggered, 1, 4)

    panel._exp_dry_run = QCheckBox('Execute as dry-run')
    panel._exp_dry_run.setChecked(False)
    top.addWidget(panel._exp_dry_run, 1, 5)

    layout.addLayout(top)

    layout.addWidget(QLabel('Schema JSON (array of bead specs):'))
    panel._exp_schema_edit = QTextEdit()
    panel._exp_schema_edit.setStyleSheet('font-family: monospace; font-size: 11px;')
    panel._exp_schema_edit.setMinimumHeight(120)
    panel._exp_schema_edit.setPlainText(
        '[\n'
        '  {"weld_current": 180.0, "weld_speed": 0.006, "stickout": 0.015, "scan_speed": 0.006},\n'
        '  {"weld_current": 200.0, "weld_speed": 0.007, "stickout": 0.015, "scan_speed": 0.007}\n'
        ']'
    )
    layout.addWidget(panel._exp_schema_edit)

    schema_tools = QHBoxLayout()
    load_btn = QPushButton('Load Schema File...')
    load_btn.clicked.connect(lambda: _load_schema_file(panel))
    schema_tools.addWidget(load_btn)
    schema_tools.addStretch()
    layout.addLayout(schema_tools)

    btns = QHBoxLayout()
    plan_btn = QPushButton('Plan')
    approve_btn = QPushButton('Approve')
    execute_btn = QPushButton('Execute Approved Plan')
    terminate_btn = QPushButton('Terminate + Lift 100mm')
    terminate_btn.setStyleSheet('font-weight: bold;')

    plan_btn.clicked.connect(lambda: _plan_experiment(panel))
    approve_btn.clicked.connect(lambda: _approve_plan(panel, True))
    execute_btn.clicked.connect(lambda: _execute_approved_plan(panel))
    terminate_btn.clicked.connect(lambda: _terminate_experiment(panel))

    btns.addWidget(plan_btn)
    btns.addWidget(approve_btn)
    btns.addWidget(execute_btn)
    btns.addWidget(terminate_btn)
    layout.addLayout(btns)

    panel._exp_plan_status = QLabel('Plan ID: —')
    panel._exp_plan_status.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    panel._exp_plan_status.setStyleSheet('font-weight: bold;')
    layout.addWidget(panel._exp_plan_status)

    return group


def _parse_schema(text: str):
    payload = json.loads(text)
    if isinstance(payload, dict):
        payload = payload.get('beads', [])
    if not isinstance(payload, list):
        raise ValueError('Schema must be a JSON array or object with beads[]')

    specs = []
    for idx, bead in enumerate(payload, start=1):
        if not isinstance(bead, dict):
            raise ValueError(f'Entry #{idx} must be an object')

        spec = ExperimentBeadSpec()
        spec.input_id = str(bead.get('input_id', f'in-{idx:03d}'))
        current = bead.get('weld_current', bead.get('current'))
        speed = bead.get('weld_speed', bead.get('tcp_speed'))
        stickout = bead.get('stickout')
        if current is None or speed is None or stickout is None:
            raise ValueError(
                f'Entry #{idx} requires current/weld_current, tcp_speed/weld_speed, and stickout')

        raw_speed = float(speed)
        raw_scan_speed = float(bead.get('scan_speed', 0.0))

        # Accept tcp_speed commonly provided in mm/s and convert to m/s.
        # If weld_speed is used and >0.2, assume mm/s as a safety heuristic.
        if 'tcp_speed' in bead:
            weld_speed_mps = raw_speed / 1000.0
        else:
            weld_speed_mps = raw_speed / 1000.0 if raw_speed > 0.2 else raw_speed

        if 'scan_speed' in bead and raw_scan_speed > 0.0:
            scan_speed_mps = raw_scan_speed / 1000.0 if raw_scan_speed > 0.2 else raw_scan_speed
        else:
            scan_speed_mps = 0.0

        raw_stickout = float(stickout)
        stickout_m = raw_stickout / 1000.0 if raw_stickout > 1.0 else raw_stickout

        spec.weld_current = float(current)
        spec.weld_speed = weld_speed_mps
        spec.stickout = stickout_m
        spec.scan_speed = scan_speed_mps
        specs.append(spec)

    if not specs:
        raise ValueError('Schema has no bead specs')
    return specs


def _plan_experiment(panel):
    if not panel._plan_experiment_client.service_is_ready():
        _log(panel, 'ERROR: /experiment/plan service not available')
        return

    try:
        bead_specs = _parse_schema(panel._exp_schema_edit.toPlainText())
    except Exception as exc:
        _log(panel, f'ERROR: invalid schema JSON: {exc}')
        return

    plates = plates_to_msgs(panel)
    if not plates:
        _log(panel, 'ERROR: no plates defined in Plates tab')
        return

    request = PlanExperiment.Request()
    request.experiment_id = panel._exp_id_edit.text().strip()
    request.beads = bead_specs
    request.plates = plates
    request.spacing_x = panel._exp_spacing_x.value() / 1000.0
    request.spacing_y = panel._exp_spacing_y.value() / 1000.0
    request.margin_x = panel._exp_margin_x.value() / 1000.0
    request.margin_y = panel._exp_margin_y.value() / 1000.0
    request.staggered = panel._exp_staggered.isChecked()
    request.publish_preview = True

    _log(panel,
        f'Planning experiment: beads={len(bead_specs)}, plates={len(plates)}')
    panel._exp_plan_status.setText('Plan ID: planning...')

    future = panel._plan_experiment_client.call_async(request)
    future.add_done_callback(lambda f: _on_plan_done(panel, f))


def _load_schema_file(panel):
    path, _ = QFileDialog.getOpenFileName(
        panel._widget,
        'Load Experiment Schema JSON',
        '',
        'JSON (*.json)',
    )
    if not path:
        return
    try:
        with open(path) as f:
            text = f.read()
        panel._exp_schema_edit.setPlainText(text)
        _log(panel, f'Loaded schema file: {path}')
    except Exception as exc:
        _log(panel, f'ERROR: failed to load schema file: {exc}')


def _on_plan_done(panel, future):
    result = future.result()
    if result is None:
        _log(panel, 'ERROR: /experiment/plan call failed')
        panel._exp_plan_status.setText('Plan ID: error')
        return

    if result.success:
        panel._current_plan_id = result.plan_id
        panel._exp_plan_status.setText(f'Plan ID: {result.plan_id}')
        _log(panel,
            f'PLAN OK: {result.message} (plan_id={result.plan_id})')
    else:
        panel._exp_plan_status.setText('Plan ID: failed')
        _log(panel, f'PLAN FAILED: {result.message}')


def _approve_plan(panel, approve: bool):
    if not panel._current_plan_id:
        _log(panel, 'ERROR: no plan_id available; run Plan first')
        return
    if not panel._approve_plan_client.service_is_ready():
        _log(panel, 'ERROR: /experiment/approve service not available')
        return

    request = ApproveExperimentPlan.Request()
    request.plan_id = panel._current_plan_id
    request.approve = bool(approve)
    _log(panel, f'Sending approval for plan_id={request.plan_id}...')

    future = panel._approve_plan_client.call_async(request)
    future.add_done_callback(lambda f: _on_approve_done(panel, f))


def _on_approve_done(panel, future):
    result = future.result()
    if result is None:
        _log(panel, 'ERROR: /experiment/approve call failed')
        return
    if result.success and result.approved:
        _log(panel, f'APPROVED: {result.message}')
    elif result.success:
        _log(panel, f'REJECTED: {result.message}')
    else:
        _log(panel, f'APPROVAL FAILED: {result.message}')


def _execute_approved_plan(panel):
    if not panel._current_plan_id:
        _log(panel, 'ERROR: no plan_id available; run Plan first')
        return

    goal = WeldExperiment.Goal()
    goal.plan_id = panel._current_plan_id
    goal.beads = []
    goal.plates = []
    goal.layout = PlateLayout()
    goal.dry_run = panel._exp_dry_run.isChecked()

    _log(panel,
        f'Sending /weld_experiment goal for plan_id={goal.plan_id}, dry_run={goal.dry_run}')

    process_plot = getattr(panel, '_exp_process_plot', None)
    if process_plot is not None:
        process_plot.clear()

    if not panel._weld_experiment_action_client.wait_for_server(timeout_sec=2.0):
        _log(panel, 'ERROR: /weld_experiment action server not available')
        return

    send_future = panel._weld_experiment_action_client.send_goal_async(
        goal,
        feedback_callback=lambda fb: _on_weld_feedback(panel, fb),
    )
    send_future.add_done_callback(lambda f: _on_goal_response(panel, f))


def _on_goal_response(panel, future):
    goal_handle = future.result()
    if goal_handle is None or not goal_handle.accepted:
        _log(panel, 'EXECUTION REJECTED by /weld_experiment')
        return

    panel._active_experiment_goal_handle = goal_handle
    _log(panel, 'Execution goal accepted')
    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(lambda f: _on_weld_result(panel, f))


def _on_weld_feedback(panel, feedback_msg):
    fb = feedback_msg.feedback
    _log(panel,
        f"Feedback: {fb.status} | {fb.progress_percentage:.1f}% | bead={fb.current_bead_id}")


def _on_weld_result(panel, future):
    wrapped = future.result()
    if wrapped is None:
        _log(panel, 'ERROR: no result from /weld_experiment')
        return

    result = wrapped.result
    panel._active_experiment_goal_handle = None
    if result.success:
        _log(panel,
            f"EXECUTION OK: {result.message} (completed={len(result.completed_bead_ids)})")
    else:
        _log(panel, f'EXECUTION FAILED: {result.message}')


def _terminate_experiment(panel):
    goal_handle = getattr(panel, '_active_experiment_goal_handle', None)
    if goal_handle is not None:
        _log(panel, 'Sending action cancel request...')
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:
            _log(panel, f'WARN: goal cancel request failed: {exc}')

    terminate_client = getattr(panel, '_terminate_experiment_client', None)
    if terminate_client is None:
        _log(panel, 'ERROR: terminate client is not configured')
        return
    if not terminate_client.service_is_ready():
        _log(panel, 'ERROR: /experiment/terminate service not available')
        return

    _log(panel, 'Requesting immediate termination + 100mm lift...')
    future = terminate_client.call_async(Trigger.Request())
    future.add_done_callback(lambda f: _on_terminate_done(panel, f))


def _on_terminate_done(panel, future):
    result = future.result()
    if result is None:
        _log(panel, 'ERROR: /experiment/terminate call failed')
        return
    if result.success:
        _log(panel, f'TERMINATE OK: {result.message}')
    else:
        _log(panel, f'TERMINATE FAILED: {result.message}')
