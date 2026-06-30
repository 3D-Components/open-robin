import requests
import sys

from typer.testing import CliRunner

import robin.cli as rcli
from robin.cli import app as cli_app


class FakeClient:
    def __init__(self, orion_url=None):
        # outcomes can be customized per test by mutating attributes
        self.create_process_ok = True
        self.create_geometry_target_ok = True
        self.set_operation_mode_ok = True
        self.create_measurement_ok = True
        self.create_ai_recommendation_ok = True
        self.stop_process_result = (True, 'Process PX stopped successfully')
        self.resume_process_result = (True, 'Process PX resumed successfully')
        self.status_result = (
            {
                'status': 'active',
                'operation_mode': 'parameter_driven',
                'started_at': '2025-01-01T00:00:00Z',
                'stopped_at': None,
                'stop_reason': None,
                'telemetry': {},
            },
            None,
        )

    # RobinFiwareClient methods
    def create_process(self, process_id, mode):
        return self.create_process_ok

    def create_geometry_target(self, process_id, height, width):
        return self.create_geometry_target_ok

    def set_operation_mode(self, process_id, mode):
        return self.set_operation_mode_ok

    def create_measurement(
        self,
        process_id,
        measurement_id,
        height,
        width,
        speed=None,
        current=None,
        voltage=None,
        input_params=None,
    ):
        return self.create_measurement_ok

    def create_ai_recommendation(self, process_id, params):
        return self.create_ai_recommendation_ok

    def stop_process(self, process_id, reason):
        return self.stop_process_result

    def resume_process(self, process_id):
        return self.resume_process_result

    def get_process_status(self, process_id):
        return self.status_result


def test_create_process_success(monkeypatch):
    def fake_ctor(orion_url=None):
        return FakeClient()

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    result = runner.invoke(
        cli_app, ['create-process', 'PX', '--mode', 'parameter_driven']
    )
    assert result.exit_code == 0
    assert 'Created process: PX' in result.stdout


def test_create_process_failure(monkeypatch):
    fc = FakeClient()
    fc.create_process_ok = False

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    result = runner.invoke(cli_app, ['create-process', 'PFAIL'])
    assert result.exit_code != 0
    # Error printed to stderr via typer.echo(err=True)
    assert 'Failed to create process' in result.stderr


def test_create_target_success(monkeypatch):
    fc = FakeClient()

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    result = runner.invoke(cli_app, ['create-target', 'PX', '5.0', '8.0'])
    assert result.exit_code == 0
    assert 'Created geometry target for process PX' in result.stdout
    assert 'Set operation mode to geometry_driven' in result.stdout


def test_create_target_warning_but_mode_ok(monkeypatch):
    fc = FakeClient()
    fc.create_geometry_target_ok = False
    fc.set_operation_mode_ok = True

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    result = runner.invoke(cli_app, ['create-target', 'PX', '5.0', '8.0'])
    assert result.exit_code == 0
    assert 'Geometry target not created (may already exist)' in result.stdout
    assert 'Set operation mode to geometry_driven' in result.stdout


def test_create_target_mode_fail_exits(monkeypatch):
    fc = FakeClient()
    fc.set_operation_mode_ok = False

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    result = runner.invoke(cli_app, ['create-target', 'PX', '5.0', '8.0'])
    assert result.exit_code != 0
    # Creation message may be on stdout, failure reason on stderr
    assert 'Failed to set operation mode' in result.stderr


def test_add_measurement_success(monkeypatch):
    fc = FakeClient()

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    result = runner.invoke(
        cli_app,
        [
            'add-measurement',
            'PX',
            'M1',
            '4.2',
            '7.9',
            '--speed',
            '10.5',
            '--current',
            '150',
            '--voltage',
            '24.5',
            '--input-param',
            'wire_feed_speed_mpm_model_input=10.5',
            '--input-param',
            'travel_speed_mps_model_input=0.021',
        ],
    )
    assert result.exit_code == 0
    assert 'Added measurement M1 for process PX' in result.stdout


def test_add_measurement_invalid_input_param(monkeypatch):
    fc = FakeClient()

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    result = runner.invoke(
        cli_app,
        [
            'add-measurement',
            'PX',
            'M1',
            '4.2',
            '7.9',
            '--input-param',
            'wire_feed_speed_mpm_model_input:not-a-pair',
        ],
    )
    assert result.exit_code != 0
    assert 'Invalid input param' in result.stderr


def test_add_measurement_failure(monkeypatch):
    fc = FakeClient()
    fc.create_measurement_ok = False

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    result = runner.invoke(
        cli_app, ['add-measurement', 'PZ', 'M2', '3.0', '4.0']
    )
    assert result.exit_code != 0
    assert 'Failed to add measurement' in result.stderr


def test_add_recommendation_success(monkeypatch):
    fc = FakeClient()

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    params = '{"wire_feed_speed_mpm_model_input": 12.3, "travel_speed_mps_model_input": 0.021, "arc_length_correction_mm_model_input": 1.5}'
    result = runner.invoke(cli_app, ['add-recommendation', 'PX', params])
    assert result.exit_code == 0
    assert 'Added AI recommendation for process PX' in result.stdout


def test_add_recommendation_invalid_json(monkeypatch):
    fc = FakeClient()

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    result = runner.invoke(cli_app, ['add-recommendation', 'PX', '{not json}'])
    assert result.exit_code != 0
    assert 'Invalid JSON format for parameters' in result.stderr


def test_add_recommendation_failure(monkeypatch):
    fc = FakeClient()
    fc.create_ai_recommendation_ok = False

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)

    runner = CliRunner()
    params = '{"wire_feed_speed_mpm_model_input": 10}'
    result = runner.invoke(cli_app, ['add-recommendation', 'PX', params])
    assert result.exit_code != 0
    assert 'Failed to add AI recommendation' in result.stderr


def test_stop_and_resume_process_commands(monkeypatch):
    fc = FakeClient()

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)
    runner = CliRunner()

    stop = runner.invoke(
        cli_app, ['stop-process', 'PX', '--reason', 'operator_request']
    )
    assert stop.exit_code == 0
    assert 'stopped successfully' in stop.stdout

    resume = runner.invoke(cli_app, ['resume-process', 'PX'])
    assert resume.exit_code == 0
    assert 'resumed successfully' in resume.stdout


def test_stop_and_resume_process_command_failures(monkeypatch):
    fc = FakeClient()
    fc.stop_process_result = (False, 'stop failed')
    fc.resume_process_result = (False, 'resume failed')

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)
    runner = CliRunner()

    stop = runner.invoke(cli_app, ['stop-process', 'PX'])
    assert stop.exit_code != 0
    assert 'stop failed' in stop.stderr

    resume = runner.invoke(cli_app, ['resume-process', 'PX'])
    assert resume.exit_code != 0
    assert 'resume failed' in resume.stderr


def test_process_status_command_prints_telemetry(monkeypatch):
    fc = FakeClient()
    fc.status_result = (
        {
            'status': 'paused',
            'operation_mode': 'geometry_driven',
            'started_at': '2025-01-01T00:00:00Z',
            'stopped_at': None,
            'stop_reason': None,
            'telemetry': {
                'height': 2.0,
                'width': 5.0,
                'speed': 0.02,
                'current': 150.0,
                'voltage': 24.0,
                'input_params': {'wire_feed_speed_mpm_model_input': 10.0},
            },
        },
        None,
    )

    def fake_ctor(orion_url=None):
        return fc

    monkeypatch.setattr(rcli, 'RobinFiwareClient', fake_ctor)
    result = CliRunner().invoke(cli_app, ['process-status', 'PX'])

    assert result.exit_code == 0
    assert 'Latest Telemetry' in result.stdout
    assert 'Height : 2.00 mm' in result.stdout
    assert 'wire_feed_speed_mpm_model_input' in result.stdout


def test_process_status_command_handles_bad_telemetry(monkeypatch):
    fc = FakeClient()
    fc.status_result = (
        {
            'status': 'active',
            'operation_mode': 'parameter_driven',
            'started_at': '2025-01-01T00:00:00Z',
            'stopped_at': None,
            'stop_reason': None,
            'telemetry': {'height': 'not-a-number'},
        },
        None,
    )

    monkeypatch.setattr(rcli, 'RobinFiwareClient', lambda orion_url=None: fc)
    result = CliRunner().invoke(cli_app, ['process-status', 'PX'])

    assert result.exit_code == 0
    assert 'Telemetry attribute exists' in result.stdout


def test_list_processes_empty_filtered_error_and_connection(monkeypatch):
    runner = CliRunner()

    monkeypatch.setattr(
        rcli.requests,
        'get',
        lambda *_args, **_kwargs: types_response(200, []),
    )
    empty = runner.invoke(cli_app, ['list-processes'])
    assert empty.exit_code == 0
    assert 'No processes found' in empty.stdout

    sample = [
        {
            'id': 'urn:ngsi-ld:Process:PX',
            'processStatus': {'value': 'active'},
            'operationMode': {'value': 'parameter_driven'},
            'startedAt': {'value': '2025-01-01T00:00:00Z'},
        }
    ]
    monkeypatch.setattr(
        rcli.requests,
        'get',
        lambda *_args, **_kwargs: types_response(200, sample),
    )
    filtered = runner.invoke(
        cli_app, ['list-processes', '--status-filter', 'stopped']
    )
    assert filtered.exit_code == 0
    assert 'No processes found with status: stopped' in filtered.stdout

    monkeypatch.setattr(
        rcli.requests,
        'get',
        lambda *_args, **_kwargs: types_response(500, {'error': 'bad'}),
    )
    failed = runner.invoke(cli_app, ['list-processes'])
    assert failed.exit_code != 0
    assert 'Failed to fetch processes' in failed.stderr

    def raise_request(*_args, **_kwargs):
        raise requests.RequestException('offline')

    monkeypatch.setattr(rcli.requests, 'get', raise_request)
    offline = runner.invoke(cli_app, ['list-processes'])
    assert offline.exit_code != 0
    assert 'Failed to connect' in offline.stderr


def test_status_command_reports_failures(monkeypatch):
    calls = {'mintaka': 0}

    def fake_get(url, timeout=5, **kwargs):
        if url.endswith('/version'):
            return types_response(503, {'orion': 'down'})
        calls['mintaka'] += 1
        raise requests.RequestException('mintaka down')

    monkeypatch.setattr(rcli.requests, 'get', fake_get)
    monkeypatch.setattr(rcli.time, 'sleep', lambda _seconds: None)

    result = CliRunner().invoke(cli_app, ['status'])
    assert result.exit_code == 0
    assert 'Orion Context Broker: Not responding' in result.stdout
    assert 'Mintaka: Connection failed' in result.stdout
    assert calls['mintaka'] == 3


def test_serve_command_import_failure(monkeypatch):
    monkeypatch.setitem(sys.modules, 'uvicorn', None)
    result = CliRunner().invoke(cli_app, ['serve'])
    assert result.exit_code != 0
    assert 'Failed to start server' in result.stderr


class types_response:
    def __init__(self, status_code, json_data=None, text=''):
        self.status_code = status_code
        self._json = json_data
        self.text = text

    def json(self):
        return self._json
