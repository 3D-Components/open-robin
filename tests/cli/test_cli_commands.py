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
    ):
        return self.create_measurement_ok

    def create_ai_recommendation(self, process_id, params):
        return self.create_ai_recommendation_ok


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
        ],
    )
    assert result.exit_code == 0
    assert 'Added measurement M1 for process PX' in result.stdout


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
    params = '{"wireSpeed": 12.3, "current": 180, "voltage": 24.5}'
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
    params = '{"wireSpeed": 10}'
    result = runner.invoke(cli_app, ['add-recommendation', 'PX', params])
    assert result.exit_code != 0
    assert 'Failed to add AI recommendation' in result.stderr
