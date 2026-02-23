from typer.testing import CliRunner

from robin.cli import app as cli_app


class DummyResponse:
    def __init__(self, status_code=200, json_data=None, text=''):
        self.status_code = status_code
        self._json = json_data
        self.text = text

    def json(self):
        return self._json


def test_process_status_success_stopped(monkeypatch):
    payload = {
        'processStatus': {'value': 'stopped'},
        'operationMode': {'value': 'geometry_driven'},
        'startedAt': {'value': '2025-01-01T00:00:00Z'},
        'stoppedAt': {'value': '2025-01-02T00:00:00Z'},
        'stopReason': {'value': 'operator_request'},
    }

    def fake_get(url, headers=None, **kwargs):
        return DummyResponse(200, json_data=payload)

    monkeypatch.setattr('robin.cli.requests.get', fake_get)

    runner = CliRunner()
    result = runner.invoke(cli_app, ['process-status', 'PX'])
    assert result.exit_code == 0
    out = result.stdout
    assert 'Process Status: PX' in out
    assert 'Status: stopped' in out
    assert 'Mode: geometry_driven' in out
    assert 'Stopped: 2025-01-02T00:00:00Z' in out
    assert 'Stop Reason: operator_request' in out


def test_process_status_not_found(monkeypatch):
    def fake_get(url, headers=None, **kwargs):
        return DummyResponse(404)

    monkeypatch.setattr('robin.cli.requests.get', fake_get)

    runner = CliRunner()
    result = runner.invoke(cli_app, ['process-status', 'P404'])
    assert result.exit_code != 0
    # Error is printed to stderr via typer.echo(err=True)
    assert 'not found' in result.stderr.lower()


def test_process_status_active_minimal(monkeypatch):
    payload = {
        'processStatus': {'value': 'active'},
        'operationMode': {'value': 'parameter_driven'},
        'startedAt': {'value': '2025-01-05T00:00:00Z'},
    }

    def fake_get(url, headers=None, **kwargs):
        return DummyResponse(200, json_data=payload)

    monkeypatch.setattr('robin.cli.requests.get', fake_get)

    runner = CliRunner()
    result = runner.invoke(cli_app, ['process-status', 'PA'])
    assert result.exit_code == 0
    out = result.stdout
    assert 'Process Status: PA' in out
    assert 'Status: active' in out
    assert 'Mode: parameter_driven' in out
    # Should not print stopped lines
    assert 'Stopped:' not in out
    assert 'Stop Reason:' not in out
