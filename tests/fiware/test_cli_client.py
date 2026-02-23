from typer.testing import CliRunner

from robin.cli import RobinFiwareClient, OperationMode, app as cli_app


class DummyResponse:
    def __init__(self, status_code=200, json_data=None, text=''):
        self.status_code = status_code
        self._json = json_data
        self.text = text

    def json(self):
        return self._json


def test_create_geometry_target_payload(monkeypatch):
    seen = {}

    def fake_post(url, headers=None, json=None, **kwargs):
        seen['url'] = url
        seen['headers'] = headers
        seen['json'] = json
        return DummyResponse(201)

    monkeypatch.setattr('robin.cli.requests.post', fake_post)

    client = RobinFiwareClient()
    ok = client.create_geometry_target('PX', 5.0, 8.0)
    assert ok is True
    assert seen['url'].endswith('/ngsi-ld/v1/entities')
    body = seen['json']
    assert body['id'] == 'urn:ngsi-ld:GeometryTarget:PX'
    assert body['type'] == 'GeometryTarget'
    assert body['targetHeight']['value'] == 5.0
    assert body['targetWidth']['value'] == 8.0
    assert body['processId']['object'].endswith(':Process:PX')


def test_set_operation_mode_patch(monkeypatch):
    captured = {}

    def fake_patch(url, headers=None, json=None, **kwargs):
        captured['url'] = url
        captured['json'] = json
        return DummyResponse(204)

    monkeypatch.setattr('robin.cli.requests.patch', fake_patch)

    client = RobinFiwareClient()
    ok = client.set_operation_mode('P2', OperationMode.GEOMETRY_DRIVEN)
    assert ok is True
    assert '/entities/urn:ngsi-ld:Process:P2/attrs' in captured['url']
    assert captured['json']['operationMode']['value'] == 'geometry_driven'


def test_create_measurement_with_params_success(monkeypatch):
    calls = {'post_entity': 0, 'patch_troe': 0}
    last = {}

    def fake_post(url, headers=None, json=None, **kwargs):
        # Measurement entity creation
        calls['post_entity'] += 1
        last['post_url'] = url
        last['post_json'] = json
        return DummyResponse(201)

    def fake_patch(url, headers=None, json=None, **kwargs):
        # TROE update on Process
        calls['patch_troe'] += 1
        last['patch_url'] = url
        last['patch_json'] = json
        return DummyResponse(204)

    monkeypatch.setattr('robin.cli.requests.post', fake_post)
    monkeypatch.setattr('robin.cli.requests.patch', fake_patch)

    client = RobinFiwareClient()
    ok = client.create_measurement(
        process_id='PROC',
        measurement_id='M1',
        height=4.2,
        width=7.9,
        speed=10.5,
        current=150.0,
        voltage=24.0,
    )
    assert ok is True
    assert calls['post_entity'] == 1
    assert calls['patch_troe'] == 1
    assert last['post_url'].endswith('/ngsi-ld/v1/entities')
    # Ensure process TROE patch targets Process entity
    assert '/entities/urn:ngsi-ld:Process:PROC/attrs' in last['patch_url']


def test_create_ai_recommendation_payload(monkeypatch):
    seen = {}

    def fake_post(url, headers=None, json=None, **kwargs):
        seen['url'] = url
        seen['json'] = json
        return DummyResponse(201)

    monkeypatch.setattr('robin.cli.requests.post', fake_post)

    client = RobinFiwareClient()
    ok = client.create_ai_recommendation(
        'PROC', {'wireSpeed': 12.3, 'current': 180, 'voltage': 24.5}
    )
    assert ok is True
    body = seen['json']
    assert body['type'] == 'AIRecommendation'
    assert 'recommendedParams' in body
    assert body['processId']['object'].endswith(':Process:PROC')


def test_cli_list_processes_prints(monkeypatch):
    sample = [
        {
            'id': 'urn:ngsi-ld:Process:PX',
            'processStatus': {'value': 'active'},
            'operationMode': {'value': 'parameter_driven'},
            'startedAt': {'value': '2025-01-01T00:00:00Z'},
        },
        {
            'id': 'urn:ngsi-ld:Process:QY',
            'processStatus': {'value': 'stopped'},
            'operationMode': {'value': 'geometry_driven'},
            'startedAt': {'value': '2025-01-02T00:00:00Z'},
        },
    ]

    def fake_get(url, headers=None, **kwargs):
        return DummyResponse(200, json_data=sample)

    monkeypatch.setattr('robin.cli.requests.get', fake_get)

    runner = CliRunner()
    result = runner.invoke(cli_app, ['list-processes'])
    assert result.exit_code == 0
    out = result.stdout
    assert 'PX' in out and 'QY' in out
    assert 'active' in out and 'stopped' in out


def test_cli_status_reports_connected(monkeypatch):
    def fake_get(url, timeout=5, **kwargs):
        # ORION /version or Mintaka probe URLs
        if url.endswith('/version'):
            return DummyResponse(200, json_data={'orion': 'ok'})
        if '/temporal/entities' in url or url.endswith('/health'):
            # Accept either probe method
            return DummyResponse(200, json_data={'results': []})
        return DummyResponse(404)

    monkeypatch.setattr('robin.cli.requests.get', fake_get)

    runner = CliRunner()
    result = runner.invoke(cli_app, ['status'])
    assert result.exit_code == 0
    out = result.stdout
    assert 'Orion Context Broker: Connected' in out
    assert 'Mintaka: Connected' in out


def test_cli_serve_invokes_uvicorn(monkeypatch):
    # Provide a fake uvicorn module before import inside the command
    import types
    import sys

    fake_uvicorn = types.SimpleNamespace()

    def fake_run(app, host='0.0.0.0', port=8000):
        return None

    fake_uvicorn.run = fake_run
    monkeypatch.setitem(sys.modules, 'uvicorn', fake_uvicorn)

    runner = CliRunner()
    result = runner.invoke(
        cli_app, ['serve', '--host', '127.0.0.1', '--port', '9999']
    )
    assert result.exit_code == 0


def test_create_alert_payload_and_success(monkeypatch):
    class AlertData:
        def __init__(self):
            self.process_id = 'PROC'
            self.deviation_type = 'height'
            self.expected_value = {'height': 5.0, 'width': 8.0}
            self.measured_value = {'height': 6.0, 'width': 8.2}
            self.deviation_percentage = 20.0
            self.recommended_actions = ['slow down', 'reduce current']

    seen = {}

    def fake_post(url, headers=None, json=None, **kwargs):
        seen['url'] = url
        seen['headers'] = headers
        seen['json'] = json
        return DummyResponse(201)

    monkeypatch.setattr('robin.cli.requests.post', fake_post)
    client = RobinFiwareClient()
    ok = client.create_alert(AlertData())
    assert ok is True
    body = seen['json']
    assert body['type'] == 'Alert'
    assert body['processId']['object'].endswith(':Process:PROC')
    assert body['deviationType']['value'] == 'height'
    assert body['expectedValue']['value']['height'] == 5.0
    assert body['measuredValue']['value']['height'] == 6.0
    assert body['deviationPercentage']['value'] == 20.0
    assert body['recommendedActions']['value'] == [
        'slow down',
        'reduce current',
    ]
    assert 'timestamp' in body


def test_create_alert_failure(monkeypatch):
    class AlertData:
        def __init__(self):
            self.process_id = 'PROC'
            self.deviation_type = 'width'
            self.expected_value = {'height': 5.0, 'width': 8.0}
            self.measured_value = {'height': 5.5, 'width': 9.0}
            self.deviation_percentage = 12.5
            self.recommended_actions = ['adjust voltage']

    def fake_post(url, headers=None, json=None, **kwargs):
        return DummyResponse(500, text='err')

    monkeypatch.setattr('robin.cli.requests.post', fake_post)
    client = RobinFiwareClient()
    ok = client.create_alert(AlertData())
    assert ok is False


def test_create_process_failure(monkeypatch):
    def fake_post(url, headers=None, json=None, **kwargs):
        return DummyResponse(500, text='err')

    monkeypatch.setattr('robin.cli.requests.post', fake_post)
    client = RobinFiwareClient()
    ok = client.create_process('P1', OperationMode.PARAMETER_DRIVEN)
    assert ok is False


def test_create_geometry_target_failure(monkeypatch):
    def fake_post(url, headers=None, json=None, **kwargs):
        return DummyResponse(400, text='bad')

    monkeypatch.setattr('robin.cli.requests.post', fake_post)
    client = RobinFiwareClient()
    ok = client.create_geometry_target('P2', 5.0, 8.0)
    assert ok is False


def test_set_operation_mode_failure(monkeypatch):
    def fake_patch(url, headers=None, json=None, **kwargs):
        return DummyResponse(500, text='fail')

    monkeypatch.setattr('robin.cli.requests.patch', fake_patch)
    client = RobinFiwareClient()
    ok = client.set_operation_mode('P3', OperationMode.PARAMETER_DRIVEN)
    assert ok is False


def test_create_measurement_fail_entity(monkeypatch):
    def fake_post(url, headers=None, json=None, **kwargs):
        return DummyResponse(500, text='entity create failed')

    monkeypatch.setattr('robin.cli.requests.post', fake_post)
    client = RobinFiwareClient()
    ok = client.create_measurement('P4', 'M1', 4.0, 7.5)
    assert ok is False


def test_create_measurement_fail_troe_patch(monkeypatch):
    calls = {'post': 0, 'patch': 0}

    def fake_post(url, headers=None, json=None, **kwargs):
        calls['post'] += 1
        return DummyResponse(201)

    def fake_patch(url, headers=None, json=None, **kwargs):
        calls['patch'] += 1
        return DummyResponse(500, text='troe failed')

    monkeypatch.setattr('robin.cli.requests.post', fake_post)
    monkeypatch.setattr('robin.cli.requests.patch', fake_patch)
    client = RobinFiwareClient()
    ok = client.create_measurement('P5', 'M2', 4.0, 7.5, speed=10.0)
    assert calls['post'] == 1 and calls['patch'] == 1
    assert ok is False


def test_create_ai_recommendation_failure(monkeypatch):
    def fake_post(url, headers=None, json=None, **kwargs):
        return DummyResponse(500)

    monkeypatch.setattr('robin.cli.requests.post', fake_post)
    client = RobinFiwareClient()
    ok = client.create_ai_recommendation('P6', {'wireSpeed': 10})
    assert ok is False


def test_stop_process_not_found(monkeypatch):
    def fake_get(url, headers=None, **kwargs):
        return DummyResponse(404)

    monkeypatch.setattr('robin.cli.requests.get', fake_get)
    client = RobinFiwareClient()
    ok, msg = client.stop_process('P404')
    assert ok is False
    assert 'not found' in msg.lower()


def test_stop_process_patch_failure(monkeypatch):
    def fake_get(url, headers=None, **kwargs):
        return DummyResponse(
            200, json_data={'processStatus': {'value': 'active'}}
        )

    def fake_patch(url, headers=None, json=None, **kwargs):
        return DummyResponse(500, text='boom')

    monkeypatch.setattr('robin.cli.requests.get', fake_get)
    monkeypatch.setattr('robin.cli.requests.patch', fake_patch)
    client = RobinFiwareClient()
    ok, msg = client.stop_process('P7')
    assert ok is False
    assert 'Failed to update process status' in msg


def test_resume_process_already_active(monkeypatch):
    def fake_get(url, headers=None, **kwargs):
        return DummyResponse(
            200, json_data={'processStatus': {'value': 'active'}}
        )

    monkeypatch.setattr('robin.cli.requests.get', fake_get)
    client = RobinFiwareClient()
    ok, msg = client.resume_process('P8')
    assert ok is False
    assert 'already active' in msg


def test_resume_process_remove_attr_failure(monkeypatch):
    calls = {'patch': 0, 'delete': 0}

    def fake_get(url, headers=None, **kwargs):
        return DummyResponse(
            200, json_data={'processStatus': {'value': 'stopped'}}
        )

    def fake_patch(url, headers=None, json=None, **kwargs):
        calls['patch'] += 1
        return DummyResponse(204)

    def fake_delete(url, headers=None, **kwargs):
        calls['delete'] += 1
        return DummyResponse(500, text='err')

    monkeypatch.setattr('robin.cli.requests.get', fake_get)
    monkeypatch.setattr('robin.cli.requests.patch', fake_patch)
    monkeypatch.setattr('robin.cli.requests.delete', fake_delete)
    client = RobinFiwareClient()
    ok, msg = client.resume_process('P9')
    assert calls['patch'] == 1 and calls['delete'] >= 1
    assert ok is False
    assert 'Failed to remove' in msg


def test_get_process_status_not_found(monkeypatch):
    def fake_get(url, headers=None, **kwargs):
        return DummyResponse(404)

    monkeypatch.setattr('robin.cli.requests.get', fake_get)

    client = RobinFiwareClient()
    info, err = client.get_process_status('P404')
    assert info is None
    assert err and 'not found' in err.lower()


def test_get_process_status_error(monkeypatch):
    def fake_get(url, headers=None, **kwargs):
        return DummyResponse(500, text='boom')

    monkeypatch.setattr('robin.cli.requests.get', fake_get)

    client = RobinFiwareClient()
    info, err = client.get_process_status('PERR')
    assert info is None
    assert err and 'failed to fetch' in err.lower()


def test_get_process_status_success(monkeypatch):
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

    client = RobinFiwareClient()
    info, err = client.get_process_status('P4')
    assert err is None
    assert info['process_id'] == 'P4'
    assert info['status'] == 'stopped'
    assert info['operation_mode'] == 'geometry_driven'
    assert info['started_at'] == '2025-01-01T00:00:00Z'
    assert info['stopped_at'] == '2025-01-02T00:00:00Z'
    assert info['stop_reason'] == 'operator_request'
