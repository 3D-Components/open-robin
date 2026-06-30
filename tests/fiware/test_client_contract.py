import pytest   # NoQA

from robin.cli import RobinFiwareClient, OperationMode


class DummyResponse:
    def __init__(self, status_code=200, json_data=None, text=''):
        self.status_code = status_code
        self._json = json_data
        self.text = text

    def json(self):
        return self._json


def test_create_process_payload(monkeypatch):
    # Ensure tenant header is set in client headers for the contract
    monkeypatch.setenv('NGSILD_TENANT', 'robin')
    # Also set module constant in case module was imported before env
    import robin.cli as rcli
    monkeypatch.setattr(rcli, 'TENANT', 'robin', raising=False)
    seen = {}

    def fake_post(url, headers=None, json=None, **kwargs):
        seen['url'] = url
        seen['headers'] = headers
        seen['json'] = json
        return DummyResponse(201)

    monkeypatch.setattr('robin.cli.requests.post', fake_post)

    client = RobinFiwareClient(orion_url='http://orion-ld:1026')
    ok = client.create_process('PX', OperationMode.PARAMETER_DRIVEN)

    assert ok is True
    assert seen['url'].endswith('/ngsi-ld/v1/entities')
    assert seen['headers']['NGSILD-Tenant'] == 'robin'
    body = seen['json']
    assert body['id'] == 'urn:ngsi-ld:Process:PX'
    assert body['operationMode']['value'] == 'parameter_driven'


def test_stop_process_happy_path(monkeypatch):
    calls = {'get': 0, 'patch': 0, 'post': 0}

    def fake_get(url, headers=None, **kwargs):
        calls['get'] += 1
        # Return process active
        return DummyResponse(
            200,
            json_data={'processStatus': {'value': 'active'}},
        )

    def fake_patch(url, headers=None, json=None, **kwargs):
        calls['patch'] += 1
        return DummyResponse(204)

    def fake_post(url, headers=None, json=None, **kwargs):
        calls['post'] += 1
        return DummyResponse(204)

    monkeypatch.setattr('robin.cli.requests.get', fake_get)
    monkeypatch.setattr('robin.cli.requests.patch', fake_patch)
    monkeypatch.setattr('robin.cli.requests.post', fake_post)

    client = RobinFiwareClient()
    ok, msg = client.stop_process('P1')
    assert ok is True
    assert 'stopped successfully' in msg
    assert calls == {'get': 1, 'patch': 1, 'post': 1}


def test_resume_process_happy_path(monkeypatch):
    calls = {'get': 0, 'patch': 0, 'delete': 0}

    def fake_get(url, headers=None, **kwargs):
        calls['get'] += 1
        return DummyResponse(
            200,
            json_data={'processStatus': {'value': 'stopped'}},
        )

    def fake_patch(url, headers=None, json=None, **kwargs):
        calls['patch'] += 1
        return DummyResponse(204)

    def fake_delete(url, headers=None, **kwargs):
        calls['delete'] += 1
        return DummyResponse(204)

    monkeypatch.setattr('robin.cli.requests.get', fake_get)
    monkeypatch.setattr('robin.cli.requests.patch', fake_patch)
    monkeypatch.setattr('robin.cli.requests.delete', fake_delete)

    client = RobinFiwareClient()
    ok, msg = client.resume_process('P2')
    assert ok is True
    assert 'resumed successfully' in msg
    assert calls['get'] == 1
    assert calls['patch'] == 1
    assert calls['delete'] >= 2  # stoppedAt and stopReason
