import pytest


@pytest.mark.asyncio
async def test_create_process_parameter_driven_success(monkeypatch, client):
    import robin.alert_engine as ae

    class FakeClient:
        def __init__(self, *_, **__):
            pass

        def create_process(self, process_id, mode):
            return True

        def create_geometry_target(self, *args, **kwargs):  # not used
            return True

    # Route uses module-level RobinFiwareClient
    monkeypatch.setattr(ae, 'RobinFiwareClient', FakeClient)

    payload = {
        'process_id': 'PROC1',
        'mode': 'parameter_driven',
        'initial_params': {'wireSpeed': 2.5, 'current': 110, 'voltage': 17},
    }
    resp = await client.post('/create-process', json=payload)
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'success'
    assert body['process_id'] == 'PROC1'


@pytest.mark.asyncio
async def test_create_process_geometry_target_warning(monkeypatch, client):
    import robin.alert_engine as ae

    class FakeClient:
        def __init__(self, *_, **__):
            pass

        def create_process(self, process_id, mode):
            return True

        def create_geometry_target(self, *args, **kwargs):
            return False

    monkeypatch.setattr(ae, 'RobinFiwareClient', FakeClient)

    payload = {
        'process_id': 'PROC2',
        'mode': 'geometry_driven',
        'target_geometry': {'height': 5.0, 'width': 6.0},
    }
    resp = await client.post('/create-process', json=payload)
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'warning'
    assert 'failed to set target geometry' in body['error'].lower()


@pytest.mark.asyncio
async def test_stop_and_resume_endpoints_success(monkeypatch, client):
    # These endpoints import RobinFiwareClient inside the function from robin.cli
    import robin.cli as rcli

    class FakeClient:
        def __init__(self, *_, **__):
            pass

        def stop_process(self, pid, reason):
            return True, f'Process {pid} stopped successfully'

        def resume_process(self, pid):
            return True, f'Process {pid} resumed successfully'

    monkeypatch.setattr(rcli, 'RobinFiwareClient', FakeClient)

    r1 = await client.post(
        '/process/PX/stop', params={'reason': 'operator_request'}
    )
    assert r1.status_code == 200
    assert r1.json()['status'] == 'success'

    r2 = await client.post('/process/PX/resume')
    assert r2.status_code == 200
    assert r2.json()['status'] == 'success'


@pytest.mark.asyncio
async def test_get_process_target_success(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_geometry_target',
        lambda pid: {'height': 5.0, 'width': 6.0},
    )

    resp = await client.get('/process/PX/target')
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'success'
    assert body['target_geometry']['height'] == 5.0


@pytest.mark.asyncio
async def test_list_processes_api_with_filter(monkeypatch, client):
    import robin.alert_engine as ae

    sample = [
        {
            'id': 'urn:ngsi-ld:Process:A',
            'processStatus': {'value': 'active'},
            'operationMode': {'value': 'parameter_driven'},
            'startedAt': {'value': '2025-01-01T00:00:00Z'},
        },
        {
            'id': 'urn:ngsi-ld:Process:B',
            'processStatus': {'value': 'stopped'},
            'operationMode': {'value': 'geometry_driven'},
            'startedAt': {'value': '2025-01-02T00:00:00Z'},
        },
    ]

    def fake_get(url, headers=None, timeout=5):
        from tests.api.conftest import DummyResponse

        return DummyResponse(200, json_data=sample)

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    resp = await client.get(
        '/processes/list', params={'status_filter': 'active'}
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'success'
    assert body['total_count'] == 1
    assert body['processes'][0]['process_id'] == 'A'


@pytest.mark.asyncio
async def test_set_process_mode_success(monkeypatch, client):
    import robin.alert_engine as ae

    class FakeClient:
        def __init__(self, *_, **__):
            pass

        def set_operation_mode(self, process_id, mode):
            return True

    monkeypatch.setattr(ae, 'RobinFiwareClient', FakeClient)

    resp = await client.post(
        '/process/PROC-MODE/mode', json={'mode': 'geometry_driven'}
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'success'
    assert body['operation_mode'] == 'geometry_driven'


@pytest.mark.asyncio
async def test_set_process_mode_invalid_mode(client):
    resp = await client.post('/process/PROC-MODE/mode', json={'mode': 'bad_mode'})
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'error'
    assert 'Invalid mode' in body['message']
