import pytest


@pytest.mark.asyncio
async def test_get_process_status_success(monkeypatch, client):
    import robin.cli as rcli

    class FakeClient:
        def __init__(self, *_, **__):
            pass

        def get_process_status(self, pid):
            return (
                {
                    'process_id': pid,
                    'status': 'active',
                    'operation_mode': 'parameter_driven',
                    'started_at': '2025-01-01T00:00:00Z',
                    'stopped_at': None,
                    'stop_reason': None,
                },
                None,
            )

    monkeypatch.setattr(rcli, 'RobinFiwareClient', FakeClient)

    resp = await client.get('/process/PX/status')
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'success'
    assert body['process_data']['process_id'] == 'PX'
    assert body['process_data']['status'] == 'active'


@pytest.mark.asyncio
async def test_get_process_status_client_error(monkeypatch, client):
    import robin.cli as rcli

    class FakeClient:
        def __init__(self, *_, **__):
            pass

        def get_process_status(self, pid):
            return None, f'Process {pid} not found'

    monkeypatch.setattr(rcli, 'RobinFiwareClient', FakeClient)

    resp = await client.get('/process/P404/status')
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'error'
    assert 'not found' in body['message'].lower()


@pytest.mark.asyncio
async def test_get_process_status_exception(monkeypatch, client):
    import robin.cli as rcli

    class FakeClient:
        def __init__(self, *_, **__):
            pass

        def get_process_status(self, pid):
            raise RuntimeError('boom')

    monkeypatch.setattr(rcli, 'RobinFiwareClient', FakeClient)

    resp = await client.get('/process/PE/status')
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'error'
    assert 'Failed to get status' in body['message']
