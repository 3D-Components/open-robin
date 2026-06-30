import pytest


@pytest.mark.asyncio
async def test_root_lists_endpoints(client):
    resp = await client.get('/')
    assert resp.status_code == 200
    data = resp.json()
    assert 'endpoints' in data and isinstance(data['endpoints'], dict)
    assert '/create-process' in data['endpoints'].values()


@pytest.mark.asyncio
async def test_health_ok(monkeypatch, client):
    import robin.alert_engine as ae
    from tests.api.conftest import DummyResponse

    def fake_get(url, timeout=5):
        return DummyResponse(200, json_data={'orion': 'ok'})

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    resp = await client.get('/health')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'healthy'
    assert data['orion_connected'] is True


@pytest.mark.asyncio
async def test_health_unhealthy(monkeypatch, client):
    import robin.alert_engine as ae

    def fake_get(url, timeout=5):
        raise Exception('boom')

    monkeypatch.setattr(ae.requests, 'get', fake_get)
    resp = await client.get('/health')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'unhealthy'
    assert 'error' in data


@pytest.mark.asyncio
async def test_dashboard_missing(monkeypatch, client):
    import builtins

    def fake_open(*args, **kwargs):
        raise FileNotFoundError('missing')

    monkeypatch.setattr(builtins, 'open', fake_open)
    resp = await client.get('/dashboard')
    assert resp.status_code == 404
