import pytest

from tests.api.conftest import DummyResponse


@pytest.mark.asyncio
async def test_health_reports_unhealthy_on_timeout(monkeypatch, client):
    import requests
    import robin.alert_engine as ae

    def boom(*_, **__):
        raise requests.Timeout('orion version timeout')

    monkeypatch.setattr(ae.requests, 'get', boom)

    resp = await client.get('/health')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'unhealthy'
    assert 'error' in data


@pytest.mark.asyncio
async def test_check_deviation_no_measurements(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_process_data',
        lambda pid: {
            'processStatus': {'value': 'active'},
            'operationMode': {'value': 'parameter_driven'},
        },
    )
    monkeypatch.setattr(ae.ENGINE, 'fetch_latest_measurement', lambda *_: None)

    payload = {
        'process_id': 'PX',
        'mode': 'parameter_driven',
        'input_params': {'wireSpeed': 1.0, 'current': 1.0, 'voltage': 1.0},
    }
    resp = await client.post('/check-deviation', json=payload)
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'no_data'


@pytest.mark.asyncio
async def test_ai_model_select_missing_returns_404(client):
    payload = {'path': '/definitely/not/here/model.pt'}
    resp = await client.post('/ai/models/select', json=payload)
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_process_not_found_returns_not_found(monkeypatch, client):
    import robin.alert_engine as ae

    def fake_get(url, *args, **kwargs):
        if '/ngsi-ld/v1/entities/urn:ngsi-ld:Process:' in url:
            return DummyResponse(404, json_data={'error': 'not found'})
        return DummyResponse(404)

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    resp = await client.get('/process/NOPE')
    assert resp.status_code == 200  # API returns JSON body with status
    body = resp.json()
    assert body.get('status') == 'not_found'
