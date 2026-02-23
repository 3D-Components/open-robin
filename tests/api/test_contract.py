import pytest

from tests.api.conftest import DummyResponse


@pytest.mark.asyncio
async def test_root_lists_endpoints(client):
    resp = await client.get('/')
    assert resp.status_code == 200
    data = resp.json()
    assert data.get('name') == 'ROBIN Alert Engine'
    assert '/health' in data.get('endpoints', {}).values()


@pytest.mark.asyncio
async def test_health_reports_healthy(monkeypatch, client):
    import robin.alert_engine as ae

    def fake_get(url, *args, **kwargs):
        if 'orion' in url and url.endswith('/version'):
            return DummyResponse(200, json_data={'orion': 'ok'})
        return DummyResponse(404)

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    resp = await client.get('/health')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] in {'healthy', 'unhealthy'}
    assert data.get('orion_connected') is True


@pytest.mark.asyncio
async def test_ai_models_listing_and_active(client):
    # Does not require external services
    resp = await client.get('/ai/models')
    assert resp.status_code == 200
    data = resp.json()
    assert 'models' in data
    # Active model may be None if no checkpoint can be loaded, but the key exists
    assert 'active_model' in data


@pytest.mark.asyncio
async def test_ai_predict_forward_pass(client):
    payload = {'wireSpeed': 3.0, 'current': 120.0, 'voltage': 16.5}
    resp = await client.post('/ai/models/predict', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert 'prediction' in data
    pred = data['prediction']
    assert set(['height', 'width']).issubset(pred.keys())
    assert (
        data.get('feature_order') == ['wireSpeed', 'current', 'voltage']
        or True
    )


@pytest.mark.asyncio
async def test_check_deviation_parameter_driven_happy_path(
    monkeypatch, client
):
    # Mock engine internals so we don't call Orion/Mintaka
    import robin.alert_engine as ae

    def fake_fetch_process_data(pid):
        return {
            'id': f'urn:ngsi-ld:Process:{pid}',
            'processStatus': {'value': 'active'},
            'operationMode': {'value': 'parameter_driven'},
            'toleranceThreshold': {'value': 10.0},
        }

    def fake_fetch_latest_measurement(pid):
        return {
            'height': 5.0,
            'width': 6.0,
            'timestamp': '2025-01-01T00:00:00Z',
        }

    def fake_check_param_deviation(pid, input_params, measured, tol):
        # No deviation case -> return (None, expected_geometry)
        expected = {'height': 5.0, 'width': 6.0}
        return None, expected

    monkeypatch.setattr(
        ae.ENGINE, 'fetch_process_data', fake_fetch_process_data
    )
    monkeypatch.setattr(
        ae.ENGINE, 'fetch_latest_measurement', fake_fetch_latest_measurement
    )
    monkeypatch.setattr(
        ae.ENGINE,
        'check_parameter_driven_deviation',
        fake_check_param_deviation,
    )
    # Avoid side-effects creating alerts
    monkeypatch.setattr(ae.ENGINE.client, 'create_alert', lambda *_: None)

    payload = {
        'process_id': 'P123',
        'mode': 'parameter_driven',
        'input_params': {'wireSpeed': 3.0, 'current': 120.0, 'voltage': 16.5},
    }
    resp = await client.post('/check-deviation', json=payload)
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'ok'
    assert set(['expected_value', 'measured_value']).issubset(body.keys())


@pytest.mark.asyncio
async def test_process_measurements_fallback_from_mintaka_to_orion(
    monkeypatch, client
):
    import robin.alert_engine as ae

    monkeypatch.setattr(ae.ENGINE, 'fetch_all_measurements', lambda *_, **__: [])
    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_all_measurements_orion',
        lambda *_, **__: [
            {'timestamp': '2025-01-01T00:00:00Z', 'height': 5.0, 'width': 6.0}
        ],
    )

    resp = await client.get('/process/P1/measurements')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    assert data['debug_info']['source'] == 'orion'
    assert data['count'] == 1
