import pytest


@pytest.mark.asyncio
async def test_get_process_data_happy_path(monkeypatch, client):
    import robin.alert_engine as ae
    from tests.api.conftest import DummyResponse

    # Base process entity
    base = {
        'id': 'urn:ngsi-ld:Process:PX',
        'processStatus': {'value': 'active'},
        'operationMode': {'value': 'parameter_driven'},
        'startedAt': {'value': '2025-01-01T00:00:00Z'},
    }

    def fake_get(url, headers=None, timeout=5):
        return DummyResponse(200, json_data=base)

    # Latest measurement to be injected
    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_latest_measurement',
        lambda pid: {
            'height': 5.2,
            'width': 8.1,
            'speed': 10.0,
            'current': 150.0,
            'voltage': 24.0,
            'timestamp': '2025-01-01T00:10:00Z',
        },
    )

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    resp = await client.get('/process/PX')
    assert resp.status_code == 200
    data = resp.json()
    # Base fields preserved
    assert data['id'].endswith(':Process:PX')
    # Enriched measurement fields present
    assert data['measuredHeight']['value'] == 5.2
    assert data['measuredWidth']['value'] == 8.1
    assert data['measuredSpeed']['value'] == 10.0
    assert data['measuredCurrent']['value'] == 150.0
    assert data['measuredVoltage']['value'] == 24.0
    assert data['measuredHeight']['observedAt'] == '2025-01-01T00:10:00Z'
