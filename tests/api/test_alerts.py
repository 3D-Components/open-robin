import pytest


@pytest.mark.asyncio
async def test_process_alerts_success(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_process_alerts',
        lambda *_, **__: [
            {
                'id': 'urn:ngsi-ld:Alert:PROC-1',
                'timestamp': '2026-01-01T00:00:00Z',
                'deviation_type': 'geometry',
                'deviation_percentage': 21.3,
                'expected_value': {'height': 5.0, 'width': 6.0},
                'measured_value': {'height': 3.8, 'width': 7.3},
                'recommended_actions': ['manual_adjust'],
            }
        ],
    )

    resp = await client.get('/process/PROC/alerts')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    assert data['process_id'] == 'PROC'
    assert data['count'] == 1
    assert data['alerts'][0]['deviation_percentage'] == 21.3


@pytest.mark.asyncio
async def test_process_alerts_engine_exception(monkeypatch, client):
    import robin.alert_engine as ae

    def boom(*args, **kwargs):
        raise Exception('alert fetch failed')

    monkeypatch.setattr(ae.ENGINE, 'fetch_process_alerts', boom)

    resp = await client.get('/process/PROC/alerts')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'
    assert 'Failed to fetch alerts' in data['message']
