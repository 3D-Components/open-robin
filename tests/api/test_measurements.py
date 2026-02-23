import pytest


@pytest.mark.asyncio
async def test_measurements_mintaka_source(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_all_measurements',
        lambda *_, **__: [
            {
                'height': 5.0,
                'width': 8.0,
                'timestamp': '2025-01-01T00:00:00Z',
            }
        ],
    )
    monkeypatch.setattr(
        ae.ENGINE, 'fetch_all_measurements_orion', lambda *_, **__: []
    )

    resp = await client.get('/process/PROC/measurements')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    assert data['debug_info']['source'] == 'mintaka'
    assert data['count'] == 1


@pytest.mark.asyncio
async def test_measurements_orion_fallback(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE, 'fetch_all_measurements', lambda *_, **__: []
    )
    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_all_measurements_orion',
        lambda *_, **__: [
            {
                'height': 4.9,
                'width': 7.8,
                'timestamp': '2025-01-01T00:00:01Z',
            }
        ],
    )

    resp = await client.get('/process/PROC/measurements')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    assert data['debug_info']['source'] == 'orion'
    assert data['count'] == 1


@pytest.mark.asyncio
async def test_measurements_none(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE, 'fetch_all_measurements', lambda *_, **__: []
    )
    monkeypatch.setattr(
        ae.ENGINE, 'fetch_all_measurements_orion', lambda *_, **__: []
    )

    resp = await client.get('/process/PROC/measurements')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    assert data['debug_info']['source'] == 'none'
    assert data['count'] == 0


@pytest.mark.asyncio
async def test_measurements_engine_exception(monkeypatch, client):
    import robin.alert_engine as ae

    def boom(*args, **kwargs):
        raise Exception('engine blew up')

    monkeypatch.setattr(ae.ENGINE, 'fetch_all_measurements', boom)

    resp = await client.get('/process/PROC/measurements')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'
    assert 'Failed to fetch measurements' in data['message']


@pytest.mark.asyncio
async def test_measurements_without_last_does_not_force_default_limit(
    monkeypatch, client
):
    import robin.alert_engine as ae

    seen = {'last_n': 'unset'}

    def fake_fetch(pid, last_n=None):
        seen['last_n'] = last_n
        return []

    monkeypatch.setattr(ae.ENGINE, 'fetch_all_measurements', fake_fetch)
    monkeypatch.setattr(
        ae.ENGINE, 'fetch_all_measurements_orion', lambda *_, **__: []
    )

    resp = await client.get('/process/PROC/measurements')
    assert resp.status_code == 200
    assert seen['last_n'] is None
