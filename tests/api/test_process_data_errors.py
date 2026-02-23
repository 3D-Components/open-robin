import pytest


@pytest.mark.asyncio
async def test_get_process_data_non200(monkeypatch, client):
    import robin.alert_engine as ae
    from tests.api.conftest import DummyResponse

    def fake_get(url, headers=None, timeout=5):
        return DummyResponse(500, text='boom')

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    resp = await client.get('/process/PX')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'


@pytest.mark.asyncio
async def test_get_process_data_exception(monkeypatch, client):
    import robin.alert_engine as ae

    def fake_get(url, headers=None, timeout=5):
        raise Exception('boom')

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    resp = await client.get('/process/PY')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'
    assert 'boom' in data['error']


@pytest.mark.asyncio
async def test_list_processes_non200(monkeypatch, client):
    import robin.alert_engine as ae
    from tests.api.conftest import DummyResponse

    def fake_get(url, headers=None, timeout=5):
        return DummyResponse(500, text='nope')

    monkeypatch.setattr(ae.requests, 'get', fake_get)
    resp = await client.get('/processes')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'


@pytest.mark.asyncio
async def test_list_processes_exception(monkeypatch, client):
    import robin.alert_engine as ae

    def fake_get(url, headers=None, timeout=5):
        raise Exception('fail')

    monkeypatch.setattr(ae.requests, 'get', fake_get)
    resp = await client.get('/processes')
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'
