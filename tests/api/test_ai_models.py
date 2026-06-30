import pytest


@pytest.mark.asyncio
async def test_ai_models_active_status(client):
    resp = await client.get('/ai/models/active')
    assert resp.status_code == 200
    body = resp.json()
    assert body.get('status') in {'active', 'inactive'}
    if body.get('status') == 'active':
        assert 'model' in body


@pytest.mark.asyncio
async def test_ai_models_directories(client):
    resp = await client.get('/ai/models/directories')
    assert resp.status_code == 200
    body = resp.json()
    assert isinstance(body.get('directories'), list)
    # Must be strings
    assert (
        all(isinstance(p, str) for p in body['directories'])
        or body['directories'] == []
    )


@pytest.mark.asyncio
async def test_ai_models_select_404(monkeypatch, client):
    import robin.alert_engine as ae

    def fake_load(path):
        raise FileNotFoundError('missing')

    monkeypatch.setattr(ae.ENGINE, 'load_model_from_path', fake_load)

    resp = await client.post('/ai/models/select', json={'path': 'nope.ckpt'})
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_ai_models_select_400(monkeypatch, client):
    import robin.alert_engine as ae

    def fake_load(path):
        raise Exception('bad checkpoint')

    monkeypatch.setattr(ae.ENGINE, 'load_model_from_path', fake_load)

    resp = await client.post('/ai/models/select', json={'path': 'bad.ckpt'})
    assert resp.status_code == 400
