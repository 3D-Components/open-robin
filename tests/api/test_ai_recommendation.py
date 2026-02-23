import pytest


@pytest.mark.asyncio
async def test_ai_recommendation_parameter_driven(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'predict_geometry_from_params',
        lambda params: {'height': 4.2, 'width': 5.1},
    )
    monkeypatch.setattr(
        ae.ENGINE.client, 'create_ai_recommendation', lambda *_: None
    )

    payload = {
        'process_id': 'P1',
        'mode': 'parameter_driven',
        'input_params': {'wireSpeed': 2.5, 'current': 110, 'voltage': 17},
    }
    resp = await client.post('/ai-recommendation', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    assert 'predicted_geometry' in data['recommendation']


@pytest.mark.asyncio
async def test_ai_recommendation_geometry_driven(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'recommend_params_for_geometry',
        lambda tg: {'wireSpeed': 2.8, 'current': 115, 'voltage': 17.5},
    )
    monkeypatch.setattr(
        ae.ENGINE.client, 'create_ai_recommendation', lambda *_: None
    )

    payload = {
        'process_id': 'P2',
        'mode': 'geometry_driven',
        'target_geometry': {'height': 5.0, 'width': 6.0},
    }
    resp = await client.post('/ai-recommendation', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    assert 'recommended_params' in data['recommendation']
