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
        'input_params': {
            'wire_feed_speed_mpm_model_input': 10.0,
            'travel_speed_mps_model_input': 0.02,
            'arc_length_correction_mm_model_input': 0.0,
        },
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
        lambda tg: {
            'wire_feed_speed_mpm_model_input': 10.5,
            'travel_speed_mps_model_input': 0.021,
            'arc_length_correction_mm_model_input': 1.0,
        },
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
