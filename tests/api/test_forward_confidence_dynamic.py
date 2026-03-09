import pytest


@pytest.mark.asyncio
async def test_ai_recommendation_parameter_uses_dynamic_confidence(
    monkeypatch, client
):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'predict_geometry_with_confidence',
        lambda params: {
            'prediction': {'height': 4.4, 'width': 6.6},
            'confidence': 0.73,
            'uncertainty': {'height_std': 0.12, 'width_std': 0.2, 'relative': 0.08},
            'diagnostics': {'input_distance': 1.2, 'mc_samples': 20},
        },
    )
    monkeypatch.setattr(
        ae.ENGINE.client, 'create_ai_recommendation', lambda *_: None
    )

    payload = {
        'process_id': 'P-DYN',
        'mode': 'parameter_driven',
        'input_params': {'wireSpeed': 10.0, 'current': 150.0, 'voltage': 24.0},
    }
    resp = await client.post('/ai-recommendation', json=payload)
    assert resp.status_code == 200
    body = resp.json()
    assert body['status'] == 'success'
    rec = body['recommendation']
    assert rec['predicted_geometry'] == {'height': 4.4, 'width': 6.6}
    assert rec['confidence'] == 0.73
    assert rec['prediction_uncertainty']['relative'] == 0.08


@pytest.mark.asyncio
async def test_ai_models_predict_includes_dynamic_confidence(
    monkeypatch, client
):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'predict_geometry_with_confidence',
        lambda params: {
            'prediction': {'height': 5.0, 'width': 7.0},
            'confidence': 0.67,
            'uncertainty': {'height_std': 0.05, 'width_std': 0.09, 'relative': 0.03},
            'diagnostics': {'input_distance': 0.9, 'mc_samples': 20},
        },
    )

    payload = {'wireSpeed': 10.0, 'current': 150.0, 'voltage': 24.0}
    resp = await client.post('/ai/models/predict', json=payload)
    assert resp.status_code == 200
    body = resp.json()
    assert body['prediction'] == {'height': 5.0, 'width': 7.0}
    assert body['confidence'] == 0.67
    assert body['uncertainty']['relative'] == 0.03
