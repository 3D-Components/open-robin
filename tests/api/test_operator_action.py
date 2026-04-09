import pytest


@pytest.mark.asyncio
async def test_operator_action_ai_retry(monkeypatch, client):
    import robin.alert_engine as ae

    calls = {'recommend': 0, 'create_ai_recommendation': 0}

    def fake_recommend(target):
        calls['recommend'] += 1
        return {
            'wire_feed_speed_mpm_model_input': 12.3,
            'travel_speed_mps_model_input': 0.021,
            'arc_length_correction_mm_model_input': 1.5,
        }

    class FakeClient:
        def create_ai_recommendation(self, process_id, new_params):
            calls['create_ai_recommendation'] += 1

    monkeypatch.setattr(
        ae.ENGINE, 'recommend_params_for_geometry', fake_recommend
    )
    # Avoid leaking ENGINE.client across tests
    monkeypatch.setattr(ae.ENGINE, 'client', FakeClient())

    resp = await client.post(
        '/operator-action',
        params={'process_id': 'PROC', 'action_id': 'ai_retry'},
        json={'target_geometry': {'height': 5.0, 'width': 8.0}},
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'new_recommendation'
    assert calls['recommend'] == 1 and calls['create_ai_recommendation'] == 1
