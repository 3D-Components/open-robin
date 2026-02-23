import pytest


@pytest.mark.asyncio
async def test_operator_action_ai_retry_missing_target_returns_error(client):
    # Send no target_geometry in body
    resp = await client.post(
        '/operator-action',
        params={'process_id': 'P', 'action_id': 'ai_retry'},
        json={},
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'
    assert 'target_geometry' in data['message']
