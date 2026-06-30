import pytest


@pytest.mark.asyncio
async def test_report_process_error_records_alert(monkeypatch, client):
    """POST /process/{id}/error records a process_error alert and echoes the reason."""
    import robin.alert_engine as ae

    created = []

    class FakeClient:
        def create_alert(self, alert):
            created.append(alert)

    # Avoid leaking ENGINE.client across tests
    monkeypatch.setattr(ae.ENGINE, 'client', FakeClient())

    resp = await client.post(
        '/process/PROC/error',
        json={'message': 'Operator aborted', 'reason': 'operator_abort'},
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'recorded'
    assert data['process_id'] == 'PROC'
    assert data['error'] == {'reason': 'operator_abort', 'message': 'Operator aborted'}

    assert len(created) == 1
    alert = created[0]
    assert alert.process_id == 'PROC'
    assert alert.deviation_type == 'process_error'
    # The abort message is surfaced as a recommended action label.
    assert any(a['label'] == 'Operator aborted' for a in alert.recommended_actions)


@pytest.mark.asyncio
async def test_report_process_error_uses_defaults(monkeypatch, client):
    """An empty body falls back to the ProcessErrorRequest defaults."""
    import robin.alert_engine as ae

    class FakeClient:
        def create_alert(self, alert):
            pass

    monkeypatch.setattr(ae.ENGINE, 'client', FakeClient())

    resp = await client.post('/process/PROC/error', json={})
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'recorded'
    assert data['error']['reason'] == 'operator_abort'


@pytest.mark.asyncio
async def test_report_process_error_handles_engine_failure(monkeypatch, client):
    """A failure while recording the alert is returned as a structured error."""
    import robin.alert_engine as ae

    class FakeClient:
        def create_alert(self, alert):
            raise RuntimeError('orion down')

    monkeypatch.setattr(ae.ENGINE, 'client', FakeClient())

    resp = await client.post('/process/PROC/error', json={'message': 'boom'})
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'
    assert 'orion down' in data['error']
