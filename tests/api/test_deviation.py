import pytest


@pytest.mark.asyncio
async def test_check_deviation_param_requires_input(monkeypatch, client):
    import robin.alert_engine as ae

    # Ensure process is active so validation continues
    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_process_data',
        lambda pid: {'processStatus': {'value': 'active'}},
    )

    payload = {
        'process_id': 'P1',
        'mode': 'parameter_driven',
        # input_params omitted intentionally
        'measured_geometry': {'height': 5.5, 'width': 8.2},
    }
    resp = await client.post('/check-deviation', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'
    assert 'input_params' in data['message']


@pytest.mark.asyncio
async def test_check_deviation_param_no_data(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_process_data',
        lambda pid: {'processStatus': {'value': 'active'}},
    )
    monkeypatch.setattr(
        ae.ENGINE, 'fetch_latest_measurement', lambda pid: None
    )

    payload = {
        'process_id': 'P2',
        'mode': 'parameter_driven',
        'input_params': {'wireSpeed': 10, 'current': 150, 'voltage': 24},
    }
    resp = await client.post('/check-deviation', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'no_data'


@pytest.mark.asyncio
async def test_check_deviation_param_alert(monkeypatch, client):
    import robin.alert_engine as ae
    from robin.alert_engine import DeviationAlert

    class FakeClient:
        def __init__(self):
            self.alerts = []

        def create_alert(self, alert):
            self.alerts.append(alert)

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_process_data',
        lambda pid: {'processStatus': {'value': 'active'}},
    )
    # Provide measured geometry directly

    def fake_check(pid, params, measured, tol):
        alert = DeviationAlert(
            process_id=pid,
            deviation_type='height',
            expected_value={'height': 5.0, 'width': 8.0},
            measured_value={'height': 6.0, 'width': 8.0},
            deviation_percentage=20.0,
            recommended_actions=['tune speed'],
        )
        expected = {'height': 5.0, 'width': 8.0}
        return alert, expected

    monkeypatch.setattr(
        ae.ENGINE, 'check_parameter_driven_deviation', fake_check
    )
    # capture create_alert calls without leaking ENGINE state to other tests
    monkeypatch.setattr(ae.ENGINE, 'client', FakeClient())

    payload = {
        'process_id': 'P3',
        'mode': 'parameter_driven',
        'input_params': {'wireSpeed': 10, 'current': 150, 'voltage': 24},
        'measured_geometry': {'height': 6.0, 'width': 8.0},
        'tolerance': 10.0,
    }
    resp = await client.post('/check-deviation', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['deviation_percentage'] == 20.0
    assert data['expected_value'] == {'height': 5.0, 'width': 8.0}
    assert ae.ENGINE.client.alerts  # ensure create_alert invoked


@pytest.mark.asyncio
async def test_check_deviation_geometry_no_target(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_process_data',
        lambda pid: {'processStatus': {'value': 'active'}},
    )
    monkeypatch.setattr(ae.ENGINE, 'fetch_geometry_target', lambda pid: None)

    payload = {
        'process_id': 'PG',
        'mode': 'geometry_driven',
        'measured_geometry': {'height': 6.0, 'width': 8.0},
    }
    resp = await client.post('/check-deviation', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'
    assert 'No geometry target' in data['message']


@pytest.mark.asyncio
async def test_check_deviation_geometry_alert(monkeypatch, client):
    import robin.alert_engine as ae
    from robin.alert_engine import DeviationAlert

    class FakeClient:
        def __init__(self):
            self.alerts = []

        def create_alert(self, alert):
            self.alerts.append(alert)

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_process_data',
        lambda pid: {'processStatus': {'value': 'active'}},
    )
    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_geometry_target',
        lambda pid: {'height': 5.0, 'width': 8.0},
    )

    def fake_check(pid, target, measured, tol):
        alert = DeviationAlert(
            process_id=pid,
            deviation_type='width',
            expected_value={'height': 5.0, 'width': 8.0},
            measured_value={'height': 5.0, 'width': 9.0},
            deviation_percentage=12.5,
            recommended_actions=['reduce voltage'],
        )
        expected = {'height': 5.0, 'width': 8.0}
        return alert, expected

    monkeypatch.setattr(
        ae.ENGINE, 'check_geometry_driven_deviation', fake_check
    )
    # capture create_alert calls without leaking ENGINE state to other tests
    monkeypatch.setattr(ae.ENGINE, 'client', FakeClient())

    payload = {
        'process_id': 'PG',
        'mode': 'geometry_driven',
        'measured_geometry': {'height': 5.0, 'width': 9.0},
        'tolerance': 10.0,
    }
    resp = await client.post('/check-deviation', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['deviation_percentage'] == 12.5
    assert ae.ENGINE.client.alerts


@pytest.mark.asyncio
async def test_check_deviation_geometry_uses_ai_prediction_source(
    monkeypatch, client
):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_process_data',
        lambda pid: {
            'processStatus': {'value': 'active'},
            'operationMode': {'value': 'geometry_driven'},
        },
    )
    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_geometry_target',
        lambda pid: {'height': 5.0, 'width': 8.0},
    )
    monkeypatch.setattr(
        ae.ENGINE,
        'predict_geometry_from_params',
        lambda params: {'height': 5.5, 'width': 7.5},
    )

    seen_reference = {}

    def fake_check(pid, reference, measured, tol):
        seen_reference['value'] = reference
        return None, reference

    monkeypatch.setattr(ae.ENGINE, 'check_geometry_driven_deviation', fake_check)

    payload = {
        'process_id': 'PG-AI',
        'mode': 'geometry_driven',
        'input_params': {'wireSpeed': 10, 'current': 150, 'voltage': 24},
        'measured_geometry': {'height': 5.0, 'width': 8.2},
        'tolerance': 10.0,
    }
    resp = await client.post('/check-deviation', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'ok'
    assert data['expected_source'] == 'ai_prediction_from_params'
    assert data['expected_value'] == {'height': 5.5, 'width': 7.5}
    assert seen_reference['value'] == {'height': 5.5, 'width': 7.5}


@pytest.mark.asyncio
async def test_check_deviation_geometry_falls_back_to_target_source(
    monkeypatch, client
):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_process_data',
        lambda pid: {
            'processStatus': {'value': 'active'},
            'operationMode': {'value': 'geometry_driven'},
        },
    )
    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_geometry_target',
        lambda pid: {'height': 5.0, 'width': 8.0},
    )

    def fail_predict(_params):
        raise RuntimeError('model unavailable')

    monkeypatch.setattr(ae.ENGINE, 'predict_geometry_from_params', fail_predict)

    seen_reference = {}

    def fake_check(pid, reference, measured, tol):
        seen_reference['value'] = reference
        return None, reference

    monkeypatch.setattr(ae.ENGINE, 'check_geometry_driven_deviation', fake_check)

    payload = {
        'process_id': 'PG-TARGET',
        'mode': 'geometry_driven',
        'input_params': {'wireSpeed': 10, 'current': 150, 'voltage': 24},
        'measured_geometry': {'height': 5.0, 'width': 8.2},
        'tolerance': 10.0,
    }
    resp = await client.post('/check-deviation', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'ok'
    assert data['expected_source'] == 'target_geometry'
    assert data['expected_value'] == {'height': 5.0, 'width': 8.0}
    assert seen_reference['value'] == {'height': 5.0, 'width': 8.0}


@pytest.mark.asyncio
async def test_check_deviation_process_inactive(monkeypatch, client):
    import robin.alert_engine as ae

    monkeypatch.setattr(
        ae.ENGINE,
        'fetch_process_data',
        lambda pid: {'processStatus': {'value': 'stopped'}},
    )

    payload = {
        'process_id': 'PZ',
        'mode': 'parameter_driven',
        'input_params': {'wireSpeed': 10, 'current': 150, 'voltage': 24},
        'measured_geometry': {'height': 5.0, 'width': 8.0},
    }
    resp = await client.post('/check-deviation', json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'process_inactive'
