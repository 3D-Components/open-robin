import pytest


@pytest.mark.asyncio
async def test_set_process_target_create_success(monkeypatch, client):
    import robin.alert_engine as ae

    class FakeClient:
        def __init__(self, *_, **__):
            self.orion_url = 'http://orion-ld:1026'
            self.headers = {'NGSILD-Tenant': 'robin'}
            self.context = {'@context': []}

        def create_geometry_target(self, process_id, height, width):
            return True

        def set_operation_mode(self, process_id, mode):
            return True

    monkeypatch.setattr(ae, 'RobinFiwareClient', FakeClient)

    resp = await client.post(
        '/process/PROC/target', json={'height': 5.0, 'width': 8.0}
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    assert data['process_id'] == 'PROC'
    assert data['target_geometry'] == {'height': 5.0, 'width': 8.0}
    assert data['mode_set'] == 'geometry_driven'


@pytest.mark.asyncio
async def test_set_process_target_fallback_patch_success(monkeypatch, client):
    import robin.alert_engine as ae
    from tests.api.conftest import DummyResponse

    class FakeClient:
        def __init__(self, *_, **__):
            self.orion_url = 'http://orion-ld:1026'
            self.headers = {'NGSILD-Tenant': 'robin'}
            self.context = {'@context': []}

        def create_geometry_target(self, process_id, height, width):
            return False  # force fallback path

        def set_operation_mode(self, process_id, mode):
            return True

    def fake_patch(url, headers=None, json=None, timeout=5):
        return DummyResponse(204)

    monkeypatch.setattr(ae, 'RobinFiwareClient', FakeClient)
    monkeypatch.setattr(ae.requests, 'patch', fake_patch)

    resp = await client.post(
        '/process/PROC/target', json={'height': 6.0, 'width': 9.0}
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    assert data['target_geometry'] == {'height': 6.0, 'width': 9.0}
    assert data['mode_set'] == 'geometry_driven'


@pytest.mark.asyncio
async def test_set_process_target_mode_set_failure(monkeypatch, client):
    import robin.alert_engine as ae

    class FakeClient:
        def __init__(self, *_, **__):
            self.orion_url = 'http://orion-ld:1026'
            self.headers = {'NGSILD-Tenant': 'robin'}
            self.context = {'@context': []}

        def create_geometry_target(self, process_id, height, width):
            return True

        def set_operation_mode(self, process_id, mode):
            return False

    monkeypatch.setattr(ae, 'RobinFiwareClient', FakeClient)

    resp = await client.post(
        '/process/PROC/target', json={'height': 5.5, 'width': 8.5}
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    assert data['mode_set'] == 'unchanged'


@pytest.mark.asyncio
async def test_set_process_target_total_failure(monkeypatch, client):
    import robin.alert_engine as ae
    from tests.api.conftest import DummyResponse

    class FakeClient:
        def __init__(self, *_, **__):
            self.orion_url = 'http://orion-ld:1026'
            self.headers = {'NGSILD-Tenant': 'robin'}
            self.context = {'@context': []}

        def create_geometry_target(self, process_id, height, width):
            return False

        def set_operation_mode(self, process_id, mode):
            return True

    def fake_patch(url, headers=None, json=None, timeout=5):
        return DummyResponse(500, text='err')

    monkeypatch.setattr(ae, 'RobinFiwareClient', FakeClient)
    monkeypatch.setattr(ae.requests, 'patch', fake_patch)

    resp = await client.post(
        '/process/PROC/target', json={'height': 5.0, 'width': 8.0}
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'error'
    assert 'Failed to set target geometry' in data['message']
