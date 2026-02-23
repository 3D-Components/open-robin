import json


class DummyResponse:
    def __init__(self, status_code=200, json_data=None):
        self.status_code = status_code
        self._json = json_data
        self.text = json.dumps(json_data) if json_data is not None else ''

    def json(self):
        return self._json


def test_fetch_geometry_target_success(monkeypatch):
    import robin.alert_engine as ae

    data = {
        'targetHeight': {'value': 5.0},
        'targetWidth': {'value': 8.0},
    }

    def fake_get(*args, **kwargs):
        return DummyResponse(200, json_data=data)

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)

    # Use a fresh engine instance with a minimal client to avoid cross-test state
    class DummyClient:
        orion_url = 'http://orion-ld:1026'
        headers = {}

    engine = ae.AlertEngine()
    engine.client = DummyClient()
    out = engine.fetch_geometry_target('PROC')
    assert out == {'height': 5.0, 'width': 8.0}


def test_fetch_geometry_target_non200(monkeypatch):
    import robin.alert_engine as ae

    def fake_get(*args, **kwargs):
        return DummyResponse(404, json_data={'error': 'not found'})

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)
    assert ae.ENGINE.fetch_geometry_target('X') is None


def test_fetch_latest_measurement_success(monkeypatch):
    import robin.alert_engine as ae

    entities = [
        {
            'measuredHeight': {
                'value': 4.9,
                'observedAt': '2025-01-01T00:00:00Z',
            },
            'measuredWidth': {
                'value': 7.8,
                'observedAt': '2025-01-01T00:00:00Z',
            },
        },
        {
            'measuredHeight': {
                'value': 5.1,
                'observedAt': '2025-01-01T00:00:02Z',
            },
            'measuredWidth': {
                'value': 8.2,
                'observedAt': '2025-01-01T00:00:02Z',
            },
            'measuredSpeed': {'value': 10.5},
            'measuredCurrent': {'value': 150.0},
            'measuredVoltage': {'value': 24.0},
        },
    ]

    def fake_get(*args, **kwargs):
        return DummyResponse(200, json_data=entities)

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)

    class DummyClient:
        orion_url = 'http://orion-ld:1026'
        headers = {}

    engine = ae.AlertEngine()
    engine.client = DummyClient()
    out = engine.fetch_latest_measurement('P')
    assert out is not None
    assert out['height'] == 5.1 and out['width'] == 8.2
    assert (
        out['speed'] == 10.5
        and out['current'] == 150.0
        and out['voltage'] == 24.0
    )
    assert out['timestamp'] == '2025-01-01T00:00:02Z'


def test_fetch_latest_measurement_non200(monkeypatch):
    import robin.alert_engine as ae

    def fake_get(*args, **kwargs):
        return DummyResponse(500, json_data={'error': 'boom'})

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)
    assert ae.ENGINE.fetch_latest_measurement('P') is None


def test_fetch_latest_measurement_exception(monkeypatch):
    import robin.alert_engine as ae

    def fake_get(*args, **kwargs):
        raise Exception('oops')

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)
    assert ae.ENGINE.fetch_latest_measurement('P') is None
