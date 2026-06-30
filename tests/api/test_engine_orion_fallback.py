import json


class DummyResponse:
    def __init__(self, status_code=200, json_data=None):
        self.status_code = status_code
        self._json = json_data
        self.text = json.dumps(json_data) if json_data is not None else ''

    def json(self):
        return self._json


def _fresh_engine_with_client():
    import robin.alert_engine as ae

    class DummyClient:
        orion_url = 'http://orion-ld:1026'
        headers = {'NGSILD-Tenant': 'robin'}

    eng = ae.AlertEngine()
    eng.client = DummyClient()
    return eng


def test_fetch_all_measurements_orion_success_sorted_and_optional(monkeypatch):

    entities = [
        {
            'measuredHeight': {
                'value': 4.9,
                'observedAt': '2025-01-01T00:00:01Z',
            },
            'measuredWidth': {
                'value': 7.8,
                'observedAt': '2025-01-01T00:00:01Z',
            },
            'measuredCurrent': {'value': 150.0},
        },
        {
            'measuredHeight': {
                'value': 5.2,
                'observedAt': '2025-01-01T00:00:00Z',
            },
            'measuredWidth': {
                'value': 8.1,
                'observedAt': '2025-01-01T00:00:00Z',
            },
            'measuredSpeed': {'value': 10.5},
            'measuredVoltage': {'value': 24.0},
        },
    ]

    def fake_get(*args, **kwargs):
        return DummyResponse(200, json_data=entities)

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)
    engine = _fresh_engine_with_client()
    series = engine.fetch_all_measurements_orion('PROC')

    # Sorted ascending by timestamp
    assert [row['timestamp'] for row in series] == [
        '2025-01-01T00:00:00Z',
        '2025-01-01T00:00:01Z',
    ]
    # Optional attributes present when available
    assert 'speed' in series[0] and series[0]['speed'] == 10.5
    assert 'voltage' in series[0] and series[0]['voltage'] == 24.0
    assert 'current' in series[1] and series[1]['current'] == 150.0


def test_fetch_all_measurements_orion_uses_width_observedAt(monkeypatch):
    entities = [
        {
            # missing measuredHeight.observedAt -> should fallback to measuredWidth.observedAt
            'measuredHeight': {'value': 4.0},
            'measuredWidth': {
                'value': 7.0,
                'observedAt': '2025-01-01T00:00:03Z',
            },
        }
    ]

    def fake_get(*args, **kwargs):
        return DummyResponse(200, json_data=entities)

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)
    engine = _fresh_engine_with_client()
    series = engine.fetch_all_measurements_orion('PROC')

    assert len(series) == 1
    assert series[0]['timestamp'] == '2025-01-01T00:00:03Z'
    # height/width values still mapped
    assert series[0]['height'] == 4.0
    assert series[0]['width'] == 7.0


def test_fetch_all_measurements_orion_non200_returns_empty(monkeypatch):
    def fake_get(*args, **kwargs):
        return DummyResponse(500, json_data={'error': 'bad'})

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)
    engine = _fresh_engine_with_client()
    assert engine.fetch_all_measurements_orion('PROC') == []


def test_fetch_all_measurements_orion_exception_returns_empty(monkeypatch):
    def fake_get(*args, **kwargs):
        raise Exception('boom')

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)
    engine = _fresh_engine_with_client()
    assert engine.fetch_all_measurements_orion('PROC') == []
