import json


class DummyResponse:
    def __init__(self, status_code=200, json_data=None):
        self.status_code = status_code
        self._json = json_data
        # keep a short text body for debugging prints
        self.text = json.dumps(json_data) if json_data is not None else ''

    def json(self):
        return self._json


def test_fetch_all_measurements_temporal_values(monkeypatch):
    import robin.alert_engine as ae

    # Use compound processTelemetry with temporalValues (value + timestamp)
    entity = {
        'processTelemetry': {
            'values': [
                [
                    {
                        'height': 5.0,
                        'width': 8.0,
                        'speed': 10.0,
                    },
                    '2025-01-01T00:00:00Z',
                ],
                [
                    {
                        'height': 5.2,
                        'width': 8.1,
                    },
                    '2025-01-01T00:00:01Z',
                ],
            ]
        }
    }

    def fake_get(url, headers=None, params=None, timeout=10):
        return DummyResponse(200, json_data=entity)

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    series = ae.ENGINE.fetch_all_measurements('PROC')
    assert isinstance(series, list) and len(series) == 2
    assert series[0]['timestamp'] == '2025-01-01T00:00:00Z'
    assert series[0]['height'] == 5.0 and series[0]['width'] == 8.0
    assert series[0]['speed'] == 10.0


def test_fetch_all_measurements_full_temporal_fallback(monkeypatch):
    import robin.alert_engine as ae

    # First call returns empty processTelemetry values for temporalValues
    entity_empty = {'processTelemetry': {'values': []}}

    # Second call returns full temporal form: list of dicts with value+observedAt
    entity_full = {
        'processTelemetry': {
            'values': [
                {
                    'value': {'height': 5.0, 'width': 8.0},
                    'observedAt': '2025-01-01T00:00:00Z',
                },
                {
                    'value': {'height': 5.1, 'width': 8.05, 'current': 150.0},
                    'observedAt': '2025-01-01T00:00:01Z',
                },
            ]
        }
    }

    calls = {'n': 0}

    def fake_get(url, headers=None, params=None, timeout=10):
        calls['n'] += 1
        if calls['n'] == 1:
            return DummyResponse(200, json_data=entity_empty)
        return DummyResponse(200, json_data=entity_full)

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    series = ae.ENGINE.fetch_all_measurements('PROC')
    assert isinstance(series, list) and len(series) == 2
    assert series[0]['timestamp'] == '2025-01-01T00:00:00Z'
    assert series[1]['timestamp'] == '2025-01-01T00:00:01Z'
    assert series[1]['current'] == 150.0


def test_fetch_all_measurements_non200_returns_empty(monkeypatch):
    import robin.alert_engine as ae

    def fake_get(url, headers=None, params=None, timeout=10):
        # non-200 leads to None entity; called twice produces empty series
        return DummyResponse(500, json_data={'error': 'bad'})

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    series = ae.ENGINE.fetch_all_measurements('PROC')
    assert series == []


def test_fetch_all_measurements_scalar_temporal_values(monkeypatch):
    import robin.alert_engine as ae

    # Simulate Mintaka temporalValues for scalar Process attributes.
    entity = {
        'measuredHeight': {
            'values': [
                [5.0, '2025-01-01T00:00:00Z'],
                [5.2, '2025-01-01T00:00:01Z'],
            ]
        },
        'measuredWidth': {
            'values': [
                [8.0, '2025-01-01T00:00:00Z'],
                [8.1, '2025-01-01T00:00:01Z'],
            ]
        },
        'measuredCurrent': {
            'values': [[150.0, '2025-01-01T00:00:01Z']]
        },
    }

    def fake_get(url, headers=None, params=None, timeout=10):
        return DummyResponse(200, json_data=entity)

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    series = ae.ENGINE.fetch_all_measurements('PROC')
    assert isinstance(series, list) and len(series) == 2
    assert series[0]['timestamp'] == '2025-01-01T00:00:00Z'
    assert series[0]['height'] == 5.0 and series[0]['width'] == 8.0
    assert series[1]['timestamp'] == '2025-01-01T00:00:01Z'
    assert series[1]['height'] == 5.2 and series[1]['width'] == 8.1
    assert series[1]['current'] == 150.0


def test_fetch_all_measurements_scalar_full_temporal_fallback(monkeypatch):
    import robin.alert_engine as ae

    # First call (temporalValues) returns no values, forcing full temporal fallback.
    entity_empty = {'measuredHeight': {'values': []}}
    entity_full = {
        'measuredHeight': {
            'values': [
                {'value': 5.0, 'observedAt': '2025-01-01T00:00:00Z'},
                {'value': 5.1, 'observedAt': '2025-01-01T00:00:01Z'},
            ]
        },
        'measuredWidth': {
            'values': [
                {'value': 8.0, 'observedAt': '2025-01-01T00:00:00Z'},
                {'value': 8.2, 'observedAt': '2025-01-01T00:00:01Z'},
            ]
        },
    }

    calls = {'n': 0}

    def fake_get(url, headers=None, params=None, timeout=10):
        calls['n'] += 1
        if calls['n'] == 1:
            return DummyResponse(200, json_data=entity_empty)
        return DummyResponse(200, json_data=entity_full)

    monkeypatch.setattr(ae.requests, 'get', fake_get)

    series = ae.ENGINE.fetch_all_measurements('PROC')
    assert isinstance(series, list) and len(series) == 2
    assert series[0]['timestamp'] == '2025-01-01T00:00:00Z'
    assert series[0]['height'] == 5.0
    assert series[1]['timestamp'] == '2025-01-01T00:00:01Z'
    assert series[1]['width'] == 8.2
