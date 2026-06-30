import json
import sys
import types
from datetime import datetime, timezone


class DummyResponse:
    def __init__(self, status_code=200, json_data=None, text=None):
        self.status_code = status_code
        self._json = json_data
        self.text = text if text is not None else (
            json.dumps(json_data) if json_data is not None else ''
        )

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
            'inputParams': {
                'value': {
                    'wire_feed_speed_mpm_model_input': 10.5,
                    'travel_speed_mps_model_input': 0.021,
                    'arc_length_correction_mm_model_input': 1.2,
                }
            },
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
    assert out['input_params']['travel_speed_mps_model_input'] == 0.021
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


def _fresh_engine(ae):
    class DummyClient:
        orion_url = 'http://orion-ld:1026'
        headers = {}

    engine = ae.AlertEngine()
    engine.client = DummyClient()
    return engine


def test_fetch_all_measurements_parses_compound_and_scalar_temporal(
    monkeypatch,
):
    import robin.alert_engine as ae

    temporal_entity = {
        'urn:robin:processTelemetry': {
            'values': [
                [
                    {
                        'height': 2.0,
                        'width': 5.0,
                        'wireSpeed': 10.0,
                        'current': 150.0,
                        'voltage': 24.0,
                        'inputParams': {
                            'wire_feed_speed_mpm_model_input': 10.0,
                            'travel_speed_mps_model_input': 0.02,
                            'arc_length_correction_mm_model_input': 1.0,
                        },
                    },
                    '2025-01-01T00:00:01Z',
                ],
                [{'measuredHeight': 2.5, 'measuredWidth': 5.5}, '2025-01-01T00:00:03Z'],
            ]
        },
        'measuredHeight': {
            'values': [
                {'value': 2.2, 'observedAt': '2025-01-01T00:00:02Z'},
                [True, '2025-01-01T00:00:04Z'],
            ]
        },
        'measuredWidth': {'values': [[5.2, '2025-01-01T00:00:02Z']]},
        'measuredSpeed': {'values': [[0.02, '2025-01-01T00:00:02Z']]},
        'measuredCurrent': {'values': [[151.0, '2025-01-01T00:00:02Z']]},
        'measuredVoltage': {'values': [[24.5, '2025-01-01T00:00:02Z']]},
        'inputParams': {
            'values': [
                [
                    {
                        'wire_feed_speed_mpm_model_input': 10.1,
                        'travel_speed_mps_model_input': 0.021,
                        'arc_length_correction_mm_model_input': 1.1,
                    },
                    '2025-01-01T00:00:02Z',
                ],
                ['not-a-dict', '2025-01-01T00:00:05Z'],
            ]
        },
    }
    seen = {}

    def fake_get(url, headers=None, params=None, timeout=10):
        seen['url'] = url
        seen['headers'] = headers
        seen['params'] = params
        return DummyResponse(200, json_data=temporal_entity)

    monkeypatch.setenv('MINTAKA_URL', 'http://mintaka:8080')
    monkeypatch.setenv('NGSILD_TENANT', 'testtenant')
    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)

    engine = _fresh_engine(ae)
    out = engine.fetch_all_measurements('PROC', last_n=2)

    assert seen['headers']['NGSILD-Tenant'] == 'testtenant'
    assert seen['params']['lastN'] == '2'
    assert len(out) == 2
    assert out[0]['timestamp'] == '2025-01-01T00:00:02Z'
    assert out[0]['height'] == 2.2
    assert out[0]['input_params']['travel_speed_mps_model_input'] == 0.021
    assert out[1]['timestamp'] == '2025-01-01T00:00:03Z'
    assert out[1]['width'] == 5.5


def test_fetch_all_measurements_falls_back_across_temporal_queries(
    monkeypatch,
):
    import robin.alert_engine as ae

    calls = []

    def fake_get(url, headers=None, params=None, timeout=10):
        calls.append((params.get('timeproperty'), params.get('options')))
        if len(calls) < 4:
            return DummyResponse(404, json_data={'error': 'not found'})
        return DummyResponse(
            200,
            json_data={
                'measuredHeight': {'values': [[3.0, '2025-01-01T00:00:06Z']]},
                'measuredWidth': {'values': [[6.0, '2025-01-01T00:00:06Z']]},
            },
        )

    monkeypatch.delenv('NGSILD_TENANT', raising=False)
    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)

    engine = _fresh_engine(ae)
    out = engine.fetch_all_measurements('PROC')

    assert calls == [
        ('observedAt', 'temporalValues'),
        ('observedAt', None),
        ('modifiedAt', 'temporalValues'),
        ('modifiedAt', None),
    ]
    assert out == [
        {
            'timestamp': '2025-01-01T00:00:06Z',
            'height': 3.0,
            'width': 6.0,
            'speed': None,
            'current': None,
            'voltage': None,
            'input_params': {},
        }
    ]


def test_fetch_all_measurements_handles_bad_mintaka_responses(monkeypatch):
    import robin.alert_engine as ae

    class BadJsonResponse(DummyResponse):
        def json(self):
            raise ValueError('bad json')

    responses = [
        DummyResponse(500, json_data={'error': 'bad'}, text='server failed'),
        BadJsonResponse(200),
    ]

    def fake_get(*_args, **_kwargs):
        return responses.pop(0) if responses else DummyResponse(404)

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)

    engine = _fresh_engine(ae)
    assert engine.fetch_all_measurements('PROC') == []


def test_fetch_all_measurements_orion_parses_sorted_entities(monkeypatch):
    import robin.alert_engine as ae

    entities = [
        {
            'measuredHeight': {'value': 3.0, 'observedAt': '2025-01-01T00:00:03Z'},
            'measuredWidth': {'value': 6.0, 'observedAt': '2025-01-01T00:00:03Z'},
            'measuredSpeed': {'value': 0.02},
            'inputParams': {
                'value': {
                    'wire_feed_speed_mpm_model_input': 10.0,
                    'travel_speed_mps_model_input': 0.02,
                    'arc_length_correction_mm_model_input': 1.0,
                }
            },
        },
        {
            'measuredHeight': {'value': 2.0, 'observedAt': '2025-01-01T00:00:01Z'},
            'measuredWidth': {'value': 5.0, 'observedAt': '2025-01-01T00:00:01Z'},
            'measuredCurrent': {'value': 150.0},
            'measuredVoltage': {'value': 24.0},
        },
        {
            'measuredHeight': {'value': 99.0},
            'measuredWidth': {'value': 99.0},
        },
    ]

    monkeypatch.setattr(
        'robin.alert_engine.requests.get',
        lambda *_args, **_kwargs: DummyResponse(200, json_data=entities),
    )

    engine = _fresh_engine(ae)
    out = engine.fetch_all_measurements_orion('PROC')

    assert [row['timestamp'] for row in out] == [
        '2025-01-01T00:00:01Z',
        '2025-01-01T00:00:03Z',
    ]
    assert out[0]['current'] == 150.0
    assert out[1]['input_params']['arc_length_correction_mm_model_input'] == 1.0


def test_fetch_measurements_from_troe_parses_rows(monkeypatch):
    import robin.alert_engine as ae

    class FakeCursor:
        def __init__(self):
            self.query = None

        def execute(self, query, params):
            self.query = query
            self.params = params

        def fetchall(self):
            return [
                (
                    {
                        'height': 3.0,
                        'width': 6.0,
                        'speed': 0.02,
                        'current': 151.0,
                        'voltage': 24.5,
                    },
                    datetime(2025, 1, 1, 0, 0, 3, tzinfo=timezone.utc),
                ),
                (
                    {
                        'height': 2.0,
                        'width': 5.0,
                        'speed': 0.01,
                        'current': 150.0,
                        'voltage': 24.0,
                    },
                    datetime(2025, 1, 1, 0, 0, 1),
                ),
            ]

        def close(self):
            pass

    class FakeConnection:
        def __init__(self):
            self.cursor_obj = FakeCursor()

        def cursor(self, cursor_factory=None):
            return self.cursor_obj

        def close(self):
            pass

    fake_psycopg2 = types.ModuleType('psycopg2')
    fake_psycopg2.connect = lambda **_kwargs: FakeConnection()
    fake_extras = types.ModuleType('psycopg2.extras')
    fake_extras.DictCursor = object
    fake_psycopg2.extras = fake_extras
    monkeypatch.setitem(sys.modules, 'psycopg2', fake_psycopg2)
    monkeypatch.setitem(sys.modules, 'psycopg2.extras', fake_extras)

    engine = _fresh_engine(ae)
    out = engine.fetch_measurements_from_troe('PROC', last_n=2)

    assert [row['timestamp'] for row in out] == [
        '2025-01-01T00:00:01Z',
        '2025-01-01T00:00:03Z',
    ]
    assert out[1]['height'] == 3.0


def test_fetch_process_alerts_paginates_and_normalizes(monkeypatch):
    import robin.alert_engine as ae

    first_page = [
        {
            'id': 'urn:ngsi-ld:Alert:A2',
            'timestamp': {'value': '2025-01-01T00:00:02Z'},
            'deviationType': {'value': 'width'},
            'expectedValue': {'value': {'width': 5.0}},
            'measuredValue': {'value': {'width': 6.0}},
            'deviationPercentage': {'value': 20.0},
            'recommendedActions': {'value': 'adjust'},
        },
        {
            'id': 'urn:ngsi-ld:Alert:A1',
            'createdAt': {'value': '2025-01-01T00:00:01Z'},
            'deviationType': {'value': 'height'},
            'expectedValue': {'value': 'bad'},
            'measuredValue': {'value': {'height': 4.0}},
            'deviationPercentage': {'value': 10.0},
            'recommendedActions': {'value': ['monitor']},
        },
    ]
    calls = []

    def fake_get(url, headers=None, params=None, timeout=10):
        calls.append(params)
        return DummyResponse(200, json_data=first_page if len(calls) == 1 else [])

    monkeypatch.setattr('robin.alert_engine.requests.get', fake_get)

    engine = _fresh_engine(ae)
    out = engine.fetch_process_alerts('PROC', last_n=1)

    assert len(calls) == 1
    assert out[0]['id'] == 'urn:ngsi-ld:Alert:A2'
    assert out[0]['recommended_actions'] == ['adjust']
    assert out[0]['expected_value'] == {'width': 5.0}
