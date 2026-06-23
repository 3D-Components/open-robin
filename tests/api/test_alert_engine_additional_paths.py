from __future__ import annotations

import types

import pytest


@pytest.mark.asyncio
async def test_profile_intent_and_dashboard_success(monkeypatch, client):
    import builtins
    import robin.alert_engine as ae

    class FakeClient:
        def __init__(self, *_, **__):
            self.calls = []

        def patch_process_intent(self, process_id, intent, data):
            self.calls.append((process_id, intent, data))
            return True

    monkeypatch.setattr(ae, 'RobinFiwareClient', FakeClient)

    profile_resp = await client.get('/profile')
    assert profile_resp.status_code == 200
    assert isinstance(profile_resp.json(), dict)

    intent_resp = await client.post(
        '/intent',
        json={
            'intent': 'START',
            'process_id': 'PROC',
            'data': {'source': 'test'},
        },
    )
    assert intent_resp.status_code == 200
    assert intent_resp.json()['status'] == 'published'

    class FakeOpen:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self):
            return '<html><body>dashboard</body></html>'

    monkeypatch.setattr(builtins, 'open', lambda *_args, **_kwargs: FakeOpen())
    dashboard_resp = await client.get('/dashboard')
    assert dashboard_resp.status_code == 200
    assert 'dashboard' in dashboard_resp.text


@pytest.mark.asyncio
async def test_publish_intent_does_not_block_on_orion_error(monkeypatch, client):
    import robin.alert_engine as ae

    class FailingClient:
        def __init__(self, *_, **__):
            pass

        def patch_process_intent(self, process_id, intent, data):
            raise RuntimeError('orion offline')

    monkeypatch.setattr(ae, 'RobinFiwareClient', FailingClient)

    resp = await client.post('/intent', json={'intent': 'STOP'})
    assert resp.status_code == 200
    assert resp.json()['status'] == 'published'


def test_alert_engine_feature_helpers_cover_profile_fallbacks(monkeypatch):
    import robin.alert_engine as ae

    profile = types.SimpleNamespace(
        ai_input_features=[
            {
                'key': 'wire_feed_speed_mpm_model_input',
                'aliases': 'not-a-list',
                'unit': 123,
                'default': 10.0,
                'step': 0.1,
            },
            {
                'key': 'custom_feature',
                'label': '',
                'aliases': ['legacy_custom', 'custom_feature', ''],
            },
        ],
        inverse_bounds='not-a-dict',
        inverse_optimizer={},
        forward_confidence={},
        model_path=None,
        default_tolerance=10.0,
        default_mode='parameter_driven',
    )
    monkeypatch.setattr(ae, 'active_profile', profile)

    engine = ae.AlertEngine()
    engine.feature_order = (
        'wire_feed_speed_mpm_model_input',
        'travel_speed_mps_model_input',
        'arc_length_correction_mm_model_input',
        'custom_feature',
    )

    specs = engine.input_feature_specs()
    by_key = {spec['key']: spec for spec in specs}

    assert by_key['wire_feed_speed_mpm_model_input']['unit'] == ''
    assert by_key['wire_feed_speed_mpm_model_input']['aliases'] == [
        'wire_feed_speed_mpm_model_input'
    ]
    assert by_key['travel_speed_mps_model_input']['label'] == 'Travel Speed'
    assert by_key['arc_length_correction_mm_model_input']['label'] == (
        'Arc Length Correction'
    )
    assert by_key['custom_feature']['label'] == 'custom_feature'
    assert by_key['custom_feature']['aliases'] == [
        'custom_feature',
        'legacy_custom',
    ]

    assert engine._canonicalize_input_params(None) == {}
    canonical = engine._canonicalize_input_params(
        {
            'wire_feed_speed_mpm_model_input': True,
            'legacy_custom': 4,
            'travel_speed_mps_model_input': 0.02,
        }
    )
    assert canonical['custom_feature'] == 4.0
    assert 'wire_feed_speed_mpm_model_input' not in canonical
    assert engine._default_feature_value('missing_feature') == 0.0

    assert ae.AlertEngine._coerce_bound({'min': 5, 'max': 1}, (0, 1)) == (1.0, 5.0)
    assert ae.AlertEngine._coerce_bound([3, 3], (0, 1)) == (3.0, 4.0)
    assert engine._resolve_inverse_bounds()['custom_feature'] == (0.0, 1.0)
