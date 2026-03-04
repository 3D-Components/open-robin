from robin.ai.inverse import (
    GeometryInverseOptimizer,
    InverseOptimizationConfig,
)


def _forward_identity(params):
    # Simple deterministic mapping used for optimizer unit tests.
    return {
        'height': float(params['wireSpeed']),
        'width': float(params['current']),
    }


def test_inverse_optimizer_hits_target_for_identity_mapping():
    optimizer = GeometryInverseOptimizer(
        feature_order=('wireSpeed', 'current', 'voltage'),
        parameter_bounds={
            'wireSpeed': (5.0, 15.0),
            'current': (100.0, 200.0),
            'voltage': (18.0, 32.0),
        },
        predict_geometry=_forward_identity,
        config=InverseOptimizationConfig(
            restarts=12,
            max_iterations=80,
            regularization=0.0,
            random_seed=123,
        ),
    )
    target = {'height': 11.2, 'width': 163.0}
    result = optimizer.solve(target)

    assert abs(result.params['wireSpeed'] - target['height']) < 0.2
    assert abs(result.params['current'] - target['width']) < 0.5
    assert abs(result.predicted_geometry['height'] - target['height']) < 0.2
    assert abs(result.predicted_geometry['width'] - target['width']) < 0.5
    assert 0.9 <= result.confidence <= 0.99


def test_inverse_optimizer_respects_bounds_when_target_is_unreachable():
    optimizer = GeometryInverseOptimizer(
        feature_order=('wireSpeed', 'current', 'voltage'),
        parameter_bounds={
            'wireSpeed': (5.0, 15.0),
            'current': (100.0, 200.0),
            'voltage': (18.0, 32.0),
        },
        predict_geometry=_forward_identity,
        config=InverseOptimizationConfig(
            restarts=8,
            max_iterations=50,
            regularization=0.0,
            random_seed=7,
        ),
    )
    target = {'height': 40.0, 'width': 800.0}
    result = optimizer.solve(target)

    assert 5.0 <= result.params['wireSpeed'] <= 15.0
    assert 100.0 <= result.params['current'] <= 200.0
    assert result.params['wireSpeed'] >= 14.9
    assert result.params['current'] >= 199.0
    assert 0.0 <= result.confidence < 0.3


def test_inverse_optimizer_regularization_keeps_unobserved_feature_stable():
    optimizer = GeometryInverseOptimizer(
        feature_order=('wireSpeed', 'current', 'voltage'),
        parameter_bounds={
            'wireSpeed': (5.0, 15.0),
            'current': (100.0, 200.0),
            'voltage': (18.0, 32.0),
        },
        predict_geometry=_forward_identity,
        config=InverseOptimizationConfig(
            restarts=10,
            max_iterations=60,
            regularization=1.0,
            random_seed=42,
        ),
    )
    current = {'wireSpeed': 10.5, 'current': 155.0, 'voltage': 29.5}
    target = {'height': 10.5, 'width': 155.0}
    result = optimizer.solve(target, current_params=current)

    # Voltage does not affect the forward mapping in this test model.
    # With regularization enabled, optimizer should keep it close to current.
    assert abs(result.params['voltage'] - current['voltage']) < 0.5
