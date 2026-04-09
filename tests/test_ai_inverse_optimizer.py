from robin.ai.inverse import (
    GeometryInverseOptimizer,
    InverseOptimizationConfig,
)


def _forward_identity(params):
    # Simple deterministic mapping used for optimizer unit tests.
    return {
        'height': float(params['wire_feed_speed_mpm_model_input']),
        'width': float(params['travel_speed_mps_model_input']),
    }


def test_inverse_optimizer_hits_target_for_identity_mapping():
    optimizer = GeometryInverseOptimizer(
        feature_order=(
            'wire_feed_speed_mpm_model_input',
            'travel_speed_mps_model_input',
            'arc_length_correction_mm_model_input',
        ),
        parameter_bounds={
            'wire_feed_speed_mpm_model_input': (5.0, 15.0),
            'travel_speed_mps_model_input': (0.01, 0.04),
            'arc_length_correction_mm_model_input': (-6.0, 6.0),
        },
        predict_geometry=_forward_identity,
        config=InverseOptimizationConfig(
            restarts=12,
            max_iterations=80,
            regularization=0.0,
            random_seed=123,
        ),
    )
    target = {'height': 11.2, 'width': 0.023}
    result = optimizer.solve(target)

    assert (
        abs(
            result.params['wire_feed_speed_mpm_model_input']
            - target['height']
        )
        < 0.2
    )
    assert (
        abs(
            result.params['travel_speed_mps_model_input']
            - target['width']
        )
        < 0.002
    )
    assert abs(result.predicted_geometry['height'] - target['height']) < 0.2
    assert abs(result.predicted_geometry['width'] - target['width']) < 0.002
    assert 0.9 <= result.confidence <= 0.99


def test_inverse_optimizer_respects_bounds_when_target_is_unreachable():
    optimizer = GeometryInverseOptimizer(
        feature_order=(
            'wire_feed_speed_mpm_model_input',
            'travel_speed_mps_model_input',
            'arc_length_correction_mm_model_input',
        ),
        parameter_bounds={
            'wire_feed_speed_mpm_model_input': (5.0, 15.0),
            'travel_speed_mps_model_input': (0.01, 0.04),
            'arc_length_correction_mm_model_input': (-6.0, 6.0),
        },
        predict_geometry=_forward_identity,
        config=InverseOptimizationConfig(
            restarts=8,
            max_iterations=50,
            regularization=0.0,
            random_seed=7,
        ),
    )
    target = {'height': 40.0, 'width': 0.8}
    result = optimizer.solve(target)

    assert 5.0 <= result.params['wire_feed_speed_mpm_model_input'] <= 15.0
    assert 0.01 <= result.params['travel_speed_mps_model_input'] <= 0.04
    assert result.params['wire_feed_speed_mpm_model_input'] >= 14.9
    assert result.params['travel_speed_mps_model_input'] >= 0.039
    assert 0.0 <= result.confidence < 0.3


def test_inverse_optimizer_regularization_keeps_unobserved_feature_stable():
    optimizer = GeometryInverseOptimizer(
        feature_order=(
            'wire_feed_speed_mpm_model_input',
            'travel_speed_mps_model_input',
            'arc_length_correction_mm_model_input',
        ),
        parameter_bounds={
            'wire_feed_speed_mpm_model_input': (5.0, 15.0),
            'travel_speed_mps_model_input': (0.01, 0.04),
            'arc_length_correction_mm_model_input': (-6.0, 6.0),
        },
        predict_geometry=_forward_identity,
        config=InverseOptimizationConfig(
            restarts=10,
            max_iterations=60,
            regularization=1.0,
            random_seed=42,
        ),
    )
    current = {
        'wire_feed_speed_mpm_model_input': 10.5,
        'travel_speed_mps_model_input': 0.025,
        'arc_length_correction_mm_model_input': 2.5,
    }
    target = {'height': 10.5, 'width': 0.025}
    result = optimizer.solve(target, current_params=current)

    # Arc length correction does not affect the forward mapping in this test model.
    # With regularization enabled, optimizer should keep it close to current.
    assert (
        abs(
            result.params['arc_length_correction_mm_model_input']
            - current['arc_length_correction_mm_model_input']
        )
        < 0.5
    )
