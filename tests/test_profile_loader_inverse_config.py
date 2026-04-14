from robin.profile_loader import Profile


def test_profile_inverse_defaults_are_available():
    profile = Profile({})

    assert profile.inverse_bounds['wire_feed_speed_mpm_model_input'] == [1.0, 30.0]
    assert profile.inverse_bounds['travel_speed_mps_model_input'] == [0.001, 1.0]
    assert profile.inverse_optimizer['restarts'] == 24
    assert profile.inverse_optimizer['max_iterations'] == 80
    assert profile.ai_input_features[0]['key'] == 'wire_feed_speed_mpm_model_input'


def test_profile_inverse_config_can_be_overridden():
    profile = Profile(
        {
            'ai': {
                'inverse_bounds': {
                    'wire_feed_speed_mpm_model_input': [5.0, 15.0],
                },
                'inverse_optimizer': {
                    'restarts': 12,
                    'regularization': 0.01,
                },
            }
        }
    )

    assert profile.inverse_bounds['wire_feed_speed_mpm_model_input'] == [5.0, 15.0]
    assert profile.inverse_optimizer['restarts'] == 12
    assert profile.inverse_optimizer['regularization'] == 0.01


def test_profile_ai_input_features_can_be_overridden():
    profile = Profile(
        {
            'ai': {
                'input_features': [
                    {
                        'key': 'travel_speed_mps_model_input',
                        'label': 'Travel Speed',
                        'unit': 'm/s',
                        'default': 0.02,
                    }
                ]
            }
        }
    )

    assert profile.ai_input_features == [
        {
            'key': 'travel_speed_mps_model_input',
            'label': 'Travel Speed',
            'unit': 'm/s',
            'default': 0.02,
        }
    ]
