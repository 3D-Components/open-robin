from robin.profile_loader import Profile


def test_profile_inverse_defaults_are_available():
    profile = Profile({})

    assert profile.inverse_bounds['wireSpeed'] == [1.0, 300.0]
    assert profile.inverse_bounds['current'] == [1.0, 400.0]
    assert profile.inverse_optimizer['restarts'] == 24
    assert profile.inverse_optimizer['max_iterations'] == 80


def test_profile_inverse_config_can_be_overridden():
    profile = Profile(
        {
            'ai': {
                'inverse_bounds': {
                    'wireSpeed': [5.0, 15.0],
                },
                'inverse_optimizer': {
                    'restarts': 12,
                    'regularization': 0.01,
                },
            }
        }
    )

    assert profile.inverse_bounds['wireSpeed'] == [5.0, 15.0]
    assert profile.inverse_optimizer['restarts'] == 12
    assert profile.inverse_optimizer['regularization'] == 0.01
