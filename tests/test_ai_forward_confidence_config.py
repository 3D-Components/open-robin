from robin.profile_loader import Profile


def test_profile_forward_confidence_defaults_are_available():
    profile = Profile({})

    assert profile.forward_confidence['mc_samples'] == 20
    assert profile.forward_confidence['uncertainty_weight'] == 0.65
    assert profile.forward_confidence['distance_weight'] == 0.35


def test_profile_forward_confidence_can_be_overridden():
    profile = Profile(
        {
            'ai': {
                'forward_confidence': {
                    'mc_samples': 40,
                    'distance_scale': 3.0,
                    'max_confidence': 0.95,
                }
            }
        }
    )

    assert profile.forward_confidence['mc_samples'] == 40
    assert profile.forward_confidence['distance_scale'] == 3.0
    assert profile.forward_confidence['max_confidence'] == 0.95
