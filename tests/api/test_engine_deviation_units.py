def test_check_parameter_driven_deviation_no_alert(monkeypatch):
    import robin.alert_engine as ae

    engine = ae.AlertEngine()

    # Force a deterministic prediction
    monkeypatch.setattr(
        engine,
        'predict_geometry_from_params',
        lambda params: {'height': 5.0, 'width': 8.0},
    )

    input_params = {'wireSpeed': 10, 'current': 150, 'voltage': 24}
    measured = {'height': 5.1, 'width': 7.9}
    alert, expected = engine.check_parameter_driven_deviation(
        'P1', input_params, measured, tolerance=5.0
    )

    assert alert is None
    assert expected == {'height': 5.0, 'width': 8.0}


def test_check_parameter_driven_deviation_alert(monkeypatch):
    import robin.alert_engine as ae

    engine = ae.AlertEngine()
    monkeypatch.setattr(
        engine,
        'predict_geometry_from_params',
        lambda params: {'height': 5.0, 'width': 8.0},
    )

    input_params = {'wireSpeed': 10, 'current': 150, 'voltage': 24}
    measured = {'height': 6.0, 'width': 8.5}
    alert, expected = engine.check_parameter_driven_deviation(
        'P2', input_params, measured, tolerance=10.0
    )

    assert expected == {'height': 5.0, 'width': 8.0}
    assert alert is not None
    assert alert.process_id == 'P2'
    assert alert.deviation_type == 'geometry'
    # Height deviation ~20%
    assert alert.deviation_percentage >= 19.9
    assert (
        isinstance(alert.recommended_actions, list)
        and len(alert.recommended_actions) > 0
    )


def test_check_geometry_driven_deviation_no_alert():
    import robin.alert_engine as ae

    engine = ae.AlertEngine()
    target = {'height': 5.0, 'width': 8.0}
    measured = {'height': 5.2, 'width': 7.7}
    alert, expected = engine.check_geometry_driven_deviation(
        'PG', target, measured, tolerance=5.0
    )

    assert expected == target
    assert alert is None


def test_check_geometry_driven_deviation_alert():
    import robin.alert_engine as ae

    engine = ae.AlertEngine()
    target = {'height': 5.0, 'width': 8.0}
    measured = {'height': 6.2, 'width': 7.9}
    alert, expected = engine.check_geometry_driven_deviation(
        'PG2', target, measured, tolerance=10.0
    )

    assert expected == target
    assert alert is not None
    assert alert.process_id == 'PG2'
    assert alert.deviation_type == 'geometry'
    # Height deviation from 5.0 to 6.2 = 24%
    assert alert.deviation_percentage >= 23.9
