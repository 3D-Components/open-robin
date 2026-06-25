#!/usr/bin/env python3

from robin_core.ctwd_model import (
    CtwdModelParams,
    compute_target_ctwd_m,
    optimal_ctwd_mm_from_voltage,
)


def test_endpoints():
    """At boundary voltages, CTWD equals boundary values."""
    assert optimal_ctwd_mm_from_voltage(14.0) == 8.0
    assert optimal_ctwd_mm_from_voltage(38.0) == 25.0


def test_midpoint():
    """At the midpoint voltage, CTWD is the midpoint."""
    mid_v = (14.0 + 38.0) / 2.0  # 26V
    mid_ctwd = (8.0 + 25.0) / 2.0  # 16.5mm
    assert abs(optimal_ctwd_mm_from_voltage(mid_v) - mid_ctwd) < 1e-9


def test_clamp_low():
    """Voltages below 14V clamp to 8mm."""
    assert optimal_ctwd_mm_from_voltage(0.0) == 8.0
    assert optimal_ctwd_mm_from_voltage(10.0) == 8.0


def test_clamp_high():
    """Voltages above 38V clamp to 25mm."""
    assert optimal_ctwd_mm_from_voltage(50.0) == 25.0
    assert optimal_ctwd_mm_from_voltage(999.0) == 25.0


def test_monotone():
    """CTWD increases monotonically with voltage."""
    voltages = [14.0, 18.0, 22.0, 26.0, 30.0, 34.0, 38.0]
    values = [optimal_ctwd_mm_from_voltage(v) for v in voltages]
    assert all(values[i + 1] > values[i] for i in range(len(values) - 1))


def test_compute_target_ctwd_m():
    """compute_target_ctwd_m returns metres."""
    ctwd_m = compute_target_ctwd_m(26.0)
    assert abs(ctwd_m - 0.0165) < 1e-9


def test_custom_params():
    """Custom params override defaults."""
    p = CtwdModelParams(voltage_min_v=10.0, voltage_max_v=20.0,
                        ctwd_min_mm=5.0, ctwd_max_mm=15.0)
    assert optimal_ctwd_mm_from_voltage(10.0, p) == 5.0
    assert optimal_ctwd_mm_from_voltage(20.0, p) == 15.0
    assert abs(optimal_ctwd_mm_from_voltage(15.0, p) - 10.0) < 1e-9
