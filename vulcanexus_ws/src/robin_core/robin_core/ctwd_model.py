"""CTWD model utilities.

Simple linear mapping from Fronius recommended voltage to target CTWD.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class CtwdModelParams:
    """Linear CTWD model: voltage_min_v → ctwd_min_mm, voltage_max_v → ctwd_max_mm."""

    voltage_min_v: float = 14.0
    voltage_max_v: float = 38.0
    ctwd_min_mm: float = 8.0
    ctwd_max_mm: float = 25.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def optimal_ctwd_mm_from_voltage(
    voltage_v: float,
    params: CtwdModelParams | None = None,
) -> float:
    """Estimate optimal CTWD (mm) from recommended voltage using a simple linear slope."""
    p = params or CtwdModelParams()
    v = _clamp(float(voltage_v), p.voltage_min_v, p.voltage_max_v)
    ratio = (v - p.voltage_min_v) / (p.voltage_max_v - p.voltage_min_v)
    return p.ctwd_min_mm + ratio * (p.ctwd_max_mm - p.ctwd_min_mm)


def compute_target_ctwd_m(
    voltage_recomm_v: float,
    params: CtwdModelParams | None = None,
) -> float:
    """Return target CTWD in metres from recommended voltage."""
    return optimal_ctwd_mm_from_voltage(voltage_recomm_v, params) / 1000.0
