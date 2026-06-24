"""Fronius synergy line model.

Maps wire feed speed (WFS, m/min) and arc length correction (mm)
to welding current (A) and voltage (V) for synergetic MIG/MAG mode.

Data from measured Fronius TPS/i synergy curves.
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class SynergyRegion:
    """One piecewise-linear segment of the synergy characteristic.

    V  = v_slope * I + v_intercept
    WFS = w_slope * I + w_intercept

    wfs_range defines the valid WFS span for this region.
    """
    wfs_min: float
    wfs_max: float
    v_slope: float       # dV/dI
    v_intercept: float   # V at I=0
    w_slope: float       # dWFS/dI
    w_intercept: float   # WFS at I=0

    def current_from_wfs(self, wfs: float) -> float:
        return (wfs - self.w_intercept) / self.w_slope

    def voltage_from_current(self, current: float) -> float:
        return self.v_slope * current + self.v_intercept

    def evaluate(self, wfs: float) -> tuple[float, float]:
        """Return (current, voltage) for a given WFS."""
        i = self.current_from_wfs(wfs)
        v = self.voltage_from_current(i)
        return i, v


# -- Measured synergy regions (ordered by WFS) -----------------------------

SYNERGY_REGIONS = [
    SynergyRegion(
        wfs_min=2.0, wfs_max=3.4,
        v_slope=0.0139461, v_intercept=13.87765,
        w_slope=0.0399366, w_intercept=0.167829,
    ),
    SynergyRegion(
        wfs_min=4.3, wfs_max=7.7,
        v_slope=0.0248039, v_intercept=13.4774,
        w_slope=0.0545947, w_intercept=-1.21337,
    ),
    SynergyRegion(
        wfs_min=8.7, wfs_max=16.7,
        v_slope=0.0486014, v_intercept=15.03566,
        w_slope=0.0703846, w_intercept=-4.60064,
    ),
]

# Operating limits — extrapolate beyond measured data up to these bounds
WFS_MIN = 2.0
WFS_MAX = 19.0

ARC_LENGTH_MIN = -10.0
ARC_LENGTH_MAX = 10.0

# Voltage offset per unit of arc length correction (V/mm)
ARC_LENGTH_VOLTAGE_GAIN = 0.92


def synergy_lookup(wfs: float, arc_length_correction: float = 0.0,
                   ) -> tuple[float, float, float, float]:
    """Map commanded WFS + arc correction → (effective WFS, current [A], voltage [V], effective ALC).
    ...
    """
    regions = SYNERGY_REGIONS

    # Clamp to operating range
    wfs = max(WFS_MIN, min(WFS_MAX, wfs))
    arc_length_correction = max(ARC_LENGTH_MIN, min(ARC_LENGTH_MAX, arc_length_correction))

    if wfs <= regions[0].wfs_min:
        i, v = regions[0].evaluate(wfs)
        return wfs, i, v + arc_length_correction * ARC_LENGTH_VOLTAGE_GAIN, arc_length_correction

    if wfs >= regions[-1].wfs_max:
        i, v = regions[-1].evaluate(wfs)
        return wfs, i, v + arc_length_correction * ARC_LENGTH_VOLTAGE_GAIN, arc_length_correction

    for idx, region in enumerate(regions):
        if region.wfs_min <= wfs <= region.wfs_max:
            i, v = region.evaluate(wfs)
            return wfs, i, v + arc_length_correction * ARC_LENGTH_VOLTAGE_GAIN, arc_length_correction

        if idx < len(regions) - 1:
            next_region = regions[idx + 1]
            if region.wfs_max < wfs < next_region.wfs_min:
                i_lo, v_lo = region.evaluate(region.wfs_max)
                i_hi, v_hi = next_region.evaluate(next_region.wfs_min)
                t = ((wfs - region.wfs_max)
                     / (next_region.wfs_min - region.wfs_max))
                i = i_lo + t * (i_hi - i_lo)
                v = v_lo + t * (v_hi - v_lo)
                return wfs, i, v + arc_length_correction * ARC_LENGTH_VOLTAGE_GAIN, arc_length_correction

    return wfs, 0.0, 0.0, arc_length_correction