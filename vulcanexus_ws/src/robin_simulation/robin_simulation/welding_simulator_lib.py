# Mock welding bead simulation: width & height from wire speed, current, voltage
# Units: wire_speed in mm/s (unless otherwise stated), current in A, voltage in V
# The model uses simplified physical relations + stochasticity:
# - Volumetric deposition V̇ ≈ η_dep * A_wire * v_wire
# - Bead cross-section A_bead ≈ V̇ / v_travel
# - Heat input per length HI ≈ η_arc * V * I / v_travel
# - Bead width W ≈ k_w * HI^a
# - Bead height H from semi-ellipse area A ≈ (π/4) * W * H ⇒ H = 4A / (π W)
#
# Travel speed v_travel is never exposed as an input; it is inferred from current and wire speed:
#   v_travel ≈ v0 + k_I * I + k_WFS * WFS + ε
# where WFS is wire feed speed in m/min. Defaults below are calibrated to give realistic mm-scale
# widths & heights for GMAW on steel with 1.2 mm wire around 150–250 A and 18–26 V.
#
# Output: a DataFrame with simulated bead width/height and process internals; a CSV is saved.
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


@dataclass
class WeldParams:
    wire_diameter_mm: float = 1.2        # Solid wire, typical 0.9–1.2 mm
    rho_wire_g_per_mm3: float = (
        7.85e-3  # Steel density ~7.85 g/cm3 = 7.85e-3 g/mm3
    )
    eta_dep_mean: float = 0.85           # Deposition efficiency
    eta_dep_sd: float = 0.05
    eta_arc_mean: float = 0.80           # Arc efficiency
    eta_arc_sd: float = 0.03
    v0_mm_s: float = 5.0  # Baseline travel speed at zero amps/feed
    k_I_mm_s_per_A: float = 0.02         # Travel speed sensitivity to current
    k_WFS_mm_s_per_mpm: float = 0.50  # Travel speed sensitivity to WFS (m/min)
    vt_sd_mm_s: float = 1.0  # Random per-bead travel speed jitter (σ)
    drift_sd_mm_s: float = (
        0.10  # Slow drift on travel speed (random walk step)
    )
    kw: float = 1.20                      # Width coefficient (scales HI)
    a_exp: float = 0.35                  # Width power exponent
    vt_min_mm_s: float = 2.0             # Safety floor on travel speed
    eta_bounds: tuple = (0.60, 0.95)     # Bounds for efficiencies
    seed: Optional[int] = 42             # RNG seed for reproducibility


def _truncnorm(mean, sd, low, high, size=None, rng=None):
    if rng is None:
        rng = np.random.default_rng()
    vals = rng.normal(mean, sd, size=size)
    if np.isscalar(vals):
        return float(np.clip(vals, low, high))
    return np.clip(vals, low, high)


def simulate_beads(
    wire_feed_rate_m_per_min: float,
    current_A: float,
    voltage_V: float,
    n_beads: int = 50,
    params: Optional[WeldParams] = None,
) -> pd.DataFrame:
    """Simulate bead width/height for a GMAW-like process with stochasticity.

    Args:
        wire_feed_rate_m_per_min: Wire feed rate (m/min).
        current_A: Welding current (A).
        voltage_V: Arc voltage (V).
        n_beads: Number of consecutive beads (or sampled locations) to simulate.
        params: Optional WeldParams to tweak assumptions.
    Returns:
        DataFrame with columns:
         - bead_idx
         - travel_speed_mm_s
         - eta_dep, eta_arc
         - heat_input_J_per_mm
         - dep_rate_mm3_s
         - cross_section_area_mm2
         - width_mm, height_mm
    """
    p = params or WeldParams()
    rng = np.random.default_rng(p.seed)
    # Wire geometry and kinematics
    r_mm = p.wire_diameter_mm / 2.0
    A_wire_mm2 = math.pi * r_mm * r_mm
    v_wire_mm_s = wire_feed_rate_m_per_min * 1000.0 / 60.0  # m/min -> mm/s

    rows = []
    drift = 0.0
    for i in range(n_beads):
        # Efficiencies with truncation
        eta_dep = _truncnorm(
            p.eta_dep_mean,
            p.eta_dep_sd,
            p.eta_bounds[0],
            p.eta_bounds[1],
            rng=rng,
        )
        eta_arc = _truncnorm(
            p.eta_arc_mean,
            p.eta_arc_sd,
            p.eta_bounds[0],
            p.eta_bounds[1],
            rng=rng,
        )

        # Travel speed model with jitter + slow drift
        drift += rng.normal(0.0, p.drift_sd_mm_s)
        vt = (
            p.v0_mm_s
            + p.k_I_mm_s_per_A * current_A
            + p.k_WFS_mm_s_per_mpm * wire_feed_rate_m_per_min
            + rng.normal(0.0, p.vt_sd_mm_s)
            + drift
        )
        vt = max(vt, p.vt_min_mm_s)

        # Deposition volumetric rate (mm^3/s)
        dep_rate_mm3_s = eta_dep * A_wire_mm2 * v_wire_mm_s

        # Cross-sectional area from continuity (mm^2)
        A_bead_mm2 = dep_rate_mm3_s / vt

        # Heat input per unit length (J/mm): (η V I)/v_t with units consistent
        # Power (W = J/s) divided by travel speed (mm/s) -> J/mm
        heat_input_J_per_mm = (eta_arc * voltage_V * current_A) / vt

        # Width from power law, Height from area with semi-ellipse assumption
        width_mm = p.kw * (heat_input_J_per_mm**p.a_exp)

        # Safety: avoid division by very small width
        width_mm = max(width_mm, 0.5)

        height_mm = (4.0 * A_bead_mm2) / (math.pi * width_mm)

        rows.append(
            {
                'bead_idx': i + 1,
                'travel_speed_mm_s': vt,
                'eta_dep': eta_dep,
                'eta_arc': eta_arc,
                'heat_input_J_per_mm': heat_input_J_per_mm,
                'dep_rate_mm3_s': dep_rate_mm3_s,
                'cross_section_area_mm2': A_bead_mm2,
                'width_mm': width_mm,
                'height_mm': height_mm,
            }
        )

    df = pd.DataFrame(rows)
    return df


def _estimate_travel_speed(
    current_A: float,
    wire_speed_mm_s: float,
    params: WeldParams,
) -> float:
    """Deterministic travel speed estimate based on current and wire feed speed."""
    wire_feed_rate_m_per_min = wire_speed_mm_s * 60.0 / 1000.0
    vt = (
        params.v0_mm_s
        + params.k_I_mm_s_per_A * current_A
        + params.k_WFS_mm_s_per_mpm * wire_feed_rate_m_per_min
    )
    return max(vt, params.vt_min_mm_s)


