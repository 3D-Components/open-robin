# Mock process geometry simulation: width & height from wire speed, current, voltage
# Units: wire_speed in mm/s (unless otherwise stated), current in A, voltage in V
# The model uses simplified physical relations + stochasticity:
# - Volumetric deposition V̇ ≈ η_dep * A_wire * v_wire
# - Profile cross-section A_profile ≈ V̇ / v_travel
# - Heat input per length HI ≈ η_arc * V * I / v_travel
# - Profile width W ≈ k_w * HI^a
# - Profile height H from semi-ellipse area A ≈ (π/4) * W * H ⇒ H = 4A / (π W)
#
# Travel speed v_travel is never exposed as an input; it is inferred from current and wire speed:
#   v_travel ≈ v0 + k_I * I + k_WFS * WFS + ε
# where WFS is wire feed speed in m/min. Defaults below are calibrated to give realistic mm-scale
# widths & heights for a representative arc-based process with 1.2 mm wire around 150–250 A and 18–26 V.
#
# Output: a DataFrame with simulated profile width/height and process internals; a CSV is saved.
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


@dataclass
class ProcessParams:
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
    vt_sd_mm_s: float = 1.0  # Random per-profile travel speed jitter (σ)
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


def simulate_profiles(
    wire_feed_rate_m_per_min: float,
    current_A: float,
    voltage_V: float,
    n_profiles: int = 50,
    params: Optional[ProcessParams] = None,
) -> pd.DataFrame:
    """Simulate profile width/height for an arc-like process with stochasticity.

    Args:
        wire_feed_rate_m_per_min: Wire feed rate (m/min).
        current_A: Process current (A).
        voltage_V: Arc voltage (V).
        n_profiles: Number of consecutive profiles (or sampled locations) to simulate.
        params: Optional ProcessParams to tweak assumptions.
    Returns:
        DataFrame with columns:
         - profile_idx
         - travel_speed_mm_s
         - eta_dep, eta_arc
         - heat_input_J_per_mm
         - dep_rate_mm3_s
         - cross_section_area_mm2
         - width_mm, height_mm
    """
    p = params or ProcessParams()
    rng = np.random.default_rng(p.seed)
    # Wire geometry and kinematics
    r_mm = p.wire_diameter_mm / 2.0
    A_wire_mm2 = math.pi * r_mm * r_mm
    v_wire_mm_s = wire_feed_rate_m_per_min * 1000.0 / 60.0  # m/min -> mm/s

    rows = []
    drift = 0.0
    for i in range(n_profiles):
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
        A_profile_mm2 = dep_rate_mm3_s / vt

        # Heat input per unit length (J/mm): (η V I)/v_t with units consistent
        # Power (W = J/s) divided by travel speed (mm/s) -> J/mm
        heat_input_J_per_mm = (eta_arc * voltage_V * current_A) / vt

        # Width from power law, Height from area with semi-ellipse assumption
        width_mm = p.kw * (heat_input_J_per_mm**p.a_exp)

        # Safety: avoid division by very small width
        width_mm = max(width_mm, 0.5)

        height_mm = (4.0 * A_profile_mm2) / (math.pi * width_mm)

        rows.append(
            {
                'profile_idx': i + 1,
                'travel_speed_mm_s': vt,
                'eta_dep': eta_dep,
                'eta_arc': eta_arc,
                'heat_input_J_per_mm': heat_input_J_per_mm,
                'dep_rate_mm3_s': dep_rate_mm3_s,
                'cross_section_area_mm2': A_profile_mm2,
                'width_mm': width_mm,
                'height_mm': height_mm,
            }
        )

    df = pd.DataFrame(rows)
    return df


def _estimate_travel_speed(
    current_A: float,
    wire_speed_mm_s: float,
    params: ProcessParams,
) -> float:
    """Deterministic travel speed estimate based on current and wire feed speed."""
    wire_feed_rate_m_per_min = wire_speed_mm_s * 60.0 / 1000.0
    vt = (
        params.v0_mm_s
        + params.k_I_mm_s_per_A * current_A
        + params.k_WFS_mm_s_per_mpm * wire_feed_rate_m_per_min
    )
    return max(vt, params.vt_min_mm_s)


def generate_input_output_dataset(
    n_samples: int = 1000,
    wire_speed_range: tuple = (60.0, 180.0),  # mm/s (~3.6 to 10.8 m/min)
    current_range: tuple = (150.0, 250.0),  # A
    voltage_range: tuple = (18.0, 26.0),  # V
    wire_diameter_mm: float = 1.2,
    seed: Optional[int] = 42,
) -> pd.DataFrame:
    """Generate a dataset with wire speed, current, voltage as inputs and profile geometry as outputs.

    Travel speed is derived internally from the sampled wire speed and current using the
    deterministic relation in :func:`_estimate_travel_speed`, keeping travel speed out of the
    public interface entirely.

    Args:
        n_samples: Number of samples to generate.
        wire_speed_range: (min, max) wire speed in mm/s.
        current_range: (min, max) process current in A.
        voltage_range: (min, max) arc voltage in V.
        wire_diameter_mm: Wire diameter in mm.
        seed: Random seed for reproducible sampling.

    Returns:
        DataFrame with columns: wire_speed_mm_s, current_A, voltage_V, width_mm, height_mm
    """
    rng = np.random.default_rng(seed)

    # Fixed parameters for deterministic calculation
    eta_dep = 0.85  # Deposition efficiency
    eta_arc = 0.80  # Arc efficiency
    kw = 1.20  # Width coefficient
    a_exp = 0.35  # Width power exponent

    # Wire geometry
    r_mm = wire_diameter_mm / 2.0
    A_wire_mm2 = math.pi * r_mm * r_mm

    params = ProcessParams()

    # Generate random input samples
    wire_speeds = rng.uniform(
        wire_speed_range[0], wire_speed_range[1], n_samples
    )
    currents = rng.uniform(current_range[0], current_range[1], n_samples)
    voltages = rng.uniform(voltage_range[0], voltage_range[1], n_samples)

    rows = []
    for i in range(n_samples):
        wire_speed = wire_speeds[i]
        I = currents[i]   # NoQA: ignore variable name conflict
        V = voltages[i]   # NoQA: ignore variable name conflict

        vt = _estimate_travel_speed(I, wire_speed, params)
        v_wire_mm_s = wire_speed

        # Deposition volumetric rate (mm^3/s)
        dep_rate_mm3_s = eta_dep * A_wire_mm2 * v_wire_mm_s

        # Cross-sectional area from continuity (mm^2)
        A_profile_mm2 = dep_rate_mm3_s / vt

        # Heat input per unit length (J/mm)
        heat_input_J_per_mm = (eta_arc * V * I) / vt

        # Width from power law
        width_mm = kw * (heat_input_J_per_mm**a_exp)
        width_mm = max(width_mm, 0.5)  # Safety floor

        # Height from semi-ellipse area assumption
        height_mm = (4.0 * A_profile_mm2) / (math.pi * width_mm)

        rows.append(
            {
                'wire_speed_mm_s': wire_speed,
                'current_A': I,
                'voltage_V': V,
                'width_mm': width_mm,
                'height_mm': height_mm,
            }
        )

    return pd.DataFrame(rows)


# --- Generate input-output dataset ---
n_samples = 1000
dataset = generate_input_output_dataset(n_samples=n_samples)

root_dir = Path(__file__).resolve().parents[1]
data_dir = root_dir / 'data'
data_dir.mkdir(parents=True, exist_ok=True)

# Save dataset
csv_path = data_dir / 'process_dataset_input_output.csv'
dataset.to_csv(csv_path, index=False)

# Quick summary
print('Generated Input-Output Dataset for Process Geometry Prediction')
print(f'Number of samples: {n_samples}')
print('\nInput ranges:')
print('  Wire speed: 60.0 - 180.0 mm/s')
print('  Current: 150 - 250 A')
print('  Voltage: 18 - 26 V')
print(
    '\nDataset columns: wire_speed_mm_s, current_A, voltage_V, width_mm, height_mm'
)
print('\nSummary statistics:')
print(dataset.describe().round(3))

# Visualization
fig, axes = plt.subplots(2, 3, figsize=(15, 10))

# Width vs inputs
# Width vs inputs
axes[0, 0].scatter(
    dataset['wire_speed_mm_s'], dataset['width_mm'], alpha=0.5, s=10
)
axes[0, 0].set_xlabel('Wire Speed (mm/s)')
axes[0, 0].set_ylabel('Width (mm)')
axes[0, 0].set_title('Width vs Travel Speed')

axes[0, 1].scatter(dataset['current_A'], dataset['width_mm'], alpha=0.5, s=10)
axes[0, 1].set_xlabel('Current (A)')
axes[0, 1].set_ylabel('Width (mm)')
axes[0, 1].set_title('Width vs Current')

axes[0, 2].scatter(dataset['voltage_V'], dataset['width_mm'], alpha=0.5, s=10)
axes[0, 2].set_xlabel('Voltage (V)')
axes[0, 2].set_ylabel('Width (mm)')
axes[0, 2].set_title('Width vs Voltage')

# Height vs inputs
axes[1, 0].scatter(
    dataset['wire_speed_mm_s'], dataset['height_mm'], alpha=0.5, s=10
)
axes[1, 0].set_xlabel('Wire Speed (mm/s)')
axes[1, 0].set_ylabel('Height (mm)')
axes[1, 0].set_title('Height vs Wire Speed')

axes[1, 1].scatter(dataset['current_A'], dataset['height_mm'], alpha=0.5, s=10)
axes[1, 1].set_xlabel('Current (A)')
axes[1, 1].set_ylabel('Height (mm)')
axes[1, 1].set_title('Height vs Current')

axes[1, 2].scatter(dataset['voltage_V'], dataset['height_mm'], alpha=0.5, s=10)
axes[1, 2].set_xlabel('Voltage (V)')
axes[1, 2].set_ylabel('Height (mm)')
axes[1, 2].set_title('Height vs Voltage')

plt.tight_layout()
viz_path = data_dir / 'process_dataset_visualization.png'
plt.savefig(viz_path, dpi=150)
print(f'\n[Saved visualization] {viz_path.relative_to(root_dir)}')
plt.show()

print(f'\n[Saved CSV] {csv_path.relative_to(root_dir)}')
