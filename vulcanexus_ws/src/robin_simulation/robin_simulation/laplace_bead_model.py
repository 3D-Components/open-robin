"""Drop-in bead profile model for weld bead cross-sections.

This version keeps the same public API as the previous Laplace-based
module, but replaces the capillary sessile-drop solve with a simpler
reduced-order bead geometry model:

1. Predict width and height from current, WFS, and travel speed using a
   compact monotonic surrogate tuned as a practical starter model for
   1.0 mm ER70S-6 with Ar + CO2 (8%).
2. Derive toe/contact angle from a parabolic cross-section:
       z(x) = H * (1 - (2x/W)^2)
   which gives:
       theta = atan(4H / W)

This is intended as a practical simulation model, not a physics-based
melt-pool model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Nominal process point for 1.0 mm ER70S-6, Ar + CO2 (8%)
# These constants are meant as a practical starter point.
NOMINAL_CURRENT_A: float = 180.0
NOMINAL_WFS_M_PER_MIN: float = 7.0
NOMINAL_TRAVEL_SPEED_MM_PER_S: float = 6.0

# Nominal bead at the above operating point
NOMINAL_WIDTH_MM: float = 7.0
NOMINAL_HEIGHT_MM: float = 1.8

# Scaling exponents
# Width responds more to heat input than height does.
# Height responds more strongly to deposited metal per unit length.
WIDTH_HEAT_EXP: float = 0.35
WIDTH_DEP_EXP: float = 0.20
HEIGHT_HEAT_EXP: float = 0.10
HEIGHT_DEP_EXP: float = 0.75

# Conservative clamps for single bead-on-plate geometry
MIN_WIDTH_MM: float = 2.0
MAX_WIDTH_MM: float = 15.0
MIN_HEIGHT_MM: float = 0.4
MAX_HEIGHT_MM: float = 6.0


@dataclass(frozen=True)
class BeadDimensions:
    """Predicted weld bead cross-section dimensions."""
    width_m: float
    height_m: float
    contact_angle_rad: float  # Toe angle from the plate, kept for API compatibility.


def bead_dimensions_from_params(
    current_A: float,
    wfs_m_per_min: float,
    travel_speed_mm_per_s: float,
) -> BeadDimensions:
    """Predict bead dimensions from welding parameters.

    Inputs
    ------
    current_A : float
        Welding current in A.
    wfs_m_per_min : float
        Wire feed speed in m/min.
    travel_speed_mm_per_s : float
        Travel speed in mm/s.

    Returns
    -------
    BeadDimensions
        width_m, height_m, contact_angle_rad
    """
    I = max(current_A, 40.0)
    WFS = max(wfs_m_per_min, 1.0)
    TS = max(travel_speed_mm_per_s, 0.5)

    # Reduced-order surrogates:
    # heat_like increases with current and decreases with travel speed
    # dep_like increases with WFS and decreases with travel speed
    heat_like = (I / NOMINAL_CURRENT_A) * (NOMINAL_TRAVEL_SPEED_MM_PER_S / TS)
    dep_like = (WFS / NOMINAL_WFS_M_PER_MIN) * (NOMINAL_TRAVEL_SPEED_MM_PER_S / TS)

    width_mm = (
        NOMINAL_WIDTH_MM
        * (heat_like ** WIDTH_HEAT_EXP)
        * (dep_like ** WIDTH_DEP_EXP)
    )
    height_mm = (
        NOMINAL_HEIGHT_MM
        * (heat_like ** HEIGHT_HEAT_EXP)
        * (dep_like ** HEIGHT_DEP_EXP)
    )

    width_mm = _clamp(width_mm, MIN_WIDTH_MM, MAX_WIDTH_MM)
    height_mm = _clamp(height_mm, MIN_HEIGHT_MM, MAX_HEIGHT_MM)

    width_m = width_mm / 1000.0
    height_m = height_mm / 1000.0
    theta0 = _solve_contact_angle(width_m, height_m)

    return BeadDimensions(width_m, height_m, theta0)


def laplace_bead_profile(
    width_m: float,
    height_m: float,
    n_per_side: int = 16,
) -> list[tuple[float, float]]:
    """Compute the bead cross-section polygon.

    Public name is kept unchanged for drop-in compatibility.

    Returns a closed contour of (x, y) points in metres, centered at
    (0, 0), with y in [-H/2, +H/2].

    Geometry used:
        z(x) = H * (1 - (2x/W)^2)

    The returned polygon uses:
        x as transverse bead direction
        y as vertical direction centered around zero

    Solving the parabola for x as a function of z:
        x(z) = (W/2) * sqrt(1 - z/H)
    """
    H = max(height_m, 0.0)
    W = max(width_m, 0.0)
    hw = W / 2.0

    if H < 1e-9 or hw < 1e-9:
        return [(-hw, -H / 2.0), (0.0, H / 2.0), (hw, -H / 2.0)]

    n = max(int(n_per_side), 2)

    # Right side from toe (bottom) to apex (top)
    right: list[tuple[float, float]] = []
    for j in range(n + 1):
        z = H * j / n
        if H <= 1e-12:
            x = 0.0
        else:
            x = hw * math.sqrt(max(0.0, 1.0 - z / H))
        right.append((x, z - H / 2.0))

    # Build polygon as:
    # left side bottom->top, then right side top->bottom
    poly: list[tuple[float, float]] = []
    for j in range(n + 1):
        poly.append((-right[j][0], right[j][1]))
    for j in range(n - 1, -1, -1):
        poly.append((right[j][0], right[j][1]))

    return poly


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _half_width_for_theta(theta0: float, H: float) -> float:
    """Half-width implied by a parabolic section and toe angle.

    For z(x) = H * (1 - (2x/W)^2), the slope magnitude at the toe is:
        tan(theta0) = 4H / W

    Therefore:
        W/2 = 2H / tan(theta0)
    """
    theta0 = _clamp(theta0, 1e-4, 1.55)
    t = math.tan(theta0)
    if abs(t) < 1e-12:
        return 1e9
    return 2.0 * H / t


def _solve_contact_angle(width_m: float, height_m: float) -> float:
    """Toe angle implied by a parabolic cross-section."""
    W = max(width_m, 1e-12)
    H = max(height_m, 0.0)
    return math.atan(4.0 * H / W)
