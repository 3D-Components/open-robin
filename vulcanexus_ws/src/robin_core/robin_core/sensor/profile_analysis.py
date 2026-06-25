"""Weld profile analysis — shared RANSAC + toe detection algorithm."""

import warnings
import numpy as np
from sklearn.linear_model import RANSACRegressor


def analyse_weld_profile(points_mm: np.ndarray):
    """Run two-pass RANSAC base fitting and interpolated toe detection.

    Parameters
    ----------
    points_mm : (N, 3) array in millimetres, already sorted by Y.

    Returns
    -------
    dict with keys:
        y, z           – sorted 1-D arrays
        z_base         – fitted base plane Z values
        z_relative     – height above base
        base_mask      – bool mask of base-plate inliers
        bead_threshold – adaptive detection threshold (mm)
        y_left, y_right, width, height – bead dimensions (or None)
        start_idx, end_idx – indices of selected bead segment (or None)
        starts, ends   – all bead segment boundary arrays
    """
    y = points_mm[:, 1]
    z = points_mm[:, 2]
    y_reshape = y.reshape(-1, 1)

    # Pre-filter: lower-Z half as RANSAC candidates
    z_median = np.median(z)
    low_mask = z <= z_median

    # Pass 1 RANSAC on low-Z subset
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="R.*score is not well-defined")
        ransac = RANSACRegressor(min_samples=2, residual_threshold=0.2, max_trials=500)
        ransac.fit(y_reshape[low_mask], z[low_mask])

    # Identify base plate inliers from ALL points
    z_pred_all = ransac.predict(y_reshape)
    residuals_all = np.abs(z - z_pred_all)
    base_mask = residuals_all <= 0.2

    # Pass 2 RANSAC on base plate points only
    if np.sum(base_mask) >= 2:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="R.*score is not well-defined")
            ransac2 = RANSACRegressor(min_samples=2, residual_threshold=0.2)
            ransac2.fit(y_reshape[base_mask], z[base_mask])
            z_base = ransac2.predict(y_reshape)
    else:
        z_base = ransac.predict(y_reshape)

    z_relative = z - z_base

    # Adaptive threshold
    base_res = z_relative[base_mask] if np.any(base_mask) else z_relative
    sigma_noise = np.std(base_res)
    bead_threshold = max(0.15, 3.0 * sigma_noise)

    # Toe detection — find contiguous bead segments
    is_bead = z_relative > bead_threshold
    edges = np.diff(is_bead.astype(int))
    starts = np.where(edges == 1)[0] + 1
    ends = np.where(edges == -1)[0] + 1
    if is_bead[0]:
        starts = np.insert(starts, 0, 0)
    if is_bead[-1]:
        ends = np.append(ends, len(is_bead))

    result = dict(
        y=y, z=z, z_base=z_base, z_relative=z_relative,
        base_mask=base_mask, bead_threshold=bead_threshold,
        starts=starts, ends=ends,
        y_left=None, y_right=None, width=None, height=None,
        start_idx=None, end_idx=None,
        left_toe_angle_rad=None, right_toe_angle_rad=None,
        toe_angle_rad=None,
    )

    if len(starts) == 0:
        return result

    # Filter noise segments (< 2 mm)
    valid = [i for i in range(len(starts))
             if (y[ends[i] - 1] - y[starts[i]]) >= 2.0]
    if not valid:
        return result

    # Select bead closest to FOV centre
    y_center = (y[0] + y[-1]) / 2.0
    best = min(valid, key=lambda i: abs(
        (y[starts[i]] + y[ends[i] - 1]) / 2.0 - y_center))
    si = starts[best]
    ei = ends[best] - 1

    # Interpolated left toe
    if si > 0:
        dz = z_relative[si] - z_relative[si - 1]
        if abs(dz) > 1e-9:
            t = (bead_threshold - z_relative[si - 1]) / dz
            y_left = y[si - 1] + t * (y[si] - y[si - 1])
        else:
            y_left = y[si]
    else:
        y_left = y[si]

    # Interpolated right toe
    if ei < len(y) - 1:
        dz = z_relative[ei + 1] - z_relative[ei]
        if abs(dz) > 1e-9:
            t = (bead_threshold - z_relative[ei]) / dz
            y_right = y[ei] + t * (y[ei + 1] - y[ei])
        else:
            y_right = y[ei]
    else:
        y_right = y[ei]

    result.update(
        y_left=y_left, y_right=y_right,
        width=y_right - y_left,
        height=float(np.max(z_relative[si:ei + 1])),
        start_idx=si, end_idx=ei,
    )

    # ------------------------------------------------------------------
    # Toe angles – the obtuse angle between base plate and bead flank
    # at each toe.  θ = π − arctan(|dz/dy|)
    # ------------------------------------------------------------------
    left_toe = None
    if si + 1 <= ei:
        dy = y[si + 1] - y[si]
        if abs(dy) > 1e-9:
            slope = (z_relative[si + 1] - z_relative[si]) / dy
            left_toe = np.pi - np.arctan(abs(slope))

    right_toe = None
    if ei - 1 >= si:
        dy = y[ei] - y[ei - 1]
        if abs(dy) > 1e-9:
            slope = (z_relative[ei] - z_relative[ei - 1]) / dy
            right_toe = np.pi - np.arctan(abs(slope))

    if left_toe is not None and right_toe is not None:
        toe_angle = (left_toe + right_toe) / 2.0
    elif left_toe is not None:
        toe_angle = left_toe
    elif right_toe is not None:
        toe_angle = right_toe
    else:
        toe_angle = None

    result.update(
        left_toe_angle_rad=left_toe,
        right_toe_angle_rad=right_toe,
        toe_angle_rad=float(toe_angle) if toe_angle is not None else None,
    )

    return result
