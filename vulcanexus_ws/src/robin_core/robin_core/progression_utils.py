"""
Shared utility for projecting a TCP position onto a bead path polyline and
returning a normalised arc-length progression value in [0, 1].
"""

import numpy as np
from geometry_msgs.msg import Point


def calculate_progression(
    tcp_pos: Point,
    bead_path: list,
    total_length: float,
) -> float:
    """Project *tcp_pos* onto the bead polyline and return arc-length / total_length.

    Args:
        tcp_pos: Current TCP position in the base frame.
        bead_path: Ordered list of geometry_msgs/Point waypoints defining the bead.
        total_length: Pre-computed total arc length of the path (metres).

    Returns:
        Normalised progression in [0.0, 1.0], or 0.0 if the path is invalid.
    """
    if total_length <= 0.0 or len(bead_path) < 2:
        return 0.0

    tcp = np.array([tcp_pos.x, tcp_pos.y, tcp_pos.z])
    pts = np.array([[p.x, p.y, p.z] for p in bead_path])
    segs = np.diff(pts, axis=0)
    seg_lens_sq = np.sum(segs ** 2, axis=1)

    best_dist_sq = float('inf')
    best_arc = 0.0
    cumulative = 0.0

    for i in range(len(segs)):
        sl_sq = seg_lens_sq[i]
        if sl_sq < 1e-12:
            continue

        t = float(np.clip(np.dot(tcp - pts[i], segs[i]) / sl_sq, 0.0, 1.0))
        proj = pts[i] + t * segs[i]
        dist_sq = float(np.sum((tcp - proj) ** 2))
        sl = float(np.sqrt(sl_sq))

        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_arc = cumulative + t * sl

        cumulative += sl

    return float(np.clip(best_arc / total_length, 0.0, 1.0))
