"""Shared plate/corner geometry helpers.

All plate local coordinates are expressed as *inward* distances from the
selected touched corner:
  - +x_inward: along plate length towards the opposite edge
  - +y_inward: along plate width towards the opposite edge
"""

import numpy as np


VALID_CORNERS = {"front_left", "front_right", "rear_left", "rear_right"}


def normalize_corner_id(corner_id: str) -> str:
    key = str(corner_id).strip().lower()
    if key not in VALID_CORNERS:
        return "front_left"
    return key


def inward_signs(corner_id: str) -> tuple[float, float]:
    key = normalize_corner_id(corner_id)
    x_sign = 1.0 if key.endswith("left") else -1.0
    y_sign = -1.0 if key.startswith("front") else 1.0
    return x_sign, y_sign


def inward_to_local(corner_id: str, x_inward: float, y_inward: float) -> tuple[float, float]:
    x_sign, y_sign = inward_signs(corner_id)
    return x_sign * float(x_inward), y_sign * float(y_inward)


def local_to_world(corner_x: float, corner_y: float, yaw: float,
                   local_x: float, local_y: float) -> tuple[float, float]:
    c, s = np.cos(float(yaw)), np.sin(float(yaw))
    lx, ly = float(local_x), float(local_y)
    wx = float(corner_x) + lx * c - ly * s
    wy = float(corner_y) + lx * s + ly * c
    return wx, wy


def inward_to_world(corner_x: float, corner_y: float, yaw: float,
                    corner_id: str, x_inward: float, y_inward: float) -> tuple[float, float]:
    lx, ly = inward_to_local(corner_id, x_inward, y_inward)
    return local_to_world(corner_x, corner_y, yaw, lx, ly)


def world_to_inward(corner_x: float, corner_y: float, yaw: float,
                    corner_id: str, wx: float, wy: float) -> tuple[float, float]:
    """Inverse of inward_to_world: world coords → plate inward coords."""
    c, s = np.cos(float(yaw)), np.sin(float(yaw))
    dx = float(wx) - float(corner_x)
    dy = float(wy) - float(corner_y)
    # Inverse rotation: local = R^T @ delta
    lx = dx * c + dy * s
    ly = -dx * s + dy * c
    # Inverse of inward_to_local
    x_sign, y_sign = inward_signs(corner_id)
    return lx / x_sign, ly / y_sign


def corner_to_center(corner_x: float, corner_y: float, yaw: float,
                     corner_id: str, length: float, width: float) -> tuple[float, float]:
    """Compute plate centre (world) from a known corner position."""
    return inward_to_world(corner_x, corner_y, yaw, corner_id,
                           float(length) / 2.0, float(width) / 2.0)


def center_to_corner(center_x: float, center_y: float, yaw: float,
                     corner_id: str, length: float, width: float) -> tuple[float, float]:
    """Compute corner position (world) from plate centre."""
    # offset = how far center is from the corner in world frame
    ox, oy = corner_to_center(0.0, 0.0, yaw, corner_id, length, width)
    return float(center_x) - ox, float(center_y) - oy
