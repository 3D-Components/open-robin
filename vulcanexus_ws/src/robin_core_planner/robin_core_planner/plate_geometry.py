"""Shared plate/corner geometry helpers.

All plate local coordinates are expressed as *inward* distances from the
selected touched corner:
  - +x_inward: along plate length towards the opposite edge
  - +y_inward: along plate width towards the opposite edge
"""

import math


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
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    wx = float(corner_x) + float(local_x) * c - float(local_y) * s
    wy = float(corner_y) + float(local_x) * s + float(local_y) * c
    return wx, wy


def inward_to_world(corner_x: float, corner_y: float, yaw: float,
                    corner_id: str, x_inward: float, y_inward: float) -> tuple[float, float]:
    lx, ly = inward_to_local(corner_id, x_inward, y_inward)
    return local_to_world(corner_x, corner_y, yaw, lx, ly)
