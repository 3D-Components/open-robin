"""Bead layout computation — pure geometry, no ROS dependencies."""

import math
from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import Point, Quaternion
from robin_interfaces.msg import PlateLayout
from scipy.spatial.transform import Rotation
from robin_core.plate_geometry import normalize_corner_id, inward_to_world


@dataclass
class OccupiedRegion:
    """Physical bead footprint in plate-local inward coordinates."""
    x_start: float  # inward X start (m)
    x_end: float    # inward X end (m)
    y: float        # inward Y centre (m)


@dataclass
class PhysicalBead:
    """Internal representation of a bead with computed physical positions."""
    bead_id: str
    plate_id: str
    path: list  # list[Point] — ordered waypoints defining the bead centreline
    total_length: float  # pre-calculated total path length (m)
    weld_speed: float
    wire_feed_speed: float
    slot_row: int = -1
    slot_col: int = -1
    local_x_start: float = 0.0
    local_x_end: float = 0.0
    local_y: float = 0.0

    @property
    def start_point(self) -> Point:
        """First waypoint (backward-compatible convenience)."""
        return self.path[0]

    @property
    def end_point(self) -> Point:
        """Last waypoint (backward-compatible convenience)."""
        return self.path[-1]


@dataclass
class PlateSlot:
    """Discrete non-overlapping placement slot in plate-local coordinates."""
    row: int
    col: int
    local_x_start: float
    local_x_end: float
    local_y: float


def _segments_overlap(a0: float, a1: float, b0: float, b1: float, eps: float = 1e-9) -> bool:
    return max(a0, b0) < (min(a1, b1) - eps)


def slot_overlaps_any(slot: PlateSlot, regions: list[OccupiedRegion],
                      min_y_clearance: float) -> bool:
    """Return True if *slot* physically overlaps any occupied region.

    Two footprints conflict when their X segments overlap **and** their
    Y centre-lines are closer than *min_y_clearance*.
    """
    for r in regions:
        if _segments_overlap(slot.local_x_start, slot.local_x_end, r.x_start, r.x_end):
            if abs(slot.local_y - r.y) < min_y_clearance - 1e-9:
                return True
    return False


def compute_plate_occupancy_slots(
    plate_length: float,
    plate_width: float,
    margin_x: float,
    margin_y: float,
    bead_length_m: float,
    spacing_x: float,
    spacing_y: float,
    staggered: bool,
    logger=None,
) -> list[PlateSlot]:
    """Compute deterministic, non-overlapping bead slots for a plate.

    `spacing_x` is interpreted as edge-to-edge gap between adjacent bead columns.
    `spacing_y` is interpreted as row pitch between parallel bead center-lines.
    """
    usable_x_max = float(plate_length) - float(margin_x)
    usable_y_max = float(plate_width) - float(margin_y)
    start_x_base = float(margin_x)
    start_y_base = float(margin_y)

    if bead_length_m <= 0.0 or spacing_y <= 0.0:
        return []
    if start_x_base + bead_length_m > usable_x_max:
        return []
    if start_y_base > usable_y_max:
        return []

    column_pitch_x = float(bead_length_m) + max(0.0, float(spacing_x))
    if column_pitch_x <= 0.0:
        return []

    slots: list[PlateSlot] = []
    row = 0
    while True:
        y = start_y_base + row * float(spacing_y)
        if y > usable_y_max + 1e-9:
            break

        row_offset = 0.5 * column_pitch_x if (staggered and (row % 2 == 1)) else 0.0
        row_start = start_x_base + row_offset

        # If stagger pushes first column out of bounds, fall back to non-staggered row.
        if row_start + bead_length_m > usable_x_max and row_offset > 0.0:
            row_start = start_x_base

        col = 0
        row_added = 0
        while True:
            x0 = row_start + col * column_pitch_x
            x1 = x0 + bead_length_m
            if x1 > usable_x_max + 1e-9:
                break

            # Defensive overlap check against prior slots on the same row.
            if not any(abs(s.local_y - y) <= 1e-9
                       and _segments_overlap(x0, x1, s.local_x_start, s.local_x_end)
                       for s in slots):
                slots.append(PlateSlot(
                    row=row,
                    col=col,
                    local_x_start=x0,
                    local_x_end=x1,
                    local_y=y,
                ))
                row_added += 1

            col += 1

        if row_added == 0 and logger:
            logger.debug(
                f"No columns available on row {row} (y={y:.4f}) with staggered={staggered}")

        row += 1

    return slots


def compute_plate_capacity(
    plate_length: float,
    plate_width: float,
    margin_x: float,
    margin_y: float,
    bead_length_m: float,
    spacing_x: float,
    spacing_y: float,
    staggered: bool,
    occupied: set[tuple[int, int]] | None = None,
    occupied_regions: list[OccupiedRegion] | None = None,
) -> int:
    """Return available (non-occupied) slot capacity for a plate.

    When *occupied_regions* is provided, each candidate slot is checked
    for physical overlap (X-segment intersection **and** Y within
    spacing_y).  The legacy *occupied* set is still supported for
    backward-compat but should be considered deprecated.
    """
    slots = compute_plate_occupancy_slots(
        plate_length=plate_length,
        plate_width=plate_width,
        margin_x=margin_x,
        margin_y=margin_y,
        bead_length_m=bead_length_m,
        spacing_x=spacing_x,
        spacing_y=spacing_y,
        staggered=staggered,
        logger=None,
    )
    if occupied_regions:
        slots = [s for s in slots
                 if not slot_overlaps_any(s, occupied_regions, spacing_y)]
    elif occupied:
        slots = [s for s in slots if (s.row, s.col) not in occupied]
    return len(slots)


def compute_physical_beads(
    beads: list,
    plates: list,
    layout: PlateLayout,
    default_bead_pitch: float,
    default_margin_x: float,
    default_margin_y: float,
    bead_length_m: float = 0.10,
    occupied_slots: dict[str, set[tuple[int, int]]] | None = None,
    occupied_regions: dict[str, list[OccupiedRegion]] | None = None,
    logger=None,
) -> list[PhysicalBead]:
    """Compute physical positions for experiment beads on plates.

    When *occupied_regions* is provided, candidate slots are filtered by
    physical overlap (X-segment intersection + Y proximity within
    spacing_y).  The legacy *occupied_slots* parameter is still accepted
    for backward-compat but is ignored when *occupied_regions* is given.

    Args:
        beads: ExperimentBead messages (parameters + plate_id)
        plates: WeldPlate messages (position + dimensions)
        layout: PlateLayout message (pitch, margins)
        default_bead_pitch: Fallback pitch when layout.bead_pitch == 0
        default_margin_x: Fallback X margin
        default_margin_y: Fallback Y margin
        logger: Optional ROS logger for warnings

    Returns:
        List of PhysicalBead with computed start/end points
    """
    spacing_y = float(getattr(layout, 'spacing_y', 0.0))
    if spacing_y <= 0.0:
        spacing_y = layout.bead_pitch if layout.bead_pitch > 0 else default_bead_pitch
    spacing_x = float(getattr(layout, 'spacing_x', 0.0))
    if spacing_x <= 0.0:
        spacing_x = spacing_y
    staggered = bool(getattr(layout, 'staggered', False))
    margin_x = layout.margin_x if layout.margin_x > 0 else default_margin_x
    margin_y = layout.margin_y if layout.margin_y > 0 else default_margin_y

    plate_map = {p.plate_id: p for p in plates}

    # Precompute slots and metadata per plate, then consume slots in original bead order.
    slots_by_plate: dict[str, list[PlateSlot]] = {}
    slot_index_by_plate: dict[str, int] = {}
    meta_by_plate: dict[str, tuple[float, bool, float, float, float, str]] = {}

    for plate in plates:
        plate_id = plate.plate_id
        slots = compute_plate_occupancy_slots(
            plate_length=float(plate.length),
            plate_width=float(plate.width),
            margin_x=margin_x,
            margin_y=margin_y,
            bead_length_m=bead_length_m,
            spacing_x=spacing_x,
            spacing_y=spacing_y,
            staggered=staggered,
            logger=logger,
        )
        # Filter out slots already occupied by previously completed beads
        plate_regions = (occupied_regions or {}).get(plate_id, [])
        if plate_regions:
            slots = [s for s in slots
                     if not slot_overlaps_any(s, plate_regions, spacing_y)]
        else:
            plate_occupied = (occupied_slots or {}).get(plate_id, set())
            if plate_occupied:
                slots = [s for s in slots if (s.row, s.col) not in plate_occupied]

        slots_by_plate[plate_id] = slots
        slot_index_by_plate[plate_id] = 0

        base_surface_z = plate.surface_z if plate.is_calibrated else plate.origin.z
        use_plane = bool(getattr(plate, 'plane_calibrated', False))
        plane_a = float(getattr(plate, 'plane_a', 0.0))
        plane_b = float(getattr(plate, 'plane_b', 0.0))
        plane_c = float(getattr(plate, 'plane_c', base_surface_z))
        corner_id = normalize_corner_id(getattr(plate, 'corner_id', 'front_left'))
        meta_by_plate[plate_id] = (
            base_surface_z,
            use_plane,
            plane_a,
            plane_b,
            plane_c,
            corner_id,
        )

        if not slots and logger:
            logger.warn(
                f"Plate '{plate_id}' has no valid occupancy slots "
                f"(L={plate.length:.3f}, W={plate.width:.3f}, bead={bead_length_m:.3f}, "
                f"gap_x={spacing_x:.3f}, pitch_y={spacing_y:.3f})")

    physical_beads: list[PhysicalBead] = []

    for bead in beads:
        plate_id = bead.plate_id
        plate = plate_map.get(plate_id)
        if plate is None:
            if logger:
                logger.error(f"Plate '{plate_id}' not found, skipping bead '{bead.bead_id}'")
            continue

        slots = slots_by_plate.get(plate_id, [])
        slot_idx = slot_index_by_plate.get(plate_id, 0)
        if slot_idx >= len(slots):
            if logger:
                logger.warn(
                    f"Plate '{plate_id}' out of occupancy slots; skipping bead '{bead.bead_id}'")
            continue

        slot = slots[slot_idx]
        slot_index_by_plate[plate_id] = slot_idx + 1

        base_surface_z, use_plane, plane_a, plane_b, plane_c, corner_id = meta_by_plate[plate_id]

        start = Point()
        start.x, start.y = inward_to_world(
            plate.origin.x, plate.origin.y, plate.orientation,
            corner_id, slot.local_x_start, slot.local_y)
        start.z = (plane_a * start.x + plane_b * start.y + plane_c) if use_plane else base_surface_z

        end = Point()
        end.x, end.y = inward_to_world(
            plate.origin.x, plate.origin.y, plate.orientation,
            corner_id, slot.local_x_end, slot.local_y)
        end.z = (plane_a * end.x + plane_b * end.y + plane_c) if use_plane else base_surface_z

        bead_path = [start, end]
        bead_length = float(np.linalg.norm(
            [end.x - start.x, end.y - start.y, end.z - start.z]))

        physical_beads.append(PhysicalBead(
            bead_id=bead.bead_id,
            plate_id=plate_id,
            path=bead_path,
            total_length=bead_length,
            weld_speed=bead.weld_speed,
            wire_feed_speed=bead.wire_feed_speed,
            slot_row=slot.row,
            slot_col=slot.col,
            local_x_start=slot.local_x_start,
            local_x_end=slot.local_x_end,
            local_y=slot.local_y,
        ))

    return physical_beads


def compute_path_length(path: list) -> float:
    """Compute total arc length of a polyline path of geometry_msgs/Point."""
    if len(path) < 2:
        return 0.0
    pts = np.array([[p.x, p.y, p.z] for p in path])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def compute_weld_orientation(start: Point, end: Point, logger=None) -> Quaternion:
    """Compute tool orientation from weld axis (not travel direction).

    Uses a base orientation (tool pointing down) and rotates around Z
    to align with the bead axis. Opposite travel directions on the same
    axis produce the same orientation, avoiding 180° flips between beads.
    """
    dx = end.x - start.x
    dy = end.y - start.y
    yaw = math.atan2(dy, dx)

    # Direction-invariant orientation: treat +axis and -axis as the same
    # physical weld axis so tool orientation does not flip between passes.
    if yaw > (math.pi / 2.0):
        yaw -= math.pi
    elif yaw < (-math.pi / 2.0):
        yaw += math.pi

    base_rot = Rotation.from_quat([0.7071, 0.7071, 0.0, 0.0])
    yaw_rot = Rotation.from_euler('z', yaw)
    combined_rot = yaw_rot * base_rot

    q = combined_rot.as_quat()  # [x, y, z, w]

    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]

    if logger:
        logger.info(
            f"Weld axis yaw: {math.degrees(yaw):.1f}°, "
            f"orientation: [{quat.x:.4f}, {quat.y:.4f}, {quat.z:.4f}, {quat.w:.4f}]")

    return quat


def compute_weld_velocity_scaling(
    target_speed: float, max_cartesian_velocity: float, logger=None,
) -> float:
    """Compute Pilz velocity scaling factor from target weld speed.

    actual_velocity = max_trans_vel * velocity_scaling_factor
    """
    if target_speed <= 0:
        if logger:
            logger.warn(f"Invalid target_speed {target_speed}, using minimum")
        return 0.01

    scaling = target_speed / max_cartesian_velocity

    if scaling > 1.0 and logger:
        logger.warn(
            f"Target speed {target_speed:.4f} m/s exceeds max Cartesian velocity "
            f"{max_cartesian_velocity:.4f} m/s, clamping to 1.0")

    return max(0.01, min(1.0, scaling))
