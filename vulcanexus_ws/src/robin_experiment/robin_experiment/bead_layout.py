"""Bead layout computation — pure geometry, no ROS dependencies."""

import math
from dataclasses import dataclass

from geometry_msgs.msg import Point, Quaternion
from robin_interfaces.msg import ExperimentBead, PlateLayout
from scipy.spatial.transform import Rotation
from robin_experiment.plate_geometry import normalize_corner_id, inward_to_world


@dataclass
class PhysicalBead:
    """Internal representation of a bead with computed physical positions."""
    bead_id: str
    plate_id: str
    start_point: Point
    end_point: Point
    target_speed: float
    primary_parameter: int
    primary_value: float
    target_current: float
    target_voltage: float
    wire_feed_speed: float


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
    """Compute deterministic, non-overlapping bead slots for a plate."""
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

        if row_start + bead_length_m > usable_x_max and row_offset > 0.0:
            row_start = start_x_base

        col = 0
        row_added = 0
        while True:
            x0 = row_start + col * column_pitch_x
            x1 = x0 + bead_length_m
            if x1 > usable_x_max + 1e-9:
                break

            overlaps = False
            for s in slots:
                if abs(s.local_y - y) <= 1e-9 and _segments_overlap(x0, x1, s.local_x_start, s.local_x_end):
                    overlaps = True
                    break

            if not overlaps:
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
) -> int:
    """Return non-overlapping slot capacity for a plate."""
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
    return len(slots)


def compute_physical_beads(
    beads: list,
    plates: list,
    layout: PlateLayout,
    default_bead_pitch: float,
    default_margin_x: float,
    default_margin_y: float,
    bead_length_m: float = 0.10,
    logger=None,
) -> list[PhysicalBead]:
    """Compute physical positions for experiment beads on plates."""
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

        current, voltage, wfs = resolve_weld_params(bead)

        physical_beads.append(PhysicalBead(
            bead_id=bead.bead_id,
            plate_id=plate_id,
            start_point=start,
            end_point=end,
            target_speed=bead.target_speed,
            primary_parameter=bead.primary_parameter,
            primary_value=bead.primary_value,
            target_current=current,
            target_voltage=voltage,
            wire_feed_speed=wfs,
        ))

    return physical_beads


def resolve_weld_params(bead: ExperimentBead) -> tuple[float, float, float]:
    """Resolve welding parameters from primary parameter + optional overrides."""
    current = bead.target_current
    voltage = bead.target_voltage
    wfs = bead.wire_feed_speed

    if bead.primary_parameter == ExperimentBead.PRIMARY_CURRENT:
        current = bead.primary_value
    elif bead.primary_parameter == ExperimentBead.PRIMARY_VOLTAGE:
        voltage = bead.primary_value
    elif bead.primary_parameter == ExperimentBead.PRIMARY_WIRE_FEED_SPEED:
        wfs = bead.primary_value

    return current, voltage, wfs


def compute_weld_orientation(start: Point, end: Point, logger=None) -> Quaternion:
    """Compute tool orientation from weld axis."""
    dx = end.x - start.x
    dy = end.y - start.y
    yaw = math.atan2(dy, dx)

    if yaw > (math.pi / 2.0):
        yaw -= math.pi
    elif yaw < (-math.pi / 2.0):
        yaw += math.pi

    base_rot = Rotation.from_quat([0.7071, 0.7071, 0.0, 0.0])
    yaw_rot = Rotation.from_euler('z', yaw)
    combined_rot = yaw_rot * base_rot

    q = combined_rot.as_quat()

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
    """Compute Pilz velocity scaling factor from target weld speed."""
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
