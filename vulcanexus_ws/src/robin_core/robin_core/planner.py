"""Experiment planning — bead-to-plate assignment and capacity computation."""

from robin_interfaces.msg import ExperimentBead

from robin_core.bead_layout import OccupiedRegion, compute_plate_capacity


def assign_beads_to_plates(bead_specs, plates,
                           spacing_x: float, spacing_y: float,
                           margin_x: float, margin_y: float,
                           bead_length: float,
                           staggered: bool,
                           occupied_slots: dict[str, set[tuple[int, int]]] | None = None,
                           occupied_regions: dict[str, list[OccupiedRegion]] | None = None):
    """Assign beads round-robin to plates based on occupancy capacity.

    Returns (planned_beads, bead_runtime) tuple, or None if capacity
    is insufficient.
    """
    capacities = {}
    plate_ids = []
    for plate in plates:
        pid = plate.plate_id
        plate_ids.append(pid)
        plate_occupied = (occupied_slots or {}).get(pid, set())
        plate_occ_regions = (occupied_regions or {}).get(pid, [])
        capacities[pid] = compute_plate_capacity(
            plate_length=float(plate.length),
            plate_width=float(plate.width),
            margin_x=float(margin_x),
            margin_y=float(margin_y),
            bead_length_m=float(bead_length),
            spacing_x=float(spacing_x),
            spacing_y=float(spacing_y),
            staggered=bool(staggered),
            occupied=plate_occupied,
            occupied_regions=plate_occ_regions or None,
        )

    if not plate_ids:
        return None

    next_plate_idx = 0
    planned_beads = []
    bead_runtime = {}
    per_plate_counts = {pid: 0 for pid in plate_ids}

    for index, spec in enumerate(bead_specs, start=1):
        selected_pid = None
        for _ in range(len(plate_ids)):
            candidate = plate_ids[next_plate_idx]
            next_plate_idx = (next_plate_idx + 1) % len(plate_ids)
            if per_plate_counts[candidate] < capacities[candidate]:
                selected_pid = candidate
                break

        if selected_pid is None:
            return None

        per_plate_counts[selected_pid] += 1
        if hasattr(spec, 'input_id') and spec.input_id:
            bead_id = spec.input_id
        else:
            bead_id = f"B{index:03d}"

        bead = ExperimentBead()
        bead.bead_id = bead_id
        bead.plate_id = selected_pid
        bead.weld_speed = float(spec.weld_speed)
        bead.wire_feed_speed = float(spec.wire_feed_speed)
        bead.arc_length_correction_mm = float(spec.arc_length_correction_mm)
        planned_beads.append(bead)

        bead_runtime[bead_id] = {
            "arc_length_correction_mm": float(spec.arc_length_correction_mm),
        }

    return planned_beads, bead_runtime
