"""Bead persistence — load/save completed beads to JSON."""

import json
import os
import time
from typing import Any

from robin_core.bead_layout import OccupiedRegion


def load_completed_beads(path: str) -> list[dict[str, Any]]:
    """Load completed bead records from a JSON file.

    Returns an empty list if the file does not exist or is invalid.
    """
    if not path or not os.path.isfile(path):
        return []
    try:
        with open(path) as f:
            payload = json.load(f)
        beads = payload if isinstance(payload, list) else payload.get('beads', [])
        return beads if isinstance(beads, list) else []
    except (json.JSONDecodeError, OSError):
        return []


def get_occupied_slots(path: str) -> dict[str, set[tuple[int, int]]]:
    """Return occupied (row, col) sets keyed by plate_id.

    Reads the beads JSON and returns a dict like:
        {"plate_A": {(0, 0), (0, 1)}, "plate_B": {(1, 0)}}
    """
    beads = load_completed_beads(path)
    occupied: dict[str, set[tuple[int, int]]] = {}
    for b in beads:
        pid = b.get('plate_id', '')
        row = b.get('slot_row', -1)
        col = b.get('slot_col', -1)
        if pid and row >= 0 and col >= 0:
            occupied.setdefault(pid, set()).add((row, col))
    return occupied


def get_occupied_regions(path: str) -> dict[str, list[OccupiedRegion]]:
    """Return occupied physical regions keyed by plate_id.

    Only records that include ``local_x_start``, ``local_x_end``, and
    ``local_y`` are returned.  Legacy records without these fields are
    silently skipped (use ``get_occupied_slots`` for those).
    """
    beads = load_completed_beads(path)
    regions: dict[str, list[OccupiedRegion]] = {}
    for b in beads:
        pid = b.get('plate_id', '')
        x_start = b.get('local_x_start')
        x_end = b.get('local_x_end')
        y = b.get('local_y')
        if pid and x_start is not None and x_end is not None and y is not None:
            regions.setdefault(pid, []).append(
                OccupiedRegion(float(x_start), float(x_end), float(y)))
    return regions


def save_completed_bead(path: str, bead_record: dict[str, Any]) -> None:
    """Append a completed bead record to the beads JSON file.

    Creates the file if it does not exist. Uses atomic write via rename.
    """
    beads = load_completed_beads(path)
    beads.append(bead_record)

    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)

    payload = {'beads': beads}
    tmp_path = path + '.tmp'
    with open(tmp_path, 'w') as f:
        json.dump(payload, f, indent=2)
        f.write('\n')
    os.replace(tmp_path, path)


def make_bead_record(
    bead_id: str,
    plate_id: str,
    slot_row: int,
    slot_col: int,
    path: list,
    total_length: float,
    local_x_start: float = 0.0,
    local_x_end: float = 0.0,
    local_y: float = 0.0,
) -> dict[str, Any]:
    """Create a bead record dict for persistence."""
    return {
        'bead_id': bead_id,
        'plate_id': plate_id,
        'slot_row': slot_row,
        'slot_col': slot_col,
        'local_x_start': local_x_start,
        'local_x_end': local_x_end,
        'local_y': local_y,
        'path': [{'x': p.x, 'y': p.y, 'z': p.z} for p in path],
        'total_length': total_length,
        'completed_at': time.time(),
    }


# =========================================================================
# Slot reservation — mark slots as occupied on partially-used plates
# =========================================================================

def save_reserved_slots(
    path: str,
    plate_id: str,
    regions: list[tuple[float, float, float]],
    slot_labels: list[tuple[int, int]] | None = None,
) -> int:
    """Mark physical regions as reserved (occupied) in the beads JSON.

    Each region is a ``(local_x_start, local_x_end, local_y)`` tuple in
    plate-local inward coordinates.

    *slot_labels* is an optional parallel list of ``(row, col)`` indices
    kept for display / backward-compat but NOT used for overlap checks.

    Already-occupied regions whose centres are within 1 mm of a new
    region are silently skipped.

    Returns the number of newly reserved regions.
    """
    beads = load_completed_beads(path)

    # Build set of existing regions for O(1) de-dup lookup
    existing: set[tuple[str, float, float, float]] = set()
    for b in beads:
        pid = b.get('plate_id', '')
        xs = b.get('local_x_start')
        xe = b.get('local_x_end')
        y = b.get('local_y')
        if pid and xs is not None and xe is not None and y is not None:
            existing.add((pid, round(float(xs), 3), round(float(xe), 3), round(float(y), 3)))

    added = 0
    for idx, (x_start, x_end, y) in enumerate(regions):
        key = (plate_id, round(x_start, 3), round(x_end, 3), round(y, 3))
        if key in existing:
            continue
        row = slot_labels[idx][0] if slot_labels and idx < len(slot_labels) else -1
        col = slot_labels[idx][1] if slot_labels and idx < len(slot_labels) else -1
        bead_id = f'reserved_{plate_id}_{added}'
        beads.append({
            'bead_id': bead_id,
            'plate_id': plate_id,
            'slot_row': row,
            'slot_col': col,
            'local_x_start': x_start,
            'local_x_end': x_end,
            'local_y': y,
            'reserved': True,
            'reserved_at': time.time(),
        })
        existing.add(key)
        added += 1

    if added > 0:
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        tmp_path = path + '.tmp'
        with open(tmp_path, 'w') as f:
            json.dump({'beads': beads}, f, indent=2)
            f.write('\n')
        os.replace(tmp_path, path)

    return added


def get_reserved_slots(path: str) -> dict[str, set[tuple[int, int]]]:
    """Return only reserved (not completed) slots keyed by plate_id."""
    beads = load_completed_beads(path)
    reserved: dict[str, set[tuple[int, int]]] = {}
    for b in beads:
        if not b.get('reserved', False):
            continue
        pid = b.get('plate_id', '')
        row = b.get('slot_row', -1)
        col = b.get('slot_col', -1)
        if pid and row >= 0 and col >= 0:
            reserved.setdefault(pid, set()).add((row, col))
    return reserved


def clear_reserved_slots(
    path: str,
    plate_id: str = '',
    slots: set[tuple[int, int]] | None = None,
    all_plates: bool = False,
    bead_ids: set[str] | None = None,
) -> int:
    """Remove reserved-slot records from the beads JSON.

    Only records with ``"reserved": true`` are removed — completed bead
    records are never touched.

    Args:
        path: Path to beads JSON file.
        plate_id: Plate to clear reservations for (ignored if *all_plates*).
        slots: Specific ``(row, col)`` tuples to clear.  If *None* or empty,
            all reservations for the plate are cleared.
        all_plates: If *True*, clear every reservation regardless of plate.
        bead_ids: Specific reservation bead_ids to clear.

    Returns:
        Number of reservation records removed.
    """
    beads = load_completed_beads(path)
    if not beads:
        return 0

    def _keep(b: dict) -> bool:
        if not b.get('reserved', False):
            return True  # never remove completed beads
        if all_plates:
            return False
        if bead_ids and b.get('bead_id', '') in bead_ids:
            return False
        if plate_id and b.get('plate_id') == plate_id:
            if not slots:
                return False  # clear all for this plate
            return (b.get('slot_row', -1), b.get('slot_col', -1)) not in slots
        return True

    filtered = [b for b in beads if _keep(b)]
    removed = len(beads) - len(filtered)

    if removed > 0:
        tmp_path = path + '.tmp'
        with open(tmp_path, 'w') as f:
            json.dump({'beads': filtered}, f, indent=2)
            f.write('\n')
        os.replace(tmp_path, path)

    return removed


def clear_all_beads(path: str) -> int:
    """Remove ALL bead records (completed + reserved) from the beads JSON.

    Returns the number of records removed.
    """
    beads = load_completed_beads(path)
    if not beads:
        return 0

    count = len(beads)
    tmp_path = path + '.tmp'
    with open(tmp_path, 'w') as f:
        json.dump({'beads': []}, f, indent=2)
        f.write('\n')
    os.replace(tmp_path, path)
    return count
