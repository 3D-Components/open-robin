#!/usr/bin/env python3
"""Unit tests for bead persistence, physical-coordinate overlap, and layout."""

import json
import math
import os
import tempfile

import pytest
from geometry_msgs.msg import Point

from robin_core.bead_persistence import (
    get_occupied_slots,
    get_occupied_regions,
    load_completed_beads,
    make_bead_record,
    save_completed_bead,
    save_reserved_slots,
    get_reserved_slots,
    clear_reserved_slots,
)
from robin_core.bead_layout import (
    OccupiedRegion,
    PhysicalBead,
    PlateSlot,
    compute_plate_capacity,
    compute_plate_occupancy_slots,
    slot_overlaps_any,
)


def _make_point(x, y, z):
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


# =========================================================================
# bead_persistence tests
# =========================================================================

class TestLoadCompletedBeads:
    def test_nonexistent_file(self, tmp_path):
        assert load_completed_beads(str(tmp_path / 'missing.json')) == []

    def test_empty_string_path(self):
        assert load_completed_beads('') == []

    def test_valid_file_list_format(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        with open(path, 'w') as f:
            json.dump([{'bead_id': 'B001'}], f)
        result = load_completed_beads(path)
        assert len(result) == 1
        assert result[0]['bead_id'] == 'B001'

    def test_valid_file_dict_format(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        with open(path, 'w') as f:
            json.dump({'beads': [{'bead_id': 'B002'}]}, f)
        result = load_completed_beads(path)
        assert len(result) == 1
        assert result[0]['bead_id'] == 'B002'

    def test_invalid_json(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        with open(path, 'w') as f:
            f.write('not json')
        assert load_completed_beads(path) == []


class TestGetOccupiedSlots:
    def test_empty_file(self, tmp_path):
        assert get_occupied_slots(str(tmp_path / 'missing.json')) == {}

    def test_occupied_slots(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        with open(path, 'w') as f:
            json.dump({'beads': [
                {'plate_id': 'A', 'slot_row': 0, 'slot_col': 0},
                {'plate_id': 'A', 'slot_row': 0, 'slot_col': 1},
                {'plate_id': 'B', 'slot_row': 1, 'slot_col': 0},
            ]}, f)
        result = get_occupied_slots(path)
        assert result == {
            'A': {(0, 0), (0, 1)},
            'B': {(1, 0)},
        }

    def test_skips_invalid_entries(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        with open(path, 'w') as f:
            json.dump({'beads': [
                {'plate_id': 'A', 'slot_row': 0, 'slot_col': 0},
                {'plate_id': '', 'slot_row': 0, 'slot_col': 1},   # empty plate_id
                {'plate_id': 'A', 'slot_row': -1, 'slot_col': 0},  # invalid row
            ]}, f)
        result = get_occupied_slots(path)
        assert result == {'A': {(0, 0)}}


class TestSaveCompletedBead:
    def test_creates_file(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_completed_bead(path, {'bead_id': 'B001', 'plate_id': 'A'})
        assert os.path.isfile(path)
        beads = load_completed_beads(path)
        assert len(beads) == 1
        assert beads[0]['bead_id'] == 'B001'

    def test_appends_to_existing(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_completed_bead(path, {'bead_id': 'B001'})
        save_completed_bead(path, {'bead_id': 'B002'})
        beads = load_completed_beads(path)
        assert len(beads) == 2
        assert beads[0]['bead_id'] == 'B001'
        assert beads[1]['bead_id'] == 'B002'

    def test_creates_directories(self, tmp_path):
        path = str(tmp_path / 'sub' / 'dir' / 'beads.json')
        save_completed_bead(path, {'bead_id': 'B001'})
        assert os.path.isfile(path)


class TestMakeBeadRecord:
    def test_creates_record(self):
        p0 = _make_point(0.0, 0.0, 0.0)
        p1 = _make_point(1.0, 0.0, 0.0)
        record = make_bead_record(
            bead_id='B001', plate_id='plate_A',
            slot_row=0, slot_col=0,
            path=[p0, p1], total_length=1.0,
            local_x_start=0.04, local_x_end=0.14, local_y=0.04,
        )
        assert record['bead_id'] == 'B001'
        assert record['slot_row'] == 0
        assert record['slot_col'] == 0
        assert len(record['path']) == 2
        assert record['path'][0] == {'x': 0.0, 'y': 0.0, 'z': 0.0}
        assert record['path'][1] == {'x': 1.0, 'y': 0.0, 'z': 0.0}
        assert record['local_x_start'] == 0.04
        assert record['local_x_end'] == 0.14
        assert record['local_y'] == 0.04
        assert 'completed_at' in record


# =========================================================================
# Slot-aware capacity tests
# =========================================================================

class TestComputePlateCapacityWithOccupied:
    """Verify compute_plate_capacity subtracts occupied slots."""

    def _capacity(self, occupied=None):
        return compute_plate_capacity(
            plate_length=0.3,
            plate_width=0.3,
            margin_x=0.04,
            margin_y=0.04,
            bead_length_m=0.1,
            spacing_x=0.03,
            spacing_y=0.03,
            staggered=False,
            occupied=occupied,
        )

    def test_full_capacity_no_occupied(self):
        full = self._capacity()
        assert full > 0

    def test_occupied_reduces_capacity(self):
        full = self._capacity()
        reduced = self._capacity(occupied={(0, 0)})
        assert reduced == full - 1

    def test_all_occupied_returns_zero(self):
        slots = compute_plate_occupancy_slots(
            plate_length=0.3, plate_width=0.3,
            margin_x=0.04, margin_y=0.04,
            bead_length_m=0.1,
            spacing_x=0.03, spacing_y=0.03,
            staggered=False,
        )
        all_occupied = {(s.row, s.col) for s in slots}
        assert self._capacity(occupied=all_occupied) == 0


class TestPhysicalBeadSlotFields:
    """Verify PhysicalBead carries slot information."""

    def test_default_slot_values(self):
        bead = PhysicalBead(
            bead_id='B001', plate_id='A',
            path=[_make_point(0, 0, 0), _make_point(1, 0, 0)],
            total_length=1.0,
            weld_speed=0.005,
            wire_feed_speed=5.0,
        )
        assert bead.slot_row == -1
        assert bead.slot_col == -1

    def test_explicit_slot_values(self):
        bead = PhysicalBead(
            bead_id='B001', plate_id='A',
            path=[_make_point(0, 0, 0), _make_point(1, 0, 0)],
            total_length=1.0,
            weld_speed=0.005,
            wire_feed_speed=5.0,
            slot_row=2, slot_col=3,
        )
        assert bead.slot_row == 2
        assert bead.slot_col == 3


class TestRoundTripPersistence:
    """Integration: save beads, load occupied slots, verify capacity reduction."""

    def test_round_trip(self, tmp_path):
        path = str(tmp_path / 'beads.json')

        # Get full capacity
        full = compute_plate_capacity(
            plate_length=0.3, plate_width=0.3,
            margin_x=0.04, margin_y=0.04,
            bead_length_m=0.1,
            spacing_x=0.03, spacing_y=0.03,
            staggered=False,
        )

        # "Complete" one bead at slot (0, 0)
        record = make_bead_record(
            bead_id='B001', plate_id='plate_A',
            slot_row=0, slot_col=0,
            path=[_make_point(0, 0, 0), _make_point(0.1, 0, 0)],
            total_length=0.1,
        )
        save_completed_bead(path, record)

        # Load occupied slots and check capacity
        occupied = get_occupied_slots(path)
        plate_occ = occupied.get('plate_A', set())
        assert (0, 0) in plate_occ

        reduced = compute_plate_capacity(
            plate_length=0.3, plate_width=0.3,
            margin_x=0.04, margin_y=0.04,
            bead_length_m=0.1,
            spacing_x=0.03, spacing_y=0.03,
            staggered=False,
            occupied=plate_occ,
        )
        assert reduced == full - 1


# =========================================================================
# Slot reservation tests
# =========================================================================

class TestSaveReservedSlots:
    def test_reserve_creates_file(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        regions = [(0.04, 0.14, 0.04), (0.04, 0.14, 0.07)]
        added = save_reserved_slots(path, 'plate_A', regions)
        assert added == 2
        beads = load_completed_beads(path)
        assert len(beads) == 2
        assert all(b['reserved'] is True for b in beads)
        assert beads[0]['local_x_start'] == 0.04
        assert beads[0]['local_x_end'] == 0.14
        assert beads[0]['local_y'] == 0.04
        assert beads[1]['local_y'] == 0.07

    def test_reserve_with_slot_labels(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        regions = [(0.04, 0.14, 0.04)]
        labels = [(0, 0)]
        added = save_reserved_slots(path, 'plate_A', regions, slot_labels=labels)
        assert added == 1
        beads = load_completed_beads(path)
        assert beads[0]['slot_row'] == 0
        assert beads[0]['slot_col'] == 0

    def test_reserve_skips_already_occupied(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        # Save a completed bead with local coords
        save_completed_bead(path, {
            'bead_id': 'B001', 'plate_id': 'plate_A',
            'slot_row': 0, 'slot_col': 0,
            'local_x_start': 0.04, 'local_x_end': 0.14, 'local_y': 0.04,
        })
        # Try to reserve the same physical spot
        added = save_reserved_slots(path, 'plate_A', [(0.04, 0.14, 0.04)])
        assert added == 0  # skipped because within 1mm

    def test_reserve_skips_already_reserved(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_reserved_slots(path, 'plate_A', [(0.04, 0.14, 0.04)])
        added = save_reserved_slots(path, 'plate_A', [(0.04, 0.14, 0.04), (0.04, 0.14, 0.07)])
        assert added == 1  # first one already reserved

    def test_reserve_empty_list(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        added = save_reserved_slots(path, 'plate_A', [])
        assert added == 0


class TestGetReservedSlots:
    def test_empty_file(self, tmp_path):
        assert get_reserved_slots(str(tmp_path / 'missing.json')) == {}

    def test_returns_only_reserved(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_completed_bead(path, {'bead_id': 'B001', 'plate_id': 'plate_A',
                                   'slot_row': 0, 'slot_col': 0,
                                   'local_x_start': 0.04, 'local_x_end': 0.14,
                                   'local_y': 0.04})
        save_reserved_slots(path, 'plate_A',
                            [(0.04, 0.14, 0.07), (0.04, 0.14, 0.10)],
                            slot_labels=[(1, 0), (1, 1)])
        reserved = get_reserved_slots(path)
        assert reserved == {'plate_A': {(1, 0), (1, 1)}}

    def test_multiple_plates(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_reserved_slots(path, 'plate_A', [(0.04, 0.14, 0.04)],
                            slot_labels=[(0, 0)])
        save_reserved_slots(path, 'plate_B', [(0.04, 0.14, 0.10)],
                            slot_labels=[(2, 1)])
        reserved = get_reserved_slots(path)
        assert reserved == {'plate_A': {(0, 0)}, 'plate_B': {(2, 1)}}


class TestClearReservedSlots:
    def test_clear_all_for_plate(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_reserved_slots(path, 'plate_A',
                            [(0.04, 0.14, 0.04), (0.04, 0.14, 0.07), (0.04, 0.14, 0.10)],
                            slot_labels=[(0, 0), (0, 1), (1, 0)])
        removed = clear_reserved_slots(path, plate_id='plate_A')
        assert removed == 3
        assert get_reserved_slots(path) == {}

    def test_clear_specific_slots(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_reserved_slots(path, 'plate_A',
                            [(0.04, 0.14, 0.04), (0.04, 0.14, 0.07), (0.04, 0.14, 0.10)],
                            slot_labels=[(0, 0), (0, 1), (1, 0)])
        removed = clear_reserved_slots(path, plate_id='plate_A',
                                       slots={(0, 0), (1, 0)})
        assert removed == 2
        assert get_reserved_slots(path) == {'plate_A': {(0, 1)}}

    def test_clear_all_plates(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_reserved_slots(path, 'plate_A', [(0.04, 0.14, 0.04)],
                            slot_labels=[(0, 0)])
        save_reserved_slots(path, 'plate_B', [(0.04, 0.14, 0.07)],
                            slot_labels=[(1, 0)])
        removed = clear_reserved_slots(path, all_plates=True)
        assert removed == 2
        assert get_reserved_slots(path) == {}

    def test_clear_does_not_remove_completed_beads(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_completed_bead(path, {'bead_id': 'B001', 'plate_id': 'plate_A',
                                   'slot_row': 0, 'slot_col': 0,
                                   'local_x_start': 0.04, 'local_x_end': 0.14,
                                   'local_y': 0.04})
        save_reserved_slots(path, 'plate_A', [(0.04, 0.14, 0.07)],
                            slot_labels=[(0, 1)])
        removed = clear_reserved_slots(path, all_plates=True)
        assert removed == 1
        # Completed bead must still be there
        beads = load_completed_beads(path)
        assert len(beads) == 1
        assert beads[0]['bead_id'] == 'B001'

    def test_clear_by_bead_ids(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_reserved_slots(path, 'plate_A',
                            [(0.04, 0.14, 0.04), (0.04, 0.14, 0.07)])
        beads = load_completed_beads(path)
        bid = beads[0]['bead_id']
        removed = clear_reserved_slots(path, bead_ids={bid})
        assert removed == 1
        remaining = load_completed_beads(path)
        assert len(remaining) == 1

    def test_clear_empty_file(self, tmp_path):
        path = str(tmp_path / 'missing.json')
        removed = clear_reserved_slots(path, plate_id='plate_A')
        assert removed == 0


class TestReservationIntegration:
    """Reserved regions should be returned by get_occupied_regions
    and thus reduce plate capacity via geometric overlap checking."""

    def test_reserved_regions_reduce_capacity(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        full = compute_plate_capacity(
            plate_length=0.3, plate_width=0.3,
            margin_x=0.04, margin_y=0.04,
            bead_length_m=0.1,
            spacing_x=0.03, spacing_y=0.03,
            staggered=False,
        )
        # Reserve two physical regions that match rows 0 and 1
        save_reserved_slots(path, 'plate_A',
                            [(0.04, 0.14, 0.04), (0.04, 0.14, 0.07)])
        regions = get_occupied_regions(path)
        reduced = compute_plate_capacity(
            plate_length=0.3, plate_width=0.3,
            margin_x=0.04, margin_y=0.04,
            bead_length_m=0.1,
            spacing_x=0.03, spacing_y=0.03,
            staggered=False,
            occupied_regions=regions.get('plate_A'),
        )
        assert reduced == full - 2

    def test_clear_restores_capacity(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        full = compute_plate_capacity(
            plate_length=0.3, plate_width=0.3,
            margin_x=0.04, margin_y=0.04,
            bead_length_m=0.1,
            spacing_x=0.03, spacing_y=0.03,
            staggered=False,
        )
        save_reserved_slots(path, 'plate_A', [(0.04, 0.14, 0.04)])
        clear_reserved_slots(path, plate_id='plate_A')
        regions = get_occupied_regions(path)
        restored = compute_plate_capacity(
            plate_length=0.3, plate_width=0.3,
            margin_x=0.04, margin_y=0.04,
            bead_length_m=0.1,
            spacing_x=0.03, spacing_y=0.03,
            staggered=False,
            occupied_regions=regions.get('plate_A'),
        )
        assert restored == full


class TestPlannerSkipsReservedSlots:
    """Verify compute_physical_beads places beads only on unreserved slots."""

    def test_beads_skip_reserved_regions(self, tmp_path):
        """Reserve row-0 physical region; verify first bead lands on row 1."""
        from robin_interfaces.msg import ExperimentBead, PlateLayout, WeldPlate

        path = str(tmp_path / 'beads.json')
        # Reserve physical coords matching row 0 with spacing_y=0.03, margin=0.04
        save_reserved_slots(path, 'plate_A', [(0.04, 0.14, 0.04)],
                            slot_labels=[(0, 0)])

        plate = WeldPlate()
        plate.plate_id = 'plate_A'
        plate.origin.x = 0.0
        plate.origin.y = 0.0
        plate.origin.z = 0.0
        plate.width = 0.3
        plate.length = 0.3
        plate.orientation = 0.0
        plate.surface_z = 0.0
        plate.is_calibrated = True
        plate.corner_id = 'front_left'
        plate.margin_x = 0.04
        plate.margin_y = 0.04

        bead = ExperimentBead()
        bead.bead_id = 'B001'
        bead.plate_id = 'plate_A'
        bead.weld_speed = 0.005
        bead.wire_feed_speed = 5.0

        layout = PlateLayout()
        layout.spacing_x = 0.03
        layout.spacing_y = 0.03
        layout.margin_x = 0.04
        layout.margin_y = 0.04

        from robin_core.bead_layout import compute_physical_beads
        result = compute_physical_beads(
            [bead], [plate], layout,
            default_bead_pitch=0.03,
            default_margin_x=0.04,
            default_margin_y=0.04,
            bead_length_m=0.1,
            occupied_regions=get_occupied_regions(path),
        )
        assert len(result) == 1
        # Row 0 was reserved, so bead should land on row 1
        assert result[0].slot_row == 1
        assert result[0].slot_col == 0
        # Verify local coords populated
        assert result[0].local_y == pytest.approx(0.07, abs=1e-6)


class TestGetOccupiedRegions:
    """Verify get_occupied_regions reads physical coordinates."""

    def test_empty_file(self, tmp_path):
        assert get_occupied_regions(str(tmp_path / 'missing.json')) == {}

    def test_returns_regions_with_local_coords(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        save_completed_bead(path, {
            'bead_id': 'B001', 'plate_id': 'plate_A',
            'slot_row': 0, 'slot_col': 0,
            'local_x_start': 0.04, 'local_x_end': 0.14, 'local_y': 0.04,
        })
        regions = get_occupied_regions(path)
        assert 'plate_A' in regions
        assert len(regions['plate_A']) == 1
        r = regions['plate_A'][0]
        assert r.x_start == pytest.approx(0.04)
        assert r.x_end == pytest.approx(0.14)
        assert r.y == pytest.approx(0.04)

    def test_skips_records_without_local_coords(self, tmp_path):
        path = str(tmp_path / 'beads.json')
        # Legacy record without local coords
        save_completed_bead(path, {
            'bead_id': 'B001', 'plate_id': 'plate_A',
            'slot_row': 0, 'slot_col': 0,
        })
        regions = get_occupied_regions(path)
        assert regions == {}


class TestSlotOverlapsAny:
    """Verify geometric overlap checking."""

    def test_no_regions_no_overlap(self):
        slot = PlateSlot(0, 0, 0.04, 0.14, 0.04)
        assert not slot_overlaps_any(slot, [], min_y_clearance=0.03)

    def test_exact_same_position_overlaps(self):
        slot = PlateSlot(0, 0, 0.04, 0.14, 0.04)
        regions = [OccupiedRegion(0.04, 0.14, 0.04)]
        assert slot_overlaps_any(slot, regions, min_y_clearance=0.03)

    def test_adjacent_row_no_overlap(self):
        """Slot exactly spacing_y away should NOT overlap."""
        slot = PlateSlot(1, 0, 0.04, 0.14, 0.07)
        regions = [OccupiedRegion(0.04, 0.14, 0.04)]
        assert not slot_overlaps_any(slot, regions, min_y_clearance=0.03)

    def test_close_y_overlaps(self):
        """Slot closer than spacing_y should overlap."""
        slot = PlateSlot(0, 0, 0.04, 0.14, 0.05)
        regions = [OccupiedRegion(0.04, 0.14, 0.04)]
        assert slot_overlaps_any(slot, regions, min_y_clearance=0.03)

    def test_no_x_overlap_no_conflict(self):
        """Different X ranges → no conflict even if Y is close."""
        slot = PlateSlot(0, 1, 0.20, 0.30, 0.04)
        regions = [OccupiedRegion(0.04, 0.14, 0.04)]
        assert not slot_overlaps_any(slot, regions, min_y_clearance=0.03)


class TestCrossSpacingOverlap:
    """Core test: beads from different spacing experiments must not collide."""

    def test_different_spacing_prevents_overlap(self, tmp_path):
        """Exp-1 spacing_y=0.03 places beads at y=0.04, 0.07, 0.10.
        Exp-2 spacing_y=0.05 would place at y=0.04, 0.09.
        y=0.09 is only 0.01 from y=0.10 — overlap must be detected."""
        path = str(tmp_path / 'beads.json')

        # Exp-1 completed beads (spacing_y=0.03)
        for i, y in enumerate([0.04, 0.07, 0.10]):
            save_completed_bead(path, {
                'bead_id': f'B{i:03d}', 'plate_id': 'plate_A',
                'slot_row': i, 'slot_col': 0,
                'local_x_start': 0.04, 'local_x_end': 0.14, 'local_y': y,
            })

        regions = get_occupied_regions(path)
        plate_regions = regions.get('plate_A', [])
        assert len(plate_regions) == 3

        # Exp-2 would compute slots with spacing_y=0.05
        slots = compute_plate_occupancy_slots(
            plate_length=0.3, plate_width=0.3,
            margin_x=0.04, margin_y=0.04,
            bead_length_m=0.1,
            spacing_x=0.03, spacing_y=0.05,
            staggered=False,
        )
        # Filter available slots
        available = [s for s in slots
                     if not slot_overlaps_any(s, plate_regions, 0.05)]

        # Every available slot must be at least spacing_y from all existing
        for s in available:
            for r in plate_regions:
                assert abs(s.local_y - r.y) >= 0.05 - 1e-9
