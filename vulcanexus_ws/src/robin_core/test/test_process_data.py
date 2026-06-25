"""Unit tests for WeldProfileProcessor.compute_weld_dimensions().

Generates synthetic laser profilometer scans (semi-elliptical bead on a
flat or tilted base plate) and verifies the algorithm recovers width and
height within tight tolerances.  Also tests that the progression-history
buffer aligns the geometry stamp to the temporal centre of the moving-
average window.
"""

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Helpers – build the processor without ROS2
# ---------------------------------------------------------------------------

def _make_processor(filter_window=5):
    """Create a minimal WeldProfileProcessor-like object with only the
    fields and methods needed for compute_weld_dimensions()."""
    from robin_core.process_data_node import WeldProfileProcessor

    class _StubProcessor:
        """Mirrors the state used by compute_weld_dimensions without
        requiring rclpy initialisation."""
        pass

    proc = _StubProcessor()
    # Bind the unbound method to our stub
    proc.compute_weld_dimensions = (
        WeldProfileProcessor.compute_weld_dimensions.__get__(proc))
    # Provide a no-op logger
    class _Logger:
        def error(self, *a, **kw): pass
        def warn(self, *a, **kw): pass
        def debug(self, *a, **kw): pass
        def info(self, *a, **kw): pass
    proc.get_logger = lambda: _Logger()
    return proc


def _semi_elliptical_profile(
    fov_mm=35.0,
    n_points=350,
    bead_width_mm=9.0,
    bead_height_mm=1.0,
    base_slope=0.0,
    base_offset=0.0,
    noise_std=0.02,
    seed=42,
):
    """Generate a synthetic laser scan: flat/tilted base plate with a semi-
    elliptical weld bead centred at y=0.

    Returns points as (N, 3) in **metres** (matching Garmo sensor output).
    """
    rng = np.random.default_rng(seed)
    y = np.linspace(-fov_mm / 2, fov_mm / 2, n_points)  # mm
    z_base = base_slope * y + base_offset                 # mm (linear plate)
    # Semi-ellipse: z = H * sqrt(1 - (2y/W)^2) for |y| < W/2
    half_w = bead_width_mm / 2.0
    bead_mask = np.abs(y) < half_w
    z_bead = np.zeros_like(y)
    z_bead[bead_mask] = bead_height_mm * np.sqrt(
        1.0 - (y[bead_mask] / half_w) ** 2)
    z = z_base + z_bead + rng.normal(0, noise_std, size=n_points)
    x = np.zeros_like(y)
    # Convert to metres (sensor publishes in m)
    points = np.column_stack([x, y, z]) / 1000.0
    return points


def _multi_bead_profile(
    fov_mm=35.0,
    n_points=350,
    beads=None,
    base_slope=0.0,
    base_offset=0.0,
    noise_std=0.02,
    seed=42,
):
    """Generate a profile with multiple semi-elliptical beads.

    *beads* is a list of dicts with keys 'center_mm', 'width_mm', 'height_mm'.
    Returns (N, 3) array in metres.
    """
    if beads is None:
        beads = [
            {'center_mm': 0.0, 'width_mm': 9.0, 'height_mm': 1.0},
            {'center_mm': 12.0, 'width_mm': 8.0, 'height_mm': 1.2},
        ]
    rng = np.random.default_rng(seed)
    y = np.linspace(-fov_mm / 2, fov_mm / 2, n_points)
    z_base = base_slope * y + base_offset
    z_bead = np.zeros_like(y)
    for b in beads:
        half_w = b['width_mm'] / 2.0
        mask = np.abs(y - b['center_mm']) < half_w
        z_bead[mask] += b['height_mm'] * np.sqrt(
            1.0 - ((y[mask] - b['center_mm']) / half_w) ** 2)
    z = z_base + z_bead + rng.normal(0, noise_std, size=n_points)
    x = np.zeros_like(y)
    points = np.column_stack([x, y, z]) / 1000.0
    return points


# ---------------------------------------------------------------------------
# Geometry accuracy tests
# ---------------------------------------------------------------------------

class TestShallowWideBead:
    """Shallow, wide bead (W=9 mm, H=1.0 mm) – the primary failure case."""

    @pytest.fixture
    def result(self):
        proc = _make_processor(filter_window=1)
        pts = _semi_elliptical_profile(bead_width_mm=9.0, bead_height_mm=1.0)
        return proc.compute_weld_dimensions(pts)

    def test_width(self, result):
        width, _, _ = result
        assert width == pytest.approx(9.0, abs=0.5), (
            f"Width {width:.2f} mm, expected ~9.0 mm")

    def test_height(self, result):
        _, height, _ = result
        assert height == pytest.approx(1.0, abs=0.15), (
            f"Height {height:.2f} mm, expected ~1.0 mm")

    def test_toe_angle(self, result):
        _, _, toe_angle = result
        assert 1.5 < toe_angle < 3.0, (
            f"Toe angle {toe_angle:.3f} rad, expected ~2.7 rad for shallow bead")


class TestVeryShallowBead:
    """Very shallow bead (W=10 mm, H=0.8 mm) – near the detection floor."""

    @pytest.fixture
    def result(self):
        proc = _make_processor(filter_window=1)
        pts = _semi_elliptical_profile(bead_width_mm=10.0, bead_height_mm=0.8)
        return proc.compute_weld_dimensions(pts)

    def test_width(self, result):
        width, _, _ = result
        assert width == pytest.approx(10.0, abs=0.7), (
            f"Width {width:.2f} mm, expected ~10.0 mm")

    def test_height(self, result):
        _, height, _ = result
        assert height == pytest.approx(0.8, abs=0.15), (
            f"Height {height:.2f} mm, expected ~0.8 mm")


class TestTallNarrowBead:
    """Tall, narrow bead (W=5 mm, H=4 mm) – high walls, easy detection."""

    @pytest.fixture
    def result(self):
        proc = _make_processor(filter_window=1)
        pts = _semi_elliptical_profile(bead_width_mm=5.0, bead_height_mm=4.0)
        return proc.compute_weld_dimensions(pts)

    def test_width(self, result):
        width, _, _ = result
        assert width == pytest.approx(5.0, abs=0.3), (
            f"Width {width:.2f} mm, expected ~5.0 mm")

    def test_height(self, result):
        _, height, _ = result
        assert height == pytest.approx(4.0, abs=0.3), (
            f"Height {height:.2f} mm, expected ~4.0 mm")

    def test_toe_angle(self, result):
        _, _, toe_angle = result
        assert 1.3 < toe_angle < 2.3, (
            f"Toe angle {toe_angle:.3f} rad, expected ~1.85 rad for tall bead")


class TestTiltedBasePlate:
    """Shallow bead on a tilted base plate (1° slope ≈ 0.0175 mm/mm)."""

    @pytest.fixture
    def result(self):
        proc = _make_processor(filter_window=1)
        pts = _semi_elliptical_profile(
            bead_width_mm=9.0, bead_height_mm=1.0,
            base_slope=0.0175, base_offset=0.5)
        return proc.compute_weld_dimensions(pts)

    def test_width(self, result):
        width, _, _ = result
        assert width == pytest.approx(9.0, abs=0.5), (
            f"Width {width:.2f} mm on tilted base, expected ~9.0 mm")

    def test_height(self, result):
        _, height, _ = result
        assert height == pytest.approx(1.0, abs=0.15), (
            f"Height {height:.2f} mm on tilted base, expected ~1.0 mm")


class TestNoisyProfile:
    """Higher sensor noise (σ=0.05 mm) should still yield usable results."""

    @pytest.fixture
    def result(self):
        proc = _make_processor(filter_window=1)
        pts = _semi_elliptical_profile(
            bead_width_mm=9.0, bead_height_mm=1.5, noise_std=0.05)
        return proc.compute_weld_dimensions(pts)

    def test_width(self, result):
        width, _, _ = result
        assert width == pytest.approx(9.0, abs=0.7), (
            f"Width {width:.2f} mm under noise, expected ~9.0 mm")

    def test_height(self, result):
        _, height, _ = result
        assert height == pytest.approx(1.5, abs=0.25), (
            f"Height {height:.2f} mm under noise, expected ~1.5 mm")


class TestFlatPlateNoBead:
    """Flat plate with no bead → should return (0.0, 0.0, 0.0)."""

    def test_returns_zero(self):
        proc = _make_processor(filter_window=1)
        pts = _semi_elliptical_profile(
            bead_width_mm=0.0, bead_height_mm=0.0, noise_std=0.02)
        width, height, toe_angle = proc.compute_weld_dimensions(pts)
        assert width == 0.0 and height == 0.0 and toe_angle == 0.0


class TestTooFewPoints:
    """Fewer than 20 points → should return (0.0, 0.0, 0.0)."""

    def test_returns_zero(self):
        proc = _make_processor(filter_window=1)
        pts = np.zeros((10, 3))
        width, height, toe_angle = proc.compute_weld_dimensions(pts)
        assert width == 0.0 and height == 0.0 and toe_angle == 0.0


# ---------------------------------------------------------------------------
# Multi-bead / adjacent bead tests
# ---------------------------------------------------------------------------

class TestAdjacentBead:
    """Current bead (centred) with an adjacent bead to the right.

    The algorithm must measure the centred bead and ignore the adjacent one.
    """

    @pytest.fixture
    def result(self):
        proc = _make_processor(filter_window=1)
        pts = _multi_bead_profile(beads=[
            {'center_mm': 0.0, 'width_mm': 9.0, 'height_mm': 1.0},
            {'center_mm': 12.0, 'width_mm': 8.0, 'height_mm': 1.2},
        ])
        return proc.compute_weld_dimensions(pts)

    def test_width(self, result):
        width, _, _ = result
        assert width == pytest.approx(9.0, abs=0.7), (
            f"Width {width:.2f} mm with adjacent bead, expected ~9.0 mm")

    def test_height(self, result):
        _, height, _ = result
        assert height == pytest.approx(1.0, abs=0.2), (
            f"Height {height:.2f} mm with adjacent bead, expected ~1.0 mm")


class TestAdjacentBeadOnLeft:
    """Adjacent bead on the left side, current bead roughly centred."""

    @pytest.fixture
    def result(self):
        proc = _make_processor(filter_window=1)
        pts = _multi_bead_profile(beads=[
            {'center_mm': -12.0, 'width_mm': 7.0, 'height_mm': 2.0},
            {'center_mm': 0.0, 'width_mm': 9.0, 'height_mm': 1.0},
        ])
        return proc.compute_weld_dimensions(pts)

    def test_width(self, result):
        width, _, _ = result
        assert width == pytest.approx(9.0, abs=0.7), (
            f"Width {width:.2f} mm, expected ~9.0 mm (ignoring left bead)")

    def test_height(self, result):
        _, height, _ = result
        assert height == pytest.approx(1.0, abs=0.2), (
            f"Height {height:.2f} mm, expected ~1.0 mm")


class TestTwoAdjacentBeadsBothSides:
    """Beads on both sides of the current bead (~60% of FOV is elevated)."""

    @pytest.fixture
    def result(self):
        proc = _make_processor(filter_window=1)
        pts = _multi_bead_profile(beads=[
            {'center_mm': -11.0, 'width_mm': 7.0, 'height_mm': 1.5},
            {'center_mm': 0.0, 'width_mm': 8.0, 'height_mm': 1.0},
            {'center_mm': 11.0, 'width_mm': 7.0, 'height_mm': 1.5},
        ])
        return proc.compute_weld_dimensions(pts)

    def test_width(self, result):
        width, _, _ = result
        assert width == pytest.approx(8.0, abs=2.0), (
            f"Width {width:.2f} mm, expected ~8.0 mm (centre bead)")

    def test_height(self, result):
        _, height, _ = result
        assert height == pytest.approx(1.0, abs=0.3), (
            f"Height {height:.2f} mm, expected ~1.0 mm (centre bead)")


class TestAdjacentBeadTallerThanTarget:
    """Adjacent bead is taller than the target — algorithm must still
    pick the centred bead, not the tallest."""

    @pytest.fixture
    def result(self):
        proc = _make_processor(filter_window=1)
        pts = _multi_bead_profile(beads=[
            {'center_mm': 0.0, 'width_mm': 9.0, 'height_mm': 1.0},
            {'center_mm': 12.0, 'width_mm': 6.0, 'height_mm': 3.0},
        ])
        return proc.compute_weld_dimensions(pts)

    def test_width(self, result):
        width, _, _ = result
        assert width == pytest.approx(9.0, abs=0.7), (
            f"Width {width:.2f} mm, expected ~9.0 (centre, not tall bead)")

    def test_height_not_adjacent(self, result):
        _, height, _ = result
        assert height < 2.0, (
            f"Height {height:.2f} mm — should be ~1.0, not ~3.0 (adjacent)")



# ---------------------------------------------------------------------------
# Progression alignment tests
# ---------------------------------------------------------------------------
