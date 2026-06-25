#!/usr/bin/env python3
"""
Unit tests for progression calculation logic.

These tests verify the mathematical correctness of the progression
calculation without requiring ROS2 infrastructure.
"""

import math

import numpy as np
import pytest
from geometry_msgs.msg import Point


class ProgressionCalculator:
    """
    Extracted progression calculation logic for testing.
    
    This mirrors the logic in WeldDataNode._calculate_progression()
    which projects the TCP position onto a polyline path and returns
    normalised arc-length progression.
    """
    
    @staticmethod
    def calculate(tcp_pos: Point, path: list, total_length: float) -> float:
        """Calculate progression (0.0-1.0) along a polyline path.
        
        Args:
            tcp_pos: Current TCP position
            path: Ordered list of geometry_msgs/Point waypoints
            total_length: Pre-computed total path length
            
        Returns:
            Progression value clamped to [0.0, 1.0]
        """
        if total_length <= 0 or len(path) < 2:
            return 0.0

        tcp = np.array([tcp_pos.x, tcp_pos.y, tcp_pos.z])
        pts = np.array([[p.x, p.y, p.z] for p in path])
        segs = np.diff(pts, axis=0)
        seg_lens_sq = np.sum(segs ** 2, axis=1)

        best_dist_sq = float('inf')
        best_arc = 0.0
        cumulative = 0.0

        for i in range(len(segs)):
            if seg_lens_sq[i] < 1e-12:
                continue

            t = float(np.clip(np.dot(tcp - pts[i], segs[i]) / seg_lens_sq[i], 0.0, 1.0))
            proj = pts[i] + t * segs[i]
            dist_sq = float(np.sum((tcp - proj) ** 2))

            seg_len = float(np.sqrt(seg_lens_sq[i]))
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_arc = cumulative + t * seg_len

            cumulative += seg_len

        return max(0.0, min(1.0, best_arc / total_length))

    @staticmethod
    def calculate_two_point(tcp_pos: Point, bead_start: Point,
                            bead_end: Point, bead_length: float) -> float:
        """Convenience wrapper for straight-line (2-point) beads."""
        return ProgressionCalculator.calculate(
            tcp_pos, [bead_start, bead_end], bead_length)


def make_point(x: float, y: float, z: float) -> Point:
    """Helper to create a Point message."""
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p


class TestProgressionCalculation:
    """Test suite for progression calculation."""
    
    def test_zero_length_bead_returns_zero(self):
        """Progression should be 0 for zero-length bead."""
        tcp = make_point(0.5, 0.0, 0.0)
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(0.0, 0.0, 0.0)
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 0.0)
        assert result == 0.0
    
    def test_at_start_returns_zero(self):
        """TCP at bead start should return progression = 0."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(0.0, 0.0, 0.0)  # At start
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 1.0)
        assert result == pytest.approx(0.0)
    
    def test_at_end_returns_one(self):
        """TCP at bead end should return progression = 1."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(1.0, 0.0, 0.0)  # At end
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 1.0)
        assert result == pytest.approx(1.0)
    
    def test_midpoint_returns_half(self):
        """TCP at bead midpoint should return progression = 0.5."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(0.5, 0.0, 0.0)  # At midpoint
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 1.0)
        assert result == pytest.approx(0.5)
    
    def test_quarter_point(self):
        """TCP at 25% along bead should return progression = 0.25."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(0.25, 0.0, 0.0)
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 1.0)
        assert result == pytest.approx(0.25)
    
    def test_clamps_before_start(self):
        """TCP before bead start should clamp to 0."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(-0.5, 0.0, 0.0)  # Before start
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 1.0)
        assert result == 0.0
    
    def test_clamps_after_end(self):
        """TCP after bead end should clamp to 1."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(1.5, 0.0, 0.0)  # After end
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 1.0)
        assert result == 1.0
    
    def test_y_axis_bead(self):
        """Progression along Y axis bead."""
        start = make_point(0.5, 0.0, 0.1)
        end = make_point(0.5, 0.3, 0.1)
        tcp = make_point(0.5, 0.15, 0.1)  # Midpoint
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 0.3)
        assert result == pytest.approx(0.5)
    
    def test_z_axis_bead(self):
        """Progression along Z axis (vertical) bead."""
        start = make_point(0.5, 0.5, 0.0)
        end = make_point(0.5, 0.5, 0.2)
        tcp = make_point(0.5, 0.5, 0.1)  # Midpoint
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 0.2)
        assert result == pytest.approx(0.5)
    
    def test_diagonal_bead_3d(self):
        """Progression along a 3D diagonal bead."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 1.0, 1.0)
        length = math.sqrt(3)
        
        # TCP at midpoint (0.5, 0.5, 0.5)
        tcp = make_point(0.5, 0.5, 0.5)
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, length)
        assert result == pytest.approx(0.5)
        
        # TCP at 75% (0.75, 0.75, 0.75)
        tcp = make_point(0.75, 0.75, 0.75)
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, length)
        assert result == pytest.approx(0.75)
    
    def test_tcp_offset_perpendicular_to_bead(self):
        """TCP offset perpendicular to bead should project correctly.
        
        If TCP is at (0.5, 0.1, 0.0) and bead is along X axis,
        progression should still be 0.5 (projection onto bead line).
        """
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(0.5, 0.1, 0.0)  # Offset in Y but at X=0.5
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 1.0)
        assert result == pytest.approx(0.5)
    
    def test_realistic_weld_scenario(self):
        """Test with realistic weld bead dimensions (300mm bead)."""
        # 300mm bead along Y axis at z=0.1m (workpiece height)
        start = make_point(0.5, 0.0, 0.1)
        end = make_point(0.5, 0.3, 0.1)
        length = 0.3  # 300mm
        
        # Test progression at 100mm into the bead
        tcp = make_point(0.5, 0.1, 0.1)
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, length)
        assert result == pytest.approx(0.333, rel=0.01)
        
        # Test progression at 200mm into the bead
        tcp = make_point(0.5, 0.2, 0.1)
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, length)
        assert result == pytest.approx(0.667, rel=0.01)
    
    def test_small_bead_precision(self):
        """Test precision with small bead (10mm)."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(0.01, 0.0, 0.0)  # 10mm bead
        length = 0.01
        
        # At 5mm (midpoint)
        tcp = make_point(0.005, 0.0, 0.0)
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, length)
        assert result == pytest.approx(0.5)
        
        # At 2.5mm (25%)
        tcp = make_point(0.0025, 0.0, 0.0)
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, length)
        assert result == pytest.approx(0.25)


class TestProgressionEdgeCases:
    """Edge case tests for progression calculation."""
    
    def test_negative_length_returns_zero(self):
        """Negative bead length should return 0 (invalid state)."""
        tcp = make_point(0.5, 0.0, 0.0)
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, -1.0)
        assert result == 0.0
    
    def test_very_small_bead(self):
        """Very small bead (1mm) should still calculate correctly."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(0.001, 0.0, 0.0)
        tcp = make_point(0.0005, 0.0, 0.0)
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 0.001)
        assert result == pytest.approx(0.5)
    
    def test_large_perpendicular_offset(self):
        """Large perpendicular offset should still project correctly."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        # TCP is 1 meter away in Y but at X=0.5
        tcp = make_point(0.5, 1.0, 0.0)
        
        result = ProgressionCalculator.calculate_two_point(tcp, start, end, 1.0)
        assert result == pytest.approx(0.5)


class TestPolylineProgression:
    """Tests for multi-segment polyline path progression."""

    def test_two_segment_l_shape_midpoint_first_segment(self):
        """TCP at midpoint of first segment of an L-shaped path."""
        p0 = make_point(0.0, 0.0, 0.0)
        p1 = make_point(1.0, 0.0, 0.0)
        p2 = make_point(1.0, 1.0, 0.0)
        path = [p0, p1, p2]
        total_length = 2.0  # 1.0 + 1.0

        tcp = make_point(0.5, 0.0, 0.0)  # Mid of first segment
        result = ProgressionCalculator.calculate(tcp, path, total_length)
        assert result == pytest.approx(0.25)

    def test_two_segment_l_shape_at_corner(self):
        """TCP at the corner point of an L-shaped path."""
        p0 = make_point(0.0, 0.0, 0.0)
        p1 = make_point(1.0, 0.0, 0.0)
        p2 = make_point(1.0, 1.0, 0.0)
        path = [p0, p1, p2]
        total_length = 2.0

        tcp = make_point(1.0, 0.0, 0.0)
        result = ProgressionCalculator.calculate(tcp, path, total_length)
        assert result == pytest.approx(0.5)

    def test_two_segment_l_shape_midpoint_second_segment(self):
        """TCP at midpoint of second segment of an L-shaped path."""
        p0 = make_point(0.0, 0.0, 0.0)
        p1 = make_point(1.0, 0.0, 0.0)
        p2 = make_point(1.0, 1.0, 0.0)
        path = [p0, p1, p2]
        total_length = 2.0

        tcp = make_point(1.0, 0.5, 0.0)  # Mid of second segment
        result = ProgressionCalculator.calculate(tcp, path, total_length)
        assert result == pytest.approx(0.75)

    def test_three_segment_path_end(self):
        """TCP at the end of a three-segment path."""
        p0 = make_point(0.0, 0.0, 0.0)
        p1 = make_point(1.0, 0.0, 0.0)
        p2 = make_point(1.0, 1.0, 0.0)
        p3 = make_point(2.0, 1.0, 0.0)
        path = [p0, p1, p2, p3]
        total_length = 3.0

        tcp = make_point(2.0, 1.0, 0.0)
        result = ProgressionCalculator.calculate(tcp, path, total_length)
        assert result == pytest.approx(1.0)

    def test_single_point_path(self):
        """Single point path should return 0."""
        p0 = make_point(0.0, 0.0, 0.0)
        result = ProgressionCalculator.calculate(
            make_point(0.5, 0.0, 0.0), [p0], 0.0)
        assert result == 0.0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
