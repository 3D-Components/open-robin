#!/usr/bin/env python3
"""
Unit tests for progression calculation logic.

These tests verify the mathematical correctness of the progression
calculation without requiring ROS2 infrastructure.
"""

import pytest
from geometry_msgs.msg import Point


class ProgressionCalculator:
    """
    Extracted progression calculation logic for testing.
    
    This mirrors the logic in WeldDataNode._calculate_progression()
    """
    
    @staticmethod
    def calculate(tcp_pos: Point, bead_start: Point, bead_end: Point, 
                  bead_length: float) -> float:
        """Calculate progression (0.0-1.0) along the bead.
        
        Args:
            tcp_pos: Current TCP position
            bead_start: Bead start point
            bead_end: Bead end point
            bead_length: Pre-computed bead length
            
        Returns:
            Progression value clamped to [0.0, 1.0]
        """
        if bead_length <= 0:
            return 0.0
        
        # Vector from bead start to end
        bead_vec_x = bead_end.x - bead_start.x
        bead_vec_y = bead_end.y - bead_start.y
        bead_vec_z = bead_end.z - bead_start.z
        
        # Vector from bead start to TCP
        tcp_vec_x = tcp_pos.x - bead_start.x
        tcp_vec_y = tcp_pos.y - bead_start.y
        tcp_vec_z = tcp_pos.z - bead_start.z
        
        # Project TCP onto bead line
        dot_product = (tcp_vec_x * bead_vec_x + 
                       tcp_vec_y * bead_vec_y + 
                       tcp_vec_z * bead_vec_z)
        bead_length_sq = bead_length * bead_length
        
        progression = dot_product / bead_length_sq
        
        return max(0.0, min(1.0, progression))


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
        
        result = ProgressionCalculator.calculate(tcp, start, end, 0.0)
        assert result == 0.0
    
    def test_at_start_returns_zero(self):
        """TCP at bead start should return progression = 0."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(0.0, 0.0, 0.0)  # At start
        
        result = ProgressionCalculator.calculate(tcp, start, end, 1.0)
        assert result == pytest.approx(0.0)
    
    def test_at_end_returns_one(self):
        """TCP at bead end should return progression = 1."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(1.0, 0.0, 0.0)  # At end
        
        result = ProgressionCalculator.calculate(tcp, start, end, 1.0)
        assert result == pytest.approx(1.0)
    
    def test_midpoint_returns_half(self):
        """TCP at bead midpoint should return progression = 0.5."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(0.5, 0.0, 0.0)  # At midpoint
        
        result = ProgressionCalculator.calculate(tcp, start, end, 1.0)
        assert result == pytest.approx(0.5)
    
    def test_quarter_point(self):
        """TCP at 25% along bead should return progression = 0.25."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(0.25, 0.0, 0.0)
        
        result = ProgressionCalculator.calculate(tcp, start, end, 1.0)
        assert result == pytest.approx(0.25)
    
    def test_clamps_before_start(self):
        """TCP before bead start should clamp to 0."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(-0.5, 0.0, 0.0)  # Before start
        
        result = ProgressionCalculator.calculate(tcp, start, end, 1.0)
        assert result == 0.0
    
    def test_clamps_after_end(self):
        """TCP after bead end should clamp to 1."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(1.5, 0.0, 0.0)  # After end
        
        result = ProgressionCalculator.calculate(tcp, start, end, 1.0)
        assert result == 1.0
    
    def test_y_axis_bead(self):
        """Progression along Y axis bead."""
        start = make_point(0.5, 0.0, 0.1)
        end = make_point(0.5, 0.3, 0.1)
        tcp = make_point(0.5, 0.15, 0.1)  # Midpoint
        
        result = ProgressionCalculator.calculate(tcp, start, end, 0.3)
        assert result == pytest.approx(0.5)
    
    def test_z_axis_bead(self):
        """Progression along Z axis (vertical) bead."""
        start = make_point(0.5, 0.5, 0.0)
        end = make_point(0.5, 0.5, 0.2)
        tcp = make_point(0.5, 0.5, 0.1)  # Midpoint
        
        result = ProgressionCalculator.calculate(tcp, start, end, 0.2)
        assert result == pytest.approx(0.5)
    
    def test_diagonal_bead_3d(self):
        """Progression along a 3D diagonal bead."""
        # Bead from (0,0,0) to (1,1,1), length = sqrt(3)
        import math
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 1.0, 1.0)
        length = math.sqrt(3)
        
        # TCP at midpoint (0.5, 0.5, 0.5)
        tcp = make_point(0.5, 0.5, 0.5)
        result = ProgressionCalculator.calculate(tcp, start, end, length)
        assert result == pytest.approx(0.5)
        
        # TCP at 75% (0.75, 0.75, 0.75)
        tcp = make_point(0.75, 0.75, 0.75)
        result = ProgressionCalculator.calculate(tcp, start, end, length)
        assert result == pytest.approx(0.75)
    
    def test_tcp_offset_perpendicular_to_bead(self):
        """TCP offset perpendicular to bead should project correctly.
        
        If TCP is at (0.5, 0.1, 0.0) and bead is along X axis,
        progression should still be 0.5 (projection onto bead line).
        """
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        tcp = make_point(0.5, 0.1, 0.0)  # Offset in Y but at X=0.5
        
        result = ProgressionCalculator.calculate(tcp, start, end, 1.0)
        assert result == pytest.approx(0.5)
    
    def test_realistic_weld_scenario(self):
        """Test with realistic weld bead dimensions (300mm bead)."""
        # 300mm bead along Y axis at z=0.1m (workpiece height)
        start = make_point(0.5, 0.0, 0.1)
        end = make_point(0.5, 0.3, 0.1)
        length = 0.3  # 300mm
        
        # Test progression at 100mm into the bead
        tcp = make_point(0.5, 0.1, 0.1)
        result = ProgressionCalculator.calculate(tcp, start, end, length)
        assert result == pytest.approx(0.333, rel=0.01)
        
        # Test progression at 200mm into the bead
        tcp = make_point(0.5, 0.2, 0.1)
        result = ProgressionCalculator.calculate(tcp, start, end, length)
        assert result == pytest.approx(0.667, rel=0.01)
    
    def test_small_bead_precision(self):
        """Test precision with small bead (10mm)."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(0.01, 0.0, 0.0)  # 10mm bead
        length = 0.01
        
        # At 5mm (midpoint)
        tcp = make_point(0.005, 0.0, 0.0)
        result = ProgressionCalculator.calculate(tcp, start, end, length)
        assert result == pytest.approx(0.5)
        
        # At 2.5mm (25%)
        tcp = make_point(0.0025, 0.0, 0.0)
        result = ProgressionCalculator.calculate(tcp, start, end, length)
        assert result == pytest.approx(0.25)


class TestProgressionEdgeCases:
    """Edge case tests for progression calculation."""
    
    def test_negative_length_returns_zero(self):
        """Negative bead length should return 0 (invalid state)."""
        tcp = make_point(0.5, 0.0, 0.0)
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        
        result = ProgressionCalculator.calculate(tcp, start, end, -1.0)
        assert result == 0.0
    
    def test_very_small_bead(self):
        """Very small bead (1mm) should still calculate correctly."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(0.001, 0.0, 0.0)
        tcp = make_point(0.0005, 0.0, 0.0)
        
        result = ProgressionCalculator.calculate(tcp, start, end, 0.001)
        assert result == pytest.approx(0.5)
    
    def test_large_perpendicular_offset(self):
        """Large perpendicular offset should still project correctly."""
        start = make_point(0.0, 0.0, 0.0)
        end = make_point(1.0, 0.0, 0.0)
        # TCP is 1 meter away in Y but at X=0.5
        tcp = make_point(0.5, 1.0, 0.0)
        
        result = ProgressionCalculator.calculate(tcp, start, end, 1.0)
        assert result == pytest.approx(0.5)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
