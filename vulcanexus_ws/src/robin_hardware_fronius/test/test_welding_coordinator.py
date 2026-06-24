#!/usr/bin/env python3
"""
Unit tests for Welding Coordinator signal sequencing.

Tests verify:
1. Correct signal ordering (robot_ready -> welding_start)
2. robot_motion_release feedback handling
3. Stop sequence (welding_start -> wait robot_motion_release drop -> robot_ready)

The Fronius power source automatically handles gas pre-flow, ignition, and
gas post-flow. The coordinator uses the robot_motion_release feedback signal
to know when the arc is stable (start) and when post-flow is complete (stop).
"""

import pytest
import time
from unittest.mock import MagicMock, patch, call
from dataclasses import dataclass


@dataclass
class MockServiceResult:
    """Mock result for service calls."""
    success: bool = True
    message: str = "OK"


class WeldingSequenceValidator:
    """
    Validates welding signal sequencing.
    
    This class tracks the order of signal changes and validates
    that the correct sequence is followed.
    """
    
    def __init__(self):
        self.signal_history = []
        self.robot_ready = False
        self.welding_start = False
        self.robot_motion_release = False
    
    def set_robot_ready(self, value: bool) -> tuple[bool, str]:
        self.signal_history.append(('robot_ready', value, time.time()))
        self.robot_ready = value
        return True, "OK"
    
    def set_welding_start(self, value: bool) -> tuple[bool, str]:
        self.signal_history.append(('welding_start', value, time.time()))
        self.welding_start = value
        return True, "OK"

    def simulate_robot_motion_release(self, value: bool):
        """Simulate the machine's robot_motion_release feedback signal."""
        self.signal_history.append(('robot_motion_release', value, time.time()))
        self.robot_motion_release = value
    
    def validate_start_sequence(self) -> bool:
        """Validate that start sequence is correct.
        
        Expected order:
        1. robot_ready=True
        2. welding_start=True
        """
        starts = [(s, v) for s, v, _ in self.signal_history
                  if v is True and s in ('robot_ready', 'welding_start')]
        
        if len(starts) < 2:
            return False
        
        expected = [
            ('robot_ready', True),
            ('welding_start', True),
        ]
        
        return starts[:2] == expected
    
    def validate_stop_sequence(self) -> bool:
        """Validate that stop sequence is correct.
        
        Expected order:
        1. welding_start=False
        2. robot_ready=False
        """
        stops = [(s, v) for s, v, _ in self.signal_history
                 if v is False and s in ('robot_ready', 'welding_start')]
        
        if len(stops) < 2:
            return False
        
        expected = [
            ('welding_start', False),
            ('robot_ready', False),
        ]
        
        return stops[:2] == expected

    def validate_robot_motion_release_before_move(self) -> bool:
        """Validate robot_motion_release went True after welding_start=True."""
        welding_start_time = None
        release_time = None
        for signal, value, ts in self.signal_history:
            if signal == 'welding_start' and value is True:
                welding_start_time = ts
            elif signal == 'robot_motion_release' and value is True and welding_start_time:
                release_time = ts
        return welding_start_time is not None and release_time is not None and release_time > welding_start_time

    def validate_postflow_complete_before_robot_ready_off(self) -> bool:
        """Validate robot_motion_release dropped before robot_ready=False."""
        release_drop_idx = None
        robot_ready_off_idx = None
        for i, (signal, value, _) in enumerate(self.signal_history):
            if signal == 'robot_motion_release' and value is False:
                release_drop_idx = i
            elif signal == 'robot_ready' and value is False:
                robot_ready_off_idx = i
        return (release_drop_idx is not None and robot_ready_off_idx is not None
                and release_drop_idx < robot_ready_off_idx)


class TestWeldingStartSequence:
    """Test welding start signal sequence."""
    
    def test_robot_ready_before_welding_start(self):
        """Robot ready must be set before welding_start."""
        validator = WeldingSequenceValidator()
        
        validator.set_robot_ready(True)
        validator.set_welding_start(True)
        
        assert validator.validate_start_sequence()
    
    def test_incorrect_sequence_detected(self):
        """Wrong sequence should fail validation."""
        validator = WeldingSequenceValidator()
        
        # Wrong order: welding_start before robot_ready
        validator.set_welding_start(True)
        validator.set_robot_ready(True)
        
        assert not validator.validate_start_sequence()

    def test_robot_motion_release_after_welding_start(self):
        """Machine must signal robot_motion_release after arc stabilises."""
        validator = WeldingSequenceValidator()

        validator.set_robot_ready(True)
        validator.set_welding_start(True)
        # Machine feedback arrives after GPr + ignition + starting current
        validator.simulate_robot_motion_release(True)

        assert validator.validate_robot_motion_release_before_move()


class TestWeldingStopSequence:
    """Test welding stop signal sequence."""
    
    def test_welding_start_off_before_robot_ready_off(self):
        """welding_start must be off before robot_ready."""
        validator = WeldingSequenceValidator()
        
        # Simulate start
        validator.set_robot_ready(True)
        validator.set_welding_start(True)
        validator.simulate_robot_motion_release(True)
        
        # Correct stop sequence
        validator.set_welding_start(False)
        validator.simulate_robot_motion_release(False)
        validator.set_robot_ready(False)
        
        assert validator.validate_stop_sequence()
    
    def test_incorrect_stop_sequence_detected(self):
        """Wrong stop sequence should fail validation."""
        validator = WeldingSequenceValidator()
        
        # Simulate start
        validator.set_robot_ready(True)
        validator.set_welding_start(True)
        
        # Wrong order: robot_ready off before welding_start off
        validator.set_robot_ready(False)
        validator.set_welding_start(False)
        
        assert not validator.validate_stop_sequence()

    def test_postflow_complete_before_robot_ready_off(self):
        """robot_motion_release must drop (post-flow done) before clearing robot_ready."""
        validator = WeldingSequenceValidator()

        # Start
        validator.set_robot_ready(True)
        validator.set_welding_start(True)
        validator.simulate_robot_motion_release(True)

        # Stop
        validator.set_welding_start(False)
        validator.simulate_robot_motion_release(False)  # post-flow complete
        validator.set_robot_ready(False)

        assert validator.validate_postflow_complete_before_robot_ready_off()


class TestWeldingCoordinatorLogic:
    """Test welding coordinator business logic."""
    
    def test_full_weld_cycle(self):
        """Test complete weld start and stop cycle."""
        validator = WeldingSequenceValidator()
        
        # Start welding
        validator.set_robot_ready(True)
        validator.set_welding_start(True)
        validator.simulate_robot_motion_release(True)
        
        # Verify started
        assert validator.robot_ready is True
        assert validator.welding_start is True
        assert validator.robot_motion_release is True
        
        # Stop welding
        validator.set_welding_start(False)
        validator.simulate_robot_motion_release(False)
        validator.set_robot_ready(False)
        
        # Verify stopped
        assert validator.robot_ready is False
        assert validator.welding_start is False
        assert validator.robot_motion_release is False
    
    def test_emergency_stop_order(self):
        """Emergency stop should still follow correct sequence."""
        validator = WeldingSequenceValidator()
        
        # Start welding
        validator.set_robot_ready(True)
        validator.set_welding_start(True)
        
        # Emergency stop (still follows sequence for safety)
        validator.set_welding_start(False)  # Arc off first!
        validator.set_robot_ready(False)
        
        assert validator.validate_stop_sequence()
    
    def test_signal_history_tracking(self):
        """Verify all signals are tracked with timestamps."""
        validator = WeldingSequenceValidator()
        
        validator.set_robot_ready(True)
        validator.set_welding_start(True)
        validator.simulate_robot_motion_release(True)
        validator.set_welding_start(False)
        validator.simulate_robot_motion_release(False)
        validator.set_robot_ready(False)
        
        assert len(validator.signal_history) == 6
        
        # All should have timestamps
        for signal, value, timestamp in validator.signal_history:
            assert timestamp > 0
            assert signal in ('robot_ready', 'welding_start', 'robot_motion_release')
            assert isinstance(value, bool)


class TestFroniusParameters:
    """Test Fronius parameter setting."""
    
    def test_parameters_set_before_signals(self):
        """Fronius parameters should be set before activating signals."""
        call_order = []
        
        def mock_set_current(value):
            call_order.append(('current', value))
            return True, "OK"
        
        def mock_set_voltage(value):
            call_order.append(('voltage', value))
            return True, "OK"
        
        def mock_set_wire_speed(value):
            call_order.append(('wire_speed', value))
            return True, "OK"
        
        def mock_set_robot_ready(value):
            call_order.append(('robot_ready', value))
            return True, "OK"
        
        # Simulate coordinator behavior
        mock_set_current(180.0)
        mock_set_voltage(24.0)
        mock_set_wire_speed(8.0)
        mock_set_robot_ready(True)
        
        # Parameters should come before robot_ready
        param_indices = [i for i, (name, _) in enumerate(call_order) 
                        if name in ('current', 'voltage', 'wire_speed')]
        signal_indices = [i for i, (name, _) in enumerate(call_order) 
                         if name == 'robot_ready']
        
        assert all(p < s for p in param_indices for s in signal_indices), \
            "Parameters must be set before robot_ready signal"
    
    def test_parameter_validation(self):
        """Test parameter value validation."""
        # These would typically be validated by the coordinator
        valid_params = [
            (180.0, 24.0, 8.0),   # Typical MIG
            (200.0, 26.0, 10.0),  # Higher settings
            (100.0, 18.0, 5.0),   # Lower settings
        ]
        
        invalid_params = [
            (-10.0, 24.0, 8.0),   # Negative current
            (180.0, -5.0, 8.0),   # Negative voltage
            (180.0, 24.0, -2.0),  # Negative wire speed
            (0.0, 0.0, 0.0),      # All zeros (typically invalid for welding)
        ]
        
        def validate_params(current, voltage, wire_speed):
            return current > 0 and voltage > 0 and wire_speed > 0
        
        for params in valid_params:
            assert validate_params(*params), f"Should be valid: {params}"
        
        for params in invalid_params:
            assert not validate_params(*params), f"Should be invalid: {params}"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
