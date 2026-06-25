"""Fronius TPS/i welding power source simulator.

Pure-Python state machine that models the Fronius behaviour as seen through
the WAGO PLC digital/analogue interface.  No ROS dependencies — the
FakeOpcUaBridge node owns an instance and maps PLC I/O to/from this class.

Signal flow (real system):
    Robot (ROS) → WAGO GVL_Fronius_IN → Fronius → WAGO GVL_Fronius_OUT → Robot (ROS)

This simulator replaces the Fronius + WAGO hardware:
    Service write → FroniusSimulator.set_input() → internal state machine
    Timer tick    → FroniusSimulator.tick()       → updated outputs
    Timer read    → FroniusSimulator.get_outputs() → published as topics
"""

from __future__ import annotations

import time
from enum import Enum, auto
from robin_simulation.synergy_model import synergy_lookup
from robin_simulation.laplace_bead_model import bead_dimensions_from_params, BeadDimensions


class WeldState(Enum):
    IDLE = auto()            # Power source ready, no welding
    GAS_PRE_FLOW = auto()    # Gas flowing before arc ignition
    IGNITION = auto()        # Arc strike attempt
    STARTING_CURRENT = auto()  # Arc burning at starting current
    WELDING = auto()         # Main welding (robot can move)
    END_CURRENT = auto()     # Ramp-down after welding_start=False
    GAS_POST_FLOW = auto()   # Post-flow gas purge
    TOUCH_SENSING = auto()   # Touch sense circuit active


class FroniusSimulator:
    """Models the Fronius TPS/i internal behaviour.

    All times are wall-clock seconds.  Call ``tick()`` periodically
    (e.g. at 10-50 Hz) to advance the state machine.
    """

    # -- Timing constants (seconds) — approximate real-machine values -------
    GAS_PRE_FLOW_TIME = 0.3
    IGNITION_TIME = 0.2
    STARTING_CURRENT_TIME = 0.5
    END_CURRENT_TIME = 0.3
    GAS_POST_FLOW_TIME = 0.5

    # Wire feed speed (mm/s) for simulated wire motion
    WIRE_FEED_SPEED_MM_S = 80.0  # typical value at moderate WFS

    def __init__(self):
        # --- GVL_Fronius_IN (written by robot / welding_coordinator) ---
        self._inputs: dict[str, bool | float | int] = {
            'robot_ready': False,
            'welding_start': False,
            'touch_sensing': False,
            'error_quit': False,
            'gas_on': False,
            'teach_mode': False,
            'torch_blow_out': False,
            'welding_simulation': False,
            'wire_forward': False,
            'wire_backward': False,
            'wire_sense_break': False,
            'wire_sense_start': False,
            'welding_speed': 0.0,
            'wire_move_length': 0.0,
            'working_mode': 0,
        }

        # --- GVL_Fronius_OUT (feedback to robot) ---
        self._outputs: dict[str, bool | float] = {
            'power_source_ready': True,
            'process_active': False,
            'arc_stable': False,
            'current_flow': False,
            'main_current_signal': False,
            'robot_motion_release': False,
            'touch_signal': False,
            'touch_signal_gas_nozzle': False,
            'warning': False,
            'heartbeat': False,
            'welding_current': 0.0,
            'welding_voltage': 0.0,
            'wire_feed_speed': 0.0,
        }

        # --- Fronius direct parameters (set via OPC UA, not WAGO) ---
        self._fronius_params: dict[str, float] = {
            'current_recommvalue': 0.0,
            'voltage_recommvalue': 0.0,
            'wfs_commandvalue': 0.0,
            'arc_length_correction': 0.0,
        }

        # --- Internal state ---
        self._state = WeldState.IDLE
        self._state_enter_time = time.monotonic()
        self._heartbeat_toggle_time = time.monotonic()

        # Wire position tracking (mm from "home")
        self._wire_position_mm = 0.0

        # Touch sensing: track wire position at moment touch_sensing was enabled
        self._touch_sense_wire_start_mm: float | None = None

        # Tick timing for energy integration
        self._last_tick_time = time.monotonic()

        # Accumulated weld energy (J) for current weld cycle
        self._weld_energy_J = 0.0

    # -- Public API for FakeOpcUaBridge ------------------------------------

    def set_input(self, name: str, value: bool | float | int) -> None:
        """Write a GVL_Fronius_IN signal (called when a service is invoked)."""
        self._inputs[name] = value

    def set_fronius_param(self, name: str, value: float) -> None:
        """Write a direct Fronius OPC UA parameter."""
        self._fronius_params[name] = value

    def get_input(self, name: str):
        """Read back a GVL_Fronius_IN signal (for readback topics)."""
        return self._inputs.get(name)

    def get_outputs(self) -> dict[str, bool | float]:
        """Return a copy of the current GVL_Fronius_OUT state."""
        return dict(self._outputs)

    def get_fronius_display(self) -> dict[str, float]:
        """Return simulated Fronius display/actual values.

        In synergetic mode the power source determines current and voltage
        from the wire feed speed via a synergy line.  Override
        ``synergy_line()`` to plug in real characteristic curves.
        """
        wfs_cmd = self._fronius_params['wfs_commandvalue']
        wfs, current, voltage, arc_length_correction = self.synergy_line(wfs_cmd)

        is_welding = self._state in (
            WeldState.STARTING_CURRENT, WeldState.WELDING, WeldState.END_CURRENT)

        if is_welding:
            display_current = current
            display_voltage = voltage
            display_wfs = wfs
            display_power = current * voltage
        else:
            display_current = 0.0
            display_voltage = 0.0
            display_wfs = 0.0
            display_power = 0.0

        # Accumulate energy (very rough — proper integration needs dt)
        # Kept as 0 for now; Step 3 can add a proper integrator.
        return {
            'fronius/display_current': display_current,
            'fronius/display_voltage': display_voltage,
            'fronius/display_wfs': display_wfs,
            'fronius/display_power': display_power,
            'fronius/display_energy': self._weld_energy_J,
            'fronius/actual_power': display_power,
            'fronius/actual_voltage': display_voltage,
            'fronius/actual_current': display_current,
            'fronius/wire_position': self._wire_position_mm,
            'fronius/actual_wfs': display_wfs,
            'fronius/current_recommvalue': self._fronius_params['current_recommvalue'],
            'fronius/voltage_recommvalue': self._fronius_params['voltage_recommvalue'],
            'fronius/wfs_commandvalue': wfs,
            'fronius/arc_length_correction': arc_length_correction,
        }

    def tick(self) -> None:
        """Advance the state machine.  Call at 10-50 Hz."""
        now = time.monotonic()
        dt = now - self._state_enter_time
        tick_dt = now - self._last_tick_time
        self._last_tick_time = now

        # Heartbeat toggles every 1.0s (like the real machine)
        if now - self._heartbeat_toggle_time >= 1.0:
            self._outputs['heartbeat'] = not self._outputs['heartbeat']
            self._heartbeat_toggle_time = now

        # Wire motion (independent of weld state)
        self._update_wire_motion()

        # Touch sensing (can be active outside welding)
        self._update_touch_sensing()

        # --- Main welding state machine ---
        if self._state == WeldState.IDLE:
            if self._inputs['welding_start'] and self._inputs['robot_ready']:
                self._enter_state(WeldState.GAS_PRE_FLOW)

        elif self._state == WeldState.GAS_PRE_FLOW:
            self._outputs['process_active'] = True
            if not self._inputs['welding_start']:
                self._abort_to_idle()
            elif dt >= self.GAS_PRE_FLOW_TIME:
                self._enter_state(WeldState.IGNITION)

        elif self._state == WeldState.IGNITION:
            if not self._inputs['welding_start']:
                self._enter_state(WeldState.GAS_POST_FLOW)
            elif dt >= self.IGNITION_TIME:
                self._outputs['current_flow'] = True
                self._outputs['main_current_signal'] = True
                self._enter_state(WeldState.STARTING_CURRENT)

        elif self._state == WeldState.STARTING_CURRENT:
            wfs_cmd = self._fronius_params['wfs_commandvalue']
            wfs, current, voltage, arc_corr = self.synergy_line(wfs_cmd)
            self._outputs['welding_current'] = current
            self._outputs['welding_voltage'] = voltage
            self._outputs['wire_feed_speed'] = wfs
            self._fronius_params['arc_length_correction'] = arc_corr
            if not self._inputs['welding_start']:
                self._enter_state(WeldState.END_CURRENT)
            elif dt >= self.STARTING_CURRENT_TIME:
                self._outputs['robot_motion_release'] = True
                self._outputs['arc_stable'] = True
                self._enter_state(WeldState.WELDING)

        elif self._state == WeldState.WELDING:
            wfs_cmd = self._fronius_params['wfs_commandvalue']
            wfs, current, voltage, arc_corr = self.synergy_line(wfs_cmd)
            self._outputs['welding_current'] = current
            self._outputs['welding_voltage'] = voltage
            self._outputs['wire_feed_speed'] = wfs
            self._fronius_params['arc_length_correction'] = arc_corr
            if not self._inputs['welding_start']:
                self._enter_state(WeldState.END_CURRENT)

        elif self._state == WeldState.END_CURRENT:
            self._outputs['robot_motion_release'] = False
            self._outputs['arc_stable'] = False
            if dt >= self.END_CURRENT_TIME:
                self._outputs['current_flow'] = False
                self._outputs['main_current_signal'] = False
                self._outputs['welding_current'] = 0.0
                self._outputs['welding_voltage'] = 0.0
                self._outputs['wire_feed_speed'] = 0.0
                self._enter_state(WeldState.GAS_POST_FLOW)

        elif self._state == WeldState.GAS_POST_FLOW:
            if dt >= self.GAS_POST_FLOW_TIME:
                self._outputs['process_active'] = False
                self._enter_state(WeldState.IDLE)

        # --- Energy accumulation during arc-active states ---
        if self._state in (WeldState.STARTING_CURRENT, WeldState.WELDING,
                           WeldState.END_CURRENT):
            self._weld_energy_J += (
                self._outputs['welding_current']
                * self._outputs['welding_voltage'] * tick_dt
            )

    # -- Synergy line (override for real curves) ----------------------------

    def synergy_line(self, wire_feed_speed: float) -> tuple[float, float, float, float]:
        arc_corr = self._fronius_params['arc_length_correction']
        return synergy_lookup(wire_feed_speed, arc_corr)

    # -- Internal helpers ---------------------------------------------------

    def _enter_state(self, new_state: WeldState) -> None:
        if new_state == WeldState.GAS_PRE_FLOW:
            self._weld_energy_J = 0.0
        self._state = new_state
        self._state_enter_time = time.monotonic()

    def _abort_to_idle(self) -> None:
        """Hard abort — clear all outputs and return to IDLE."""
        self._outputs['process_active'] = False
        self._outputs['arc_stable'] = False
        self._outputs['current_flow'] = False
        self._outputs['main_current_signal'] = False
        self._outputs['robot_motion_release'] = False
        self._outputs['welding_current'] = 0.0
        self._outputs['welding_voltage'] = 0.0
        self._outputs['wire_feed_speed'] = 0.0
        self._enter_state(WeldState.IDLE)

    def _update_wire_motion(self) -> None:
        if self._inputs['wire_forward']:
            self._wire_position_mm += self.WIRE_FEED_SPEED_MM_S * 0.02
        elif self._inputs['wire_backward']:
            self._wire_position_mm = max(0.0, self._wire_position_mm - self.WIRE_FEED_SPEED_MM_S * 0.02)

    # Simulated distance (mm) the wire must feed forward while touch_sensing
    # is active before a touch contact is generated.  Approximates the wire
    # travelling from a retracted position to the workpiece surface.
    TOUCH_FEED_DISTANCE_MM = 10.0

    def _update_touch_sensing(self) -> None:
        """Simulate touch contact in software.

        When touch_sensing is active and wire_forward is on, the wire feeds
        towards the workpiece.  After it has travelled TOUCH_FEED_DISTANCE_MM
        since touch_sensing was enabled, we assert touch_signal (simulating
        electrical contact with the workpiece).
        """
        if not self._inputs['touch_sensing']:
            # Touch mode off — reset everything
            self._outputs['touch_signal'] = False
            self._touch_sense_wire_start_mm = None
            return

        # Record wire position when touch_sensing is first enabled
        if self._touch_sense_wire_start_mm is None:
            self._touch_sense_wire_start_mm = self._wire_position_mm

        # Check if wire has fed far enough to "reach" the workpiece
        travel = self._wire_position_mm - self._touch_sense_wire_start_mm
        if travel >= self.TOUCH_FEED_DISTANCE_MM:
            self._outputs['touch_signal'] = True

    @property
    def state(self) -> WeldState:
        return self._state

    @property
    def wire_position_mm(self) -> float:
        return self._wire_position_mm

    def reset_wire_position(self) -> None:
        """Reset wire position tracking (e.g. after retract)."""
        self._wire_position_mm = 0.0

    def predict_bead_dimensions(
        self, travel_speed_mm_per_s: float = 5.0,
    ) -> BeadDimensions:
        """Predict bead dimensions using the Laplace capillary model.

        Uses the current synergy line parameters (WFS → current, voltage)
        and the given travel speed to compute expected bead width, height,
        and contact angle.
        """
        wfs_cmd = self._fronius_params['wfs_commandvalue']
        wfs, current, voltage, _ = self.synergy_line(wfs_cmd)
        return bead_dimensions_from_params(current, wfs, travel_speed_mm_per_s)