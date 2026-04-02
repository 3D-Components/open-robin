"""
Typed UI state for ROBIN operator panel.
"""

from dataclasses import dataclass, field


@dataclass
class CalibrationUiState:
    servo_active: bool = False
    jog_speed: float = 0.01
    jog_frame: str = 'base_link'
    jog_twist_dir: tuple[float, float, float, float, float, float] = (
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    )
    tcp_active_frame: str = ''
    tcp_stickout: float = 0.0
    tcp_stickout_calibrated: bool = False
    stickout_user_editing: bool = False


@dataclass
class OperatorPanelState:
    robot_ready: bool = False
    calibration: CalibrationUiState = field(default_factory=CalibrationUiState)