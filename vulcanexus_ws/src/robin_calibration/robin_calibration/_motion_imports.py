"""Lazy motion-related imports for robin_calibration.

These imports are deferred to avoid hard build-time dependency on
MoveIt packages, which live in the robin_core_planner package.
The MotionPlanner and related types are only needed at runtime when
calibration service callbacks execute.
"""

from robin_core_planner.motion import MotionPlanner
from robin_core_planner.bead_layout import compute_weld_velocity_scaling
from moveit.planning import PlanRequestParameters

__all__ = ["MotionPlanner", "compute_weld_velocity_scaling", "PlanRequestParameters"]
