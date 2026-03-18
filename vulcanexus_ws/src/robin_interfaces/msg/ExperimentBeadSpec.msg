# ExperimentBeadSpec - System-agnostic bead input for experiment planning
#
# This schema is intentionally independent of robot/plate geometry.
# The planner will assign bead_id/plate_id and compute physical placement.

string input_id                    # Optional upstream identifier (can be empty)
float32 weld_current               # Target weld current (A)
float32 weld_speed                 # TCP speed during weld pass (m/s)
float32 stickout                   # Stickout target for this bead (m)
float32 scan_speed                 # TCP speed during scan pass (m/s), 0 => weld_speed
