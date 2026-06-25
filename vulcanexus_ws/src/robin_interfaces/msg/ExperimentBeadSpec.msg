# ExperimentBeadSpec - System-agnostic bead input for experiment planning
#
# This schema is intentionally independent of robot/plate geometry.
# The planner will assign bead_id/plate_id and compute physical placement.

string input_id                    # Optional upstream identifier (can be empty)
float32 wire_feed_speed            # Wire feed speed command (m/min)
float32 weld_speed                 # TCP speed during weld pass (m/s)
float32 arc_length_correction_mm   # Arc length correction for this bead (mm)
