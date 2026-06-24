# ExperimentBead - Dashboard-facing bead specification
# Defines welding parameters for a single bead without positional data.
# The robot system computes physical placement based on plate layout.
#
# The Fronius welder operates in synergetic mode: the operator commands
# wire feed speed (WFS) and the machine automatically determines the
# optimal current and voltage. These are read back, never commanded.

# Bead identification
string bead_id                    # Unique identifier for this bead

# Which plate to deposit on
string plate_id                   # References a WeldPlate by ID

# Motion
float32 weld_speed                # Travel speed in m/s

# Welding command (synergetic — WFS is the sole input)
float32 wire_feed_speed           # Wire feed speed command (m/min)

# Arc tuning
float32 arc_length_correction_mm  # Arc length correction (mm)
