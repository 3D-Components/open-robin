# ExperimentBead - Dashboard-facing bead specification
# Defines welding parameters for a single bead without positional data.
# The robot system computes physical placement based on plate layout.

# Bead identification
string bead_id                    # Unique identifier for this bead

# Which plate to deposit on
string plate_id                   # References a WeldPlate by ID

# Robot motion parameter
float32 target_speed              # Travel speed in m/s

# Welding parameter mode
# The Fronius welder supports synergetic mode where setting one parameter
# causes the other two to adjust automatically.
# primary_parameter selects which parameter the operator is controlling:
uint8 PRIMARY_CURRENT=0
uint8 PRIMARY_VOLTAGE=1
uint8 PRIMARY_WIRE_FEED_SPEED=2

uint8 primary_parameter           # Which parameter to set as primary (see constants above)
float32 primary_value             # Value for the primary parameter

# Optional overrides (0.0 = let synergy decide / don't set)
# Only sent to the welder if non-zero, allowing manual override of synergy
float32 target_voltage            # Voltage override (V), 0.0 = synergy
float32 target_current            # Current override (A), 0.0 = synergy
float32 wire_feed_speed           # Wire feed speed override (m/min), 0.0 = synergy
