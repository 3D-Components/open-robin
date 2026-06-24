# Active weld bead information
# Published when a bead starts, used by data node to calculate progression

std_msgs/Header header

# Bead identification
string bead_id              # Unique bead identifier

# Bead path (for progression calculation)
# Ordered waypoints defining the bead centreline. The data node projects
# the TCP position onto this polyline to compute normalised progression.
geometry_msgs/Point[] path
float32 total_length        # Pre-calculated total path length (m)

# Target parameters for this bead
float32 weld_speed          # Travel speed (m/s)
float32 wire_feed_speed     # Wire feed speed command (m/min)

# Fronius synergetic recommendation values (read-back after param preload)
float32 voltage_recommvalue   # Recommended voltage (V)
float32 current_recommvalue   # Recommended current (A)
float32 arc_length_correction_mm  # Arc length correction applied (mm)
