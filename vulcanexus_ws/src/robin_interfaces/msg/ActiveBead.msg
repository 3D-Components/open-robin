# Active weld bead information
# Published when a bead starts, used by data node to calculate progression

std_msgs/Header header

# Bead identification
string bead_id              # Unique bead identifier

# Bead geometry (for progression calculation)
geometry_msgs/Point start_point
geometry_msgs/Point end_point
float32 length              # Pre-calculated bead length (m)

# Target parameters for this bead
float32 target_speed        # Target travel speed (m/s)
float32 target_current      # Target current (A)
float32 target_voltage      # Target voltage (V)
float32 target_wire_speed   # Target wire feed speed (m/min)
