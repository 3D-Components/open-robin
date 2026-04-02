# WeldPlate - Definition of a welding plate on the table
# A rectangular region where weld beads are deposited.

string plate_id                    # Unique plate identifier

# Plate position and orientation in base_link frame
geometry_msgs/Point origin         # Corner of plate (start corner for bead layout)
float32 width                      # m - direction perpendicular to bead travel
float32 length                     # m - direction along bead travel
float32 orientation                # Yaw angle in radians (rotation about Z in base_link frame)

# Calibrated surface height (set by touch sensing calibration)
float32 surface_z                  # Actual Z height of plate surface (m in base_link frame)
bool is_calibrated                 # Whether surface_z has been calibrated via touch sensing

# Corner + fixture margins used for plate-plane probing
string corner_id                   # front_left | front_right | rear_left | rear_right
float32 margin_x                   # Margin along plate length axis (m)
float32 margin_y                   # Margin along plate width axis (m)

# Plane calibration model: z = a*x + b*y + c in base_link
bool plane_calibrated
float32 plane_a
float32 plane_b
float32 plane_c
geometry_msgs/Point[] probe_points # Last successful 4-point probe set (base_link)
