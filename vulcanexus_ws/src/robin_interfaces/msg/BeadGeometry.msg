std_msgs/Header header

# Bead identification and progression
string bead_id              # Active bead ID (empty if not welding)
float32 progression         # 0.0 = bead start, 1.0 = bead end

# Cross-section measurements
float32 height_mm
float32 width_mm
float32 toe_angle_rad       # Mean toe angle (left+right)/2 in radians