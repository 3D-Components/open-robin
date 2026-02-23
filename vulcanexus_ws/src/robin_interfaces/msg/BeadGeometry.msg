# The result of processing the profilometer's PointCloud2
# This is the "dedicated topic" for bead features.
std_msgs/Header header

# Using float32 instead of float64 for Orion-LD DDS compatibility
float32 height_mm
float32 width_mm
float32 cross_sectional_area_mm2

# Add timestamp for the observation
# builtin_interfaces/Time timestamp