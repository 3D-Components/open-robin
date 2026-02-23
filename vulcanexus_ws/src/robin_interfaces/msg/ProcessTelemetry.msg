# Aggregated process telemetry for DDS -> NGSI-LD bridging.
#
# Keep fields as simple float32 primitives so Orion-LD's DDS bridge can
# deserialize and store them reliably.
std_msgs/Header header

# Primary output/quality metrics (example: height/width of a deposited feature)
float32 height
float32 width

# Primary input/actuation metrics (example: robot tool speed, machine power)
float32 speed
float32 current
float32 voltage

