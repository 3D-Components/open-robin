# Fronius welder measurement with progression tracking
# Progression allows alignment with other data sources along the weld bead
#
# Legacy message retained for replay of pre-WelderData-refactor ROS bags
# (e.g. data/rosbags/exp001_rosbag_real). WelderData is this message plus a
# trailing `energy` field, so WelderData CDR bytes also deserialize as a
# FroniusSample (the shared leading fields), which the telemetry aggregator
# relies on to read both bag generations.

std_msgs/Header header

# Bead identification and progression
string bead_id              # Active weld bead ID (empty if not welding)
float32 progression         # 0.0 = bead start, 1.0 = bead end

# Measured values from Fronius welder
float32 current             # Display current (A)
float32 voltage             # Display voltage (V)
float32 wire_feed_speed     # Display wire feed speed (m/min)
float32 power               # Display power (W)
