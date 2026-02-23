# Fronius welder measurement with progression tracking
# Progression allows alignment with other data sources along the weld bead

std_msgs/Header header

# Bead identification and progression
string bead_id              # Active weld bead ID (empty if not welding)
float32 progression         # 0.0 = bead start, 1.0 = bead end

# Measured values from Fronius welder
float32 current             # Display current (A)
float32 voltage             # Display voltage (V)
float32 wire_feed_speed     # Display wire feed speed (m/min)
float32 power               # Display power (W)
