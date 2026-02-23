# Simulated data from the Fronius welder
std_msgs/Header header

# Welding parameters
float64 voltage_v         # Arc Voltage (V)
float64 current_a         # Welding Current (A)
float64 wire_feed_speed_mpm # Wire Feed Speed (m/min)

# Add timestamp for the observation
# builtin_interfaces/Time timestamp