# Aggregated telemetry published on /robin/telemetry by telemetry_aggregator_node.
# Field names (current, voltage, speed, width, height) are intentional:
# Orion-LD DDS bridge serialises them into the urn:robin:processTelemetry compound JSON
# attribute with these exact keys, which are then parsed by timescale_plot_sanity.py,
# mintaka_plot_sanity.py and the Alert Engine.
std_msgs/Header header

# From /robin/data/fronius (WelderData)
float32 current           # Welding current (A)
float32 voltage           # Arc voltage (V)
float32 speed             # Wire feed speed (m/min)

# From /robin/weld_dimensions (BeadGeometry)
float32 width             # Bead width (mm)
float32 height            # Bead height (mm)
float32 cross_sectional_area  # Cross-sectional area (mm^2)
