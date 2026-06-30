# Weld progression tracking message
# Progression (0.0-1.0) indicates normalized position along the active bead
# This allows spatio-temporal alignment of data from different sources
# by matching progression values rather than timestamps

std_msgs/Header header

# Bead identification
string bead_id              # Active weld bead ID (empty if not welding)

# Progression along the bead
float32 progression         # 0.0 = bead start, 1.0 = bead end

# Pass state — exactly one is True during an active pass, both False when idle
bool is_welding             # True during weld pass (arc on)
bool is_scanning            # True during scan pass (laser active)

# TCP position at this progression point
geometry_msgs/Point tcp_position
