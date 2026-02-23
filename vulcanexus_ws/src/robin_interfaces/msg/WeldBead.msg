# TODO:Weld bead should have nothing to do with positioning, only the welding parameters and length
string bead_id                # Unique identifier for this weld bead
geometry_msgs/Point start_point
geometry_msgs/Point end_point
float32 target_speed
float32 target_voltage
float32 target_current
float32 wire_feed_speed
float32 stickout  # CTWD (Contact Tip to Work Distance) in meters, default ~0.015m (15mm)