# PlateLayout - Configuration for automatic bead placement on plates
# Beads are placed in parallel rows, filling one plate before moving to the next.

float32 bead_pitch                 # Legacy spacing field (m), treated as fallback for spacing_y
float32 margin_x                   # Margin from plate edge along bead direction (m)
float32 margin_y                   # Margin from plate edge perpendicular to bead (m)
float32 spacing_x                  # Spacing/stagger step along bead direction (m)
float32 spacing_y                  # Spacing between adjacent bead rows (m)
bool staggered                     # True => apply alternating x-offset per row
