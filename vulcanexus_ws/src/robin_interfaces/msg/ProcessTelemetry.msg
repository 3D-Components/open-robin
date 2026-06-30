# Aggregated process telemetry for DDS->FIWARE bridge.
# Field names MUST match what alert_engine.py parses from the compound
# NGSI-LD attribute (height, width, speed, current, voltage).

# Bead identification and progression
string bead_id              # Active weld bead ID (empty if not welding)
float32 progression         # 0.0 = bead start, 1.0 = bead end

float32 height                 # bead height (mm) -- from BeadGeometry
float32 width                  # bead width (mm) -- from BeadGeometry
float32 speed                  # wire feed speed (m/min) -- from WelderData
float32 current                # welding current (A) -- from WelderData
float32 voltage                # arc voltage (V) -- from WelderData
float32 cross_sectional_area   # optional, 0.0 if not computed
