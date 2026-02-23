# Welding Reference Profile

This is the reference profile for ROBIN robust demos.

## Canonical script

```bash
python demo/profiles/welding_profile.py --mode both --duration 120 --interval 2
```

This single script validates both operational modes with real alert generation.

## Dual operational modes in this demo

### Parameter-driven

- Operator chooses initial welding parameters (`wireSpeed`, `current`, `voltage`).
- AI predicts expected bead geometry.
- Streamed measurements are checked against AI prediction.

### Geometry-driven

- Operator sets target bead geometry.
- AI recommends process parameters.
- Expected geometry from AI-guided parameters is compared against measured geometry.

## Metric mapping

| Welding term | Core field | Unit |
|---|---|---|
| Bead Height | `measuredHeight` | mm |
| Bead Width | `measuredWidth` | mm |
| Wire Speed | `measuredSpeed` | m/min or mm/s (profile dependent display) |
| Welding Current | `measuredCurrent` | A |
| Arc Voltage | `measuredVoltage` | V |

## ROS 2 / DDS note

For ROS/DDS interoperability validation use:

```bash
./demo/simulation-demo-rosbag.sh
```

The robust canonical welding demo remains `welding_profile.py`; rosbag replay is a
separate transport-path utility.
