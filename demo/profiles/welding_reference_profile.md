# Welding Reference Profile

This is the reference profile for ROBIN robust demos.

## Canonical script

```bash
python demo/profiles/welding_profile.py --mode both --duration 120 --interval 2
```

This single script validates both operational modes with real alert generation.

## Dual operational modes in this demo

### Parameter-driven

- Operator chooses initial welding AI inputs (`wire_feed_speed_mpm_model_input`, `travel_speed_mps_model_input`, `arc_length_correction_mm_model_input`).
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
| Controlled welding inputs | `inputParams` | mixed (`m/min`, `m/s`, `mm`) |

The welding demo records these controlled inputs in each measurement snapshot:

- `wire_feed_speed_mpm_model_input`
- `travel_speed_mps_model_input`
- `arc_length_correction_mm_model_input`

## ROS 2 / DDS note

For ROS/DDS interoperability validation use:

```bash
./demo/simulation-demo-rosbag.sh
```

The robust canonical welding demo remains `welding_profile.py`; rosbag replay is a
separate transport-path utility.
