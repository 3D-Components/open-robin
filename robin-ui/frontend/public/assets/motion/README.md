# Motion Data

This directory contains robot motion trajectories for playback in the FAROS visualization.

## Structure

```
motion/
├── trajectories/        # Pre-recorded motion paths
│   ├── demo_run_001.json
│   └── demo_run_002.json
└── workpieces/          # Workpiece geometry and bead paths
    └── plate_001.json
```

## Trajectory Format

JSON format with timestamped joint positions:
```json
{
  "robotId": "robotA",
  "fps": 30,
  "frames": [
    { "t": 0.0, "joints": [0, -90, 90, 0, 90, 0], "tcp": [x, y, z, rx, ry, rz] },
    { "t": 0.033, "joints": [...], "tcp": [...] }
  ]
}
```

## Coordinate System
- Right-handed, Z-up
- Units: millimeters, degrees
