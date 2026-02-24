# Robot Assets

This directory contains robot model files (URDF, meshes) for the FAROS visualization.

## Structure

```
robots/
├── robot_a/
│   ├── robot.urdf       # Robot description file
│   └── meshes/          # STL/DAE visual meshes
└── robot_b/
    ├── robot.urdf
    └── meshes/
```

## URDF Format

Place your URDF files here. They should reference meshes using relative paths:
```xml
<mesh filename="meshes/link_0.stl"/>
```

## Supported Formats
- URDF (recommended)
- GLTF/GLB (for pre-baked models)
