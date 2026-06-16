# ROBIN D4 Evidence Pack

This folder is the top-level evidence index for the ARISE D4 "Shareable HRI Module" review of `open-robin`.

Use this folder for screenshots, diagrams, logs, exports, video links, and validation references that support the D4 submission. Keep files small enough for GitHub review when possible. Larger videos or datasets can be linked from this README instead of committed directly.

## Reviewer Entry Points

| Evidence item | Status | Location |
| --- | --- | --- |
| Architecture diagram | Added | [Full system architecture](architecture/robin-full-system-architecture.png); [FIWARE/context broker layer](architecture/fiware-context-broker-layer.png); [ROS 2/ROS4HRI system layer](architecture/ros2-ros4hri-system-layer.png); [ROBIN hardware layer](architecture/robin-hardware-layer.png) |
| Data-flow or sequence diagram | Added | [Orion DDS to intent bridge flow](architecture/orion-dds-intent-bridge-flow.png); [ROS4HRI intent layer](architecture/ros4hri-intent-layer.png) |
| ROBIN Dashboard screenshots | Added | [Live Ops with 3D visualization](screenshots/dashboard/dashboard-live-ops-3d-visualization.png); [AI recommendation confirmation](screenshots/dashboard/dashboard-ai-recommendation-confirmation.png) |
| Demonstrator screenshots | Added | [D3 welding cell demonstrator](screenshots/demonstrator/d3-welding-cell-demonstrator.png) |
| Demo screenshots or GIFs | To be added | `media/screenshots/demo/` |
| Demo logs or terminal output | To be added | `media/logs/` |
| Example API responses | To be added | `media/api-responses/` |
| CSV export example | To be added | `media/exports/` |
| D3 demonstrator video | Available externally | [Google Drive video](https://drive.google.com/file/d/1NPXDISejwRpxbMHidZyFyc0_4-ERQhcg/view?usp=drive_link) |
| SUS/user-validation folder | Available externally | [Google Drive folder](https://drive.google.com/drive/folders/1JNtOJkZSoUBujClqRHi9FKAKPs3XxdlQ?usp=drive_link) |
| AI model evidence for open online model, R^2 about 0.90 | To be added | `media/model-evidence/` |
| Proprietary/offline benchmark, R^2 about 0.96 | Added as D3 evidence | [Offline benchmark scatter plot](model-evidence/proprietary-offline-model-r2-096-scatter.png). This is evidence for the D3 benchmark and should be described as proprietary/offline if referenced in D4. |

## Bundled Image Evidence

| File | Description |
| --- | --- |
| [architecture/robin-full-system-architecture.png](architecture/robin-full-system-architecture.png) | End-to-end ROBIN architecture covering dashboard, FIWARE/context broker layer, ROS 2/ROS4HRI layer, visualization, hardware, and physical welding cell components. |
| [architecture/fiware-context-broker-layer.png](architecture/fiware-context-broker-layer.png) | FIWARE/context broker layer showing dashboard/API access, Orion-LD, MongoDB, Mintaka, TimescaleDB/TROE, and telemetry persistence paths. |
| [architecture/ros2-ros4hri-system-layer.png](architecture/ros2-ros4hri-system-layer.png) | ROS 2 layer including ROS4HRI-inspired intent flow, skill routing, ROBIN hardware services, and physical hardware interfaces. |
| [architecture/ros4hri-intent-layer.png](architecture/ros4hri-intent-layer.png) | Focused ROS4HRI intent layer showing dashboard intent bridge, `/intents`, supervisor routing, and skill action servers. |
| [architecture/robin-hardware-layer.png](architecture/robin-hardware-layer.png) | Focused ROBIN hardware layer showing planner, experiment node, welding coordinator, data node, OPC UA bridge, Garmo, Fronius, WAGO, and UR10e connections. |
| [architecture/orion-dds-intent-bridge-flow.png](architecture/orion-dds-intent-bridge-flow.png) | Bridge-flow view connecting Orion-LD/DDS data with the ROS4HRI intent layer and selected hardware/data nodes. |
| [screenshots/dashboard/dashboard-live-ops-3d-visualization.png](screenshots/dashboard/dashboard-live-ops-3d-visualization.png) | ROBIN Dashboard Live Ops view with service health chips, robot controls, process controls, 3D visualization, deviation monitor, and telemetry area. |
| [screenshots/dashboard/dashboard-ai-recommendation-confirmation.png](screenshots/dashboard/dashboard-ai-recommendation-confirmation.png) | ROBIN Dashboard deviation workflow showing AI recommendation confirmation and operator apply/cancel decision point. |
| [screenshots/demonstrator/d3-welding-cell-demonstrator.png](screenshots/demonstrator/d3-welding-cell-demonstrator.png) | D3 welding-cell demonstrator photo collage with welding operation, robot cell, local screen, torch/profilometer setup, and work table. |
| [model-evidence/proprietary-offline-model-r2-096-scatter.png](model-evidence/proprietary-offline-model-r2-096-scatter.png) | Proprietary/offline model benchmark plot with generalized R^2 about 0.96; use only as external benchmark evidence, not as open reusable model evidence. |

## Human Authority Statement

ROBIN provides monitoring, deviation detection, and AI-assisted recommendations. These recommendations are advisory: the human operator remains the final authority for process decisions and can accept, reject, override, or adjust recommendations before action is taken.

## Suggested Folder Layout

```text
media/
  README.md
  architecture/
  screenshots/
    dashboard/
    demonstrator/
    demo/
  logs/
  api-responses/
  exports/
  model-evidence/
```

## Evidence Notes

- Prefer descriptive filenames, for example `dashboard-live-ops-demo-process.png` or `api-health-response.json`.
- Add a short note beside each file in the table above once the evidence is committed or linked.
- Mark external or proprietary evidence explicitly. Do not commit private datasets, credentials, or proprietary deployment details.
- If an image or log comes from a D3 demonstrator run, include the date, scenario, and whether it used real hardware, recorded data, or mock data.
