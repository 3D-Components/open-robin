# ROBIN Evidence Pack

This folder is the top-level evidence index for `open-robin`. It documents bundled validation evidence so you can evaluate the modules and reproduce the results when building your own solution on top of them.

Use this folder for screenshots, diagrams, logs, exports, video links, and validation references. The bundled demonstrator media is from the physical ROBIN welding-cell setup; profile/user-demo runs can generate their own media and logs. Keep files small enough to browse on GitHub when possible. Larger videos or datasets can be linked from this README instead of committed directly.

## Entry Points

| Evidence item | Status | Location |
| --- | --- | --- |
| Architecture diagram | Added | [Full system architecture](architecture/robin-full-system-architecture.png); [FIWARE/context broker layer](architecture/fiware-context-broker-layer.png); [ROS 2/ROS4HRI system layer](architecture/ros2-ros4hri-system-layer.png); [ROBIN hardware layer](architecture/robin-hardware-layer.png) |
| Data-flow or sequence diagram | Added | [Orion DDS to intent bridge flow](architecture/orion-dds-intent-bridge-flow.png); [ROS4HRI intent layer](architecture/ros4hri-intent-layer.png) |
| ROBIN Dashboard screenshots | Added | [Live Ops with 3D visualization](screenshots/dashboard/dashboard-live-ops-3d-visualization.png); [AI recommendation confirmation](screenshots/dashboard/dashboard-ai-recommendation-confirmation.png) |
| Basic no-hardware demo evidence | Added | [Commands, expected outputs, and screenshots](basic-demo/README.md) |
| Physical ROBIN welding demonstrator media | Added | [Welding-cell demonstrator collage](screenshots/demonstrator/d3-welding-cell-demonstrator.png); [external demonstrator video](https://drive.google.com/file/d/1NPXDISejwRpxbMHidZyFyc0_4-ERQhcg/view?usp=drive_link) |
| User/profile demo screenshots or GIFs | Not bundled in v0.1.0 | Use the profile demo commands in [docs/user_guide/demos.rst](../docs/user_guide/demos.rst) to capture run-specific media. |
| User/profile demo logs or terminal output | Not bundled in v0.1.0 | Use the profile demo commands in [docs/user_guide/demos.rst](../docs/user_guide/demos.rst) to capture run-specific logs. |
| Example API responses (welding reference) | Added | [API response examples](api-responses/README.md): [health](api-responses/health-response.json), [create process](api-responses/create-process-response.json), [set target](api-responses/set-target-response.json), [measurements](api-responses/process-measurements-response.json), [deviation check](api-responses/check-deviation-response.json), [AI recommendation](api-responses/ai-recommendation-response.json), [operator intent](api-responses/publish-intent-response.json) |
| Example API responses (spray coating, non-welding) | Not bundled in v0.1.0 | Reuse beyond welding is covered by the [profile documentation](../docs/user_guide/profiles.rst) and [spray-coating demo script](../demo/profiles/spray_coating_profile.py). |
| CSV export example (welding reference) | Added | [CSV export notes](exports/README.md); [sample History / Reports export](exports/reviewer_demo_process-history-2026-06-16T12-02-00-000Z.csv) |
| CSV export example (spray coating, non-welding) | Not bundled in v0.1.0 | Generate a run-specific export from the dashboard after running the [spray-coating demo](../demo/profiles/spray_coating_profile.py). |
| No-hardware demo | Added | [No-hardware hello world](../docs/quickstart.rst); [simulated welding demo](../docs/user_guide/demos.rst); [adapt to another process via profiles](../docs/user_guide/profiles.rst) |
| Physical ROBIN demonstrator video | Available externally | [Google Drive video](https://drive.google.com/file/d/1NPXDISejwRpxbMHidZyFyc0_4-ERQhcg/view?usp=drive_link) |
| SUS/user-validation folder | Available externally | [Google Drive folder](https://drive.google.com/drive/folders/1JNtOJkZSoUBujClqRHi9FKAKPs3XxdlQ?usp=drive_link) |
| AI model evidence for open online model, R^2 about 0.90 | Added | [Open online model scatter plot](model-evidence/open-online-model-r2-090-scatter.png); [metrics JSON](model-evidence/open-online-model-r2-090-metrics.json); [predictions CSV](model-evidence/open-online-model-r2-090-predictions.csv); [model evidence notes](model-evidence/README.md) |
| Proprietary/offline benchmark, R^2 about 0.96 | Added as proprietary benchmark evidence | [Offline benchmark scatter plot](model-evidence/proprietary-offline-model-r2-096-scatter.png). This is evidence for the proprietary/offline benchmark and should be described as proprietary/offline if referenced externally. |

## Bundled Image Evidence

| File | Description |
| --- | --- |
| [basic-demo/screenshots/basic-demo-dashboard-live-ops.png](basic-demo/screenshots/basic-demo-dashboard-live-ops.png) | No-hardware basic demo dashboard evidence showing the selected `reviewer-basic-demo-20260630-083524` process and one ingested measurement. |
| [basic-demo/screenshots/basic-demo-api-measurement-readback.png](basic-demo/screenshots/basic-demo-api-measurement-readback.png) | No-hardware basic demo API read-back evidence showing `status=success`, `count=1`, `source=mintaka`, and the mock geometry values. |
| [basic-demo/screenshots/basic-demo-ai-recommendation.png](basic-demo/screenshots/basic-demo-ai-recommendation.png) | No-hardware basic demo AI recommendation evidence showing a successful parameter-driven prediction. |
| [architecture/robin-full-system-architecture.png](architecture/robin-full-system-architecture.png) | End-to-end ROBIN architecture covering dashboard, FIWARE/context broker layer, ROS 2/ROS4HRI layer, visualization, hardware, and physical welding cell components. |
| [architecture/fiware-context-broker-layer.png](architecture/fiware-context-broker-layer.png) | FIWARE/context broker layer showing dashboard/API access, Orion-LD, MongoDB, Mintaka, TimescaleDB/TROE, and telemetry persistence paths. |
| [architecture/ros2-ros4hri-system-layer.png](architecture/ros2-ros4hri-system-layer.png) | ROS 2 layer including ROS4HRI-inspired intent flow, skill routing, ROBIN hardware services, and physical hardware interfaces. |
| [architecture/ros4hri-intent-layer.png](architecture/ros4hri-intent-layer.png) | Focused ROS4HRI intent layer showing dashboard intent bridge, `/intents`, supervisor routing, and skill action servers. |
| [architecture/robin-hardware-layer.png](architecture/robin-hardware-layer.png) | Focused ROBIN hardware layer showing planner, experiment node, welding coordinator, data node, OPC UA bridge, Garmo, Fronius, WAGO, and UR10e connections. |
| [architecture/orion-dds-intent-bridge-flow.png](architecture/orion-dds-intent-bridge-flow.png) | Bridge-flow view connecting Orion-LD/DDS data with the ROS4HRI intent layer and selected hardware/data nodes. |
| [screenshots/dashboard/dashboard-live-ops-3d-visualization.png](screenshots/dashboard/dashboard-live-ops-3d-visualization.png) | ROBIN Dashboard Live Ops view with service health chips, robot controls, process controls, 3D visualization, deviation monitor, and telemetry area. |
| [screenshots/dashboard/dashboard-ai-recommendation-confirmation.png](screenshots/dashboard/dashboard-ai-recommendation-confirmation.png) | ROBIN Dashboard deviation workflow showing AI recommendation confirmation and operator apply/cancel decision point. |
| [screenshots/demonstrator/d3-welding-cell-demonstrator.png](screenshots/demonstrator/d3-welding-cell-demonstrator.png) | Physical ROBIN welding-cell demonstrator photo collage with welding operation, robot cell, local screen, torch/profilometer setup, and work table. |
| [model-evidence/open-online-model-r2-090-scatter.png](model-evidence/open-online-model-r2-090-scatter.png) | Open online Torch MLP benchmark plot from leave-one-out predictions: variance-weighted R2 = 0.8947, height R2 = 0.9019, width R2 = 0.8942. |
| [model-evidence/open-online-model-r2-090-metrics.json](model-evidence/open-online-model-r2-090-metrics.json) | Machine-readable metrics for the open online model benchmark. |
| [model-evidence/open-online-model-r2-090-predictions.csv](model-evidence/open-online-model-r2-090-predictions.csv) | Leave-one-out prediction records used to generate the open online model scatter plot. |
| [model-evidence/proprietary-offline-model-r2-096-scatter.png](model-evidence/proprietary-offline-model-r2-096-scatter.png) | Proprietary/offline model benchmark plot with generalized R^2 about 0.96; use only as external benchmark evidence, not as open reusable model evidence. |

## Bundled API Evidence

| File | Description |
| --- | --- |
| [api-responses/README.md](api-responses/README.md) | Index of representative API responses and the curl commands they correspond to. |
| [api-responses/health-response.json](api-responses/health-response.json) | Example `GET /health` response showing backend, Orion-LD, and Mintaka connectivity fields. |
| [api-responses/create-process-response.json](api-responses/create-process-response.json) | Example `POST /create-process` success response for the no-hardware demo process. |
| [api-responses/set-target-response.json](api-responses/set-target-response.json) | Example `POST /process/{process_id}/target` response showing a geometry target persisted for geometry-driven mode. |
| [api-responses/process-measurements-response.json](api-responses/process-measurements-response.json) | Example `GET /process/{process_id}/measurements?last=1` response showing a mock or recorded telemetry sample. |
| [api-responses/check-deviation-response.json](api-responses/check-deviation-response.json) | Example `POST /check-deviation` response showing expected geometry, measured geometry, deviation breakdown, and operator action options. |
| [api-responses/ai-recommendation-response.json](api-responses/ai-recommendation-response.json) | Example `POST /ai-recommendation` response showing advisory parameters and confidence. |
| [api-responses/publish-intent-response.json](api-responses/publish-intent-response.json) | Example `POST /intent` response showing operator intent publication for the FIWARE-to-ROS 2 bridge path. |

## Bundled Export Evidence

| File | Description |
| --- | --- |
| [exports/README.md](exports/README.md) | Notes for the ROBIN Dashboard History / Reports CSV export path. |
| [exports/reviewer_demo_process-history-2026-06-16T12-02-00-000Z.csv](exports/reviewer_demo_process-history-2026-06-16T12-02-00-000Z.csv) | Representative CSV export containing twelve telemetry rows and one persisted warning/deviation row for `reviewer_demo_process` (welding). |

## Human Authority Statement

ROBIN provides monitoring, deviation detection, and AI-assisted recommendations. These recommendations are advisory: the human operator remains the final authority for process decisions and can accept, reject, override, or adjust recommendations before action is taken.
