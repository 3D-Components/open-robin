# ROBIN ROS 2 System — Node & Feature Reference

This workspace is the **hardware-agnostic open module**: an intent-driven welding demo
that runs on synthetic data with no physical hardware, MoveIt, or Gazebo. The skills
ship a default **simulation mode** and a `use_simulation:=false` **hardware mode** that
calls generic ROS interfaces, so you can adapt the module to your own robot/process by
implementing nodes that advertise those interfaces.

## Architecture Overview

```
welding_demo / welding_robin_sim.launch.py        (no-hardware demo)
├── robot_state_publisher   ← robin_description/urdf/ur_fronius_garmo.urdf.xacro
├── foxglove_bridge         ← 3D view at ws://localhost:8765
└── welding_robin_demo.launch.py  (intent pipeline, use_simulation:=true)
    ├── welding_home_skill          → owns /joint_states; MoveToHome action
    ├── welding_seam_skill          → /joint_states_manual; ProcessTelemetry; weld/pause
    ├── welding_manual_skill        → ManualAdjust action
    ├── welding_recommendation_skill → RequestAIRecommendation action (HTTP)
    ├── welding_http_bridge         → HTTP :8766 → /intents
    └── welding_supervisor          → routes /intents → skill action servers
```

Data path to FIWARE: `welding_seam_skill` publishes `ProcessTelemetry` on
`/robin/telemetry`; Orion-LD's DDS bridge maps it to an NGSI-LD entity
(see `config-dds.json` and `docs/DDS_BRIDGE_ARCHITECTURE.md`).

---

## Packages & Nodes

### `welding_msgs`

Interface definitions for the intent pipeline.

| Type | Name | Purpose |
|------|------|---------|
| **msg** | `Intent` | Intent bus message (type, JSON data, source, modality, priority, confidence) |
| **action** | `ExecuteSeam` | Cartesian weld-seam goal (seam_id, weld_speed, wire_feed_rate) |
| **action** | `MoveToHome` | Home the arm (use_fast_speed) |
| **action** | `ManualAdjust` | Adjust a process parameter (name, value, unit) |
| **action** | `RequestAIRecommendation` | Request AI welding parameters (process_id, mode) |

### `welding_http_bridge`

Bridges dashboard / FIWARE events to ROS 2.

- **Node** `welding_http_bridge`: aiohttp server on **:8766**; `POST /intent` → publishes
  `welding_msgs/Intent` on `/intents`. Optionally registers an Orion-LD (NGSI-LD)
  subscription and forwards notifications, de-duplicated by `(entity_id, observedAt)`.

### `welding_supervisor`

Intent-to-skill mission controller.

- **Node** `welding_supervisor`: subscribes `/intents`; dispatches each intent to the
  matching skill action server; publishes `/doe/launch` (String) and `/weld_errors`
  (String); calls `weld/pause` (`std_srvs/SetBool`).

| Intent | Action / effect |
|--------|-----------------|
| `MOVE_TO_HOME` | `welding_home_skill/execute` |
| `EXECUTE_SEAM` / `START_PROCESS` | `welding_seam_skill/execute` |
| `REQUEST_AI_RECOMMENDATION` | `welding_recommendation_skill/execute` |
| `MANUAL_ADJUST` | `welding_manual_skill/execute` |
| `PAUSE_PROCESS` / `RESUME_PROCESS` | `weld/pause` (true / false) |
| `STOP_PROCESS` / `ESTOP` | cancel all goals → home (+ `/weld_errors` on ESTOP) |
| `LAUNCH_NEW_DOE` | publish `/doe/launch` → cancel all → home |

### `welding_home_skill`

- **LifecycleNode** `welding_home_skill`; action server `welding_home_skill/execute`
  (`MoveToHome`).
  - *Simulation* (default): sole `/joint_states` publisher; relays
    `/joint_states_manual`; interpolates to `HOME_RADIANS`.
  - *Hardware* (`use_simulation:=false`): action client to an external `/move_home`.

### `welding_seam_skill`

- **LifecycleNode** `welding_seam_skill`; action server `welding_seam_skill/execute`
  (`ExecuteSeam`).
  - *Simulation* (default): drives `/joint_states_manual`; publishes synthetic
    `robin_interfaces/ProcessTelemetry` on `/robin/telemetry`; hosts `weld/pause`
    (`std_srvs/SetBool`).
  - *Hardware*: action client to an external `/execute_bead`.

### `welding_manual_skill`

- **LifecycleNode** `welding_manual_skill`; action server `welding_manual_skill/execute`
  (`ManualAdjust`). Clamps to safe ranges.
  - *Hardware*: service clients `/fronius/set_current`, `/fronius/set_voltage`,
    `/fronius/set_wire_speed` (`robin_interfaces/SetFloat32`).

### `welding_recommendation_skill`

- **LifecycleNode** `welding_recommendation_skill`; action server
  `welding_recommendation_skill/execute` (`RequestAIRecommendation`). Calls the
  Process-Intelligence API at `ROBIN_API_URL` (default `http://localhost:8001`).

### `robin_interfaces`

Shared ROS 2 message, service, and action definitions used across the skills (the
schema also retains the richer set used by hardware-mode integrations).

| Type | Name | Purpose |
|------|------|---------|
| **msg** | `ProcessTelemetry` | Normalized telemetry for the DDS → FIWARE path |
| **msg** | `BeadGeometry`, `WeldProgression`, `WelderData`, `ActiveBead` | Weld geometry / progression / welder data |
| **msg** | `ExperimentBead`, `ExperimentBeadSpec`, `PlateLayout`, `WeldPlate` | Experiment / plate definitions |
| **srv** | `SetFloat32` / `SetInt32` / `SetBool` | Generic typed setters (e.g. Fronius params) |
| **srv** | `SetTcpMode`, `SetCtwd`, `FindSurface`, `CalibrateWireTip`, `CalibratePlatePlane` | TCP / calibration services |
| **srv** | `StartWeld`, `PlanExperiment`, `ApproveExperimentPlan`, `ReserveSlots`, `ClearReservedSlots` | Weld / experiment planning |
| **action** | `ExecuteBead`, `WeldExperiment` | Bead / experiment execution (hardware-mode) |

### `robin_description`

Hardware-agnostic URDF/xacro + meshes for the cell (no nodes).

- Entry point: `urdf/ur_fronius_garmo.urdf.xacro` (UR10e + Fronius weld torch + Garmo
  laser profilometer + welding table). Includes `ur.urdf.xacro` (uses external
  `ur_description` / `ur_robot_driver`), `robin_tools.xacro`, `scene_macro.xacro`, and
  the `fronius_macro` / `garmo_garline_macro` component macros. Meshes under `meshes/`.

---

> **Adapting to real hardware / another process:** provide nodes that advertise the
> hardware-mode interfaces named above (`/move_home`, `/execute_bead`, `/fronius/set_*`)
> for your robot and power source, then launch with `use_simulation:=false`. No fork of
> the module is required.
