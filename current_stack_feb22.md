# ROBIN Stack Snapshot (February 22, 2026)

## 1. Scope

This document captures the current technical baseline of the `open-robin` repository after the generic core refactor:

- core entities and APIs are process-agnostic (`Process`, `Measurement`, `AIRecommendation`)
- ROS 2 to Orion ingestion is DDS-first
- the legacy HTTP NGSI bridge path has been removed from runtime/demo flow

The welding domain remains available as a demo/adaptation profile, not as core naming.

---

## 2. Runtime Architecture

```mermaid
graph LR
  subgraph ROS2 / DDS
    BAG[rosbag / live topics]
    AGG[telemetry_aggregator_node]
    TOPIC[/robin/telemetry]
  end

  subgraph FIWARE
    ORION[Orion-LD + DDS module]
    MONGO[MongoDB]
    TSDB[TimescaleDB (TROE)]
    MINTAKA[Mintaka]
    ALERT[Alert Engine (FastAPI)]
    UI[FAROS UI]
  end

  BAG --> AGG --> TOPIC --> ORION
  ORION --> MONGO
  ORION --> TSDB --> MINTAKA
  ALERT --> ORION
  ALERT --> MINTAKA
  UI --> ALERT
```

---

## 3. Containers and Ports

| Service | Container | Host Port | Purpose |
|---|---|---|---|
| Orion-LD | `fiware-orion` | `1026` | Context broker + DDS ingestion |
| MongoDB | `db-mongo` | `27017` | Orion persistence |
| TimescaleDB | `fiware-timescaledb` | `5433` | TROE temporal backend |
| Mintaka | `fiware-mintaka` | `9090` | Temporal API |
| Alert Engine | `robin-alert-processor` | `8001` | API, process status, AI/deviation logic |
| FAROS UI | `robin-ui` | `5174` | React operator UI |
| Vulcanexus | `vulcanexus-bridge` | host net | ROS 2 workspace and rosbag playback |

Notes:

- Orion and Vulcanexus use `network_mode: host` for DDS discovery.
- `alert-processor` reaches Orion through `extra_hosts: orion-ld -> host-gateway`.

---

## 4. Core Data Model (Generic)

### Process entity

- ID pattern: `urn:ngsi-ld:Process:<process_id>`
- Type: `urn:robin:Process`
- Key attributes:
  - `operationMode`
  - process parameters (wire speed/current/voltage/speed in current demo)
  - `toleranceThreshold`
  - `urn:robin:processTelemetry` (compound telemetry from DDS path)

### Measurement entity

- Type: `urn:robin:Measurement`
- Linked to process via `processId`
- Includes geometry and optional machine signals.

### GeometryTarget entity

- Type: `urn:robin:GeometryTarget`
- Stores target height/width for geometry-driven operation mode.

### AIRecommendation entity

- Type: `urn:robin:AIRecommendation`
- Stores model outputs and suggested adjustments.

---

## 5. ROS 2 to NGSI-LD Ingestion (Current)

### Topic aggregation

Node:

- `vulcanexus_ws/src/robin_core_data/scripts/telemetry_aggregator_node.py`

Message:

- `vulcanexus_ws/src/robin_interfaces/msg/ProcessTelemetry.msg`

Default subscriptions:

- geometry topic (`/robin/weld_dimensions` in current welding demo bag)
- Fronius topic(s)
- TCP/pose topic

Published topic:

- `/robin/telemetry`

### Orion DDS mapping

`config-dds.json` maps:

- DDS topic `rt/robin/telemetry`
- entity type `urn:robin:Process`
- entity ID `urn:ngsi-ld:Process:ros_bridge`
- attribute `urn:robin:processTelemetry`

### Legacy bridge status

- `ngsi_bridge_node.py` is removed from active workflow.
- Demo and docs should use the DDS telemetry path above.

---

## 6. Alert Engine API Surface (`robin/alert_engine.py`)

Key endpoints in use:

| Method | Path | Purpose |
|---|---|---|
| GET | `/health` | backend and Orion health check |
| GET | `/processes/list` | list processes with status summary |
| GET | `/process/{id}` | process snapshot + latest measurements |
| GET | `/process/{id}/measurements` | time series from Mintaka (fallback Orion) |
| GET | `/process/{id}/status` | process lifecycle/status details |
| POST | `/create-process` | create process entity |
| POST | `/process/{id}/stop` | lifecycle control |
| POST | `/process/{id}/resume` | lifecycle control |
| POST | `/process/{id}/target` | create/update geometry target |
| GET | `/process/{id}/target` | read geometry target |
| POST | `/check-deviation` | compare measured vs expected/target |
| GET | `/ai/models` | list model checkpoints |
| POST | `/ai/models/select` | activate model |
| POST | `/ai/models/predict` | forward prediction |
| POST | `/ai-recommendation` | recommendation endpoint |

---

## 7. Frontend Stack

Directory:

- `robin-dashboard/`

Current role:

- operator UI consuming Alert Engine API
- process selector, telemetry views, deviation panel, model panel

Domain wording:

- UI terminology is now configurable via `robin-dashboard/src/config/domain.ts`
- defaults are process-generic; welding-specific terms can be injected via Vite env vars if needed for a demo profile

---

## 8. Demo Baseline

Primary flow:

- `demo/simulation-demo-rosbag.sh`

This script:

- starts required stack
- ensures `urn:ngsi-ld:Process:ros_bridge` exists
- runs the ROS telemetry aggregation + DDS path
- validates measurements via Alert Engine API

Welding specifics are currently isolated to demo sources/topics and optional hardware adapters.

Additional non-welding profile:

- `demo/profiles/spray_coating_profile.py` reuses the same core APIs with profile-level metric mapping.

---

## 9. Reusability Status (as of Feb 22, 2026)

Reusable core ready to publish:

1. FIWARE process data model and lifecycle APIs (`robin/`)
2. Alert/deviation engine logic (`robin/alert_engine.py`)
3. AI model selection/prediction serving hooks (`robin/ai/*`)
4. ROS telemetry aggregation + DDS ingestion pattern (`telemetry_aggregator_node.py`, `ProcessTelemetry.msg`, `config-dds.json`)
5. Process monitoring UI shell (`robin-dashboard/`) with configurable domain vocabulary

Domain/profile layer (kept as example/adapters):

1. welding hardware interfaces (Fronius, WAGO, etc.)
2. welding-specific bag topics and scripts
3. domain narratives in historical logbooks
