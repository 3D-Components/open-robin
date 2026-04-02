# ROBIN Operations Reference — Welding Intent Pipeline & Demos

## Architecture overview

```
React Dashboard  (http://localhost:5174)
   │  POST http://localhost:8001/intent
   ▼
Alert Engine     (robin-alert-processor, port 8001)
   │  PATCH pendingIntent on urn:ngsi-ld:Process:{id}
   ▼
Orion-LD         (fiware-orion, port 1026)  ←── telemetry from DDS bridge
   │  NGSI-LD subscription notification
   │  POST http://localhost:8766/orion-notify
   ▼
welding_http_bridge  (vulcanexus-bridge, port 8766)
   │  publish welding_msgs/Intent
   ▼
/intents ROS2 topic
   ▼
welding_supervisor  →  skill action servers
```

---

## 1. Start the Docker stack

Run from the repository root:

```bash
# Start all services
docker compose up -d

# Verify all containers are healthy
docker compose ps
```

| URL | Service |
|-----|---------|
| `http://localhost:5174` | ROBIN dashboard |
| `http://localhost:8001` | Alert Engine API |
| `http://localhost:1026` | Orion-LD context broker |
| `http://localhost:9090` | Mintaka temporal API |
| `http://localhost:8766` | welding_http_bridge (inside vulcanexus) |

---

## 2. Launch the welding intent pipeline

Open a shell inside the vulcanexus container, source the workspace, and launch:

```bash
# Step 1 — enter the container
docker exec -it vulcanexus-bridge bash

# Step 2 — source the workspace (inside container)
source ws_setup.sh    # or: source /opt/ros/jazzy/setup.bash && source install/setup.bash

# Step 3 — launch the full intent pipeline
ros2 launch welding_demo welding_robin_demo.launch.py
```

**What starts (in order):**

| t | Node | Package | Role |
|---|------|---------|------|
| 0 s | `welding_home_skill` | welding_home_skill | Action server — move to home position |
| 0 s | `welding_seam_skill` | welding_seam_skill | Action server — execute weld seam |
| 0 s | `welding_recommendation_skill` | welding_recommendation_skill | Action server — AI recommendation |
| 0 s | `welding_manual_skill` | welding_manual_skill | Action server — manual parameter adjust |
| 0 s | `welding_http_bridge` | welding_http_bridge | HTTP server on **port 8766**, registers Orion-LD subscription |
| 2 s | `welding_supervisor` | welding_supervisor | Intent router — subscribes to `/intents`, dispatches to skills |

**Expected log lines at startup:**
```
[welding_http_bridge]: WeldingHttpBridgeNode ready — publishing on /intents, HTTP server on port 8766
[welding_http_bridge]: Subscription registered (or already active): urn:ngsi-ld:Subscription:welding-intent-bridge
[welding_supervisor]: WeldingSupervisor ready — waiting for intents on /intents
```

---

## 3. Monitor intents triggered by dashboard buttons

Use any combination of these four methods simultaneously in separate terminals.

### Method 1 — Raw ROS2 topic (inside container)

```bash
docker exec -it vulcanexus-bridge bash -c "
  source ws_setup.sh &&
  ros2 topic echo /intents welding_msgs/msg/Intent"
```

Each button press prints a message like:
```yaml
intent: START_PROCESS
data: '{"seam_id": "seam_01", "weld_speed": 5.0}'
source: 1      # SOURCE_REMOTE
modality: 2    # MODALITY_TOUCHSCREEN
priority: 0.5
confidence: 1.0
```

### Method 2 — Watch Orion-LD pendingIntent (from host)

Polls Orion-LD every 0.8 s and prints whenever the `pendingIntent` attribute changes:

```bash
bash demo/watch-intents.sh

# Customise entity or Orion URL:
ENTITY_ID=urn:ngsi-ld:Process:my-process bash demo/watch-intents.sh
ORION_URL=http://localhost:1026 bash demo/watch-intents.sh
POLL_INTERVAL=0.5 bash demo/watch-intents.sh
```

Sample output:
```
[14:22:07.341] pendingIntent updated:
{
  "intent": "START_PROCESS",
  "data": {"seam_id": "seam_01"},
  "source": "dashboard"
}
```

### Method 3 — Direct bridge test (bypasses Orion-LD)

Sends intent straight to `welding_http_bridge` on port 8766. Use this to verify
the bridge and ROS2 publisher work in isolation:

```bash
curl -s -X POST http://localhost:8766/intent \
  -H 'Content-Type: application/json' \
  -d '{"intent": "START_PROCESS", "data": {"seam_id": "seam_01"}}' | python3 -m json.tool
```

Expected response:
```json
{"status": "published", "intent": "START_PROCESS"}
```

### Method 4 — Full Path A via Alert Engine (mirrors the dashboard button)

Sends intent through the full production path:
Alert Engine → Orion-LD PATCH → subscription notification → bridge → `/intents`

```bash
curl -s -X POST http://localhost:8001/intent \
  -H 'Content-Type: application/json' \
  -d '{"intent": "START_PROCESS", "process_id": "ros_bridge", "data": {"seam_id": "seam_01"}}' \
  | python3 -m json.tool
```

Expected response:
```json
{"status": "published", "intent": "START_PROCESS", "process_id": "ros_bridge"}
```

---

## 4. Intent constants reference

All valid intent types (from `welding_msgs/msg/Intent.msg`):

| Intent | Dashboard button | Routed to |
|--------|-----------------|-----------|
| `START_PROCESS` | Start | welding_seam_skill |
| `EXECUTE_SEAM` | — | welding_seam_skill |
| `MOVE_TO_HOME` | — | welding_home_skill |
| `MOVE_TO_ZONE` | — | welding_home_skill |
| `REQUEST_AI_RECOMMENDATION` | AI Recommend | welding_recommendation_skill |
| `MANUAL_ADJUST` | Manual Adjust | welding_manual_skill |
| `FINE_TUNE_MODEL` | Fine-tune | welding_manual_skill |
| `ESTOP` | Abort | cancels all active goals |

---

## 5. Fire all intent types at once

Sends every intent type sequentially through the full alert-engine → Orion-LD → bridge path:

```bash
# Default (process "ros_bridge", 2 s pause between intents)
bash demo/test-intent.sh

# Custom process ID
bash demo/test-intent.sh weld-1234567890

# Environment variable overrides
ALERT_ENGINE_URL=http://localhost:8001 \
INTENT_PAUSE=1 \
  bash demo/test-intent.sh ros_bridge
```

**Prerequisites before running:**
- Docker stack running (`docker compose up -d`)
- Intent pipeline launched inside container (`ros2 launch welding_demo welding_robin_demo.launch.py`)
- Optional: Method 1 or 2 open in a separate terminal to observe arrivals

---

## 6. Rosbag demos

### 6a. Continuous loop replay — DDS → Orion-LD → Dashboard

Replays `data/rosbags/exp001_rosbag_real` in a loop, streaming telemetry through
DDS into Orion-LD so the dashboard shows live-looking measurements.

```bash
# From repository root (host machine)
bash demo/simulation-demo-rosbag.sh
```

**What the script does:**
1. Starts FIWARE stack and vulcanexus container if not running
2. Restarts Orion-LD to reload `config-dds.json` DDS topic allowlist
3. Installs a TimescaleDB trigger to preserve ROS timestamps in TROE
4. Starts `telemetry_aggregator_node` (aggregates DDS geometry + Fronius data → `/robin/telemetry`)
5. Starts `foxglove_bridge` on `ws://localhost:8765` for Lichtblick 3D viewer
6. Starts `robot_state_publisher` with current URDF (fresh, not from bag)
7. Plays the bag on loop at 1× speed (Ctrl+C to stop)

**Environment variables:**
```bash
ORION_URL=http://127.0.0.1:1026   # default
```

**Check data is flowing while bag plays:**
```bash
# Measurements visible in Alert Engine
curl -s 'http://localhost:8001/process/ros_bridge/measurements?last=5' | python3 -m json.tool

# Orion-LD entity state
curl -s 'http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:Process:ros_bridge' | python3 -m json.tool
```

**Dashboard:**
- Open `http://localhost:5174`, select process `ros_bridge`
- Switch panel to the **Lichtblick** tab for 3D visualization
- Lichtblick: open `http://localhost:8080` → Connection → Rosbridge WebSocket → `ws://localhost:8765`

---

### 6b. One-shot E2E pipeline test — 2026-03-16 bag

Plays `bag_2026-03-16` once (no loop) and prints a message-count report at every
pipeline stage to verify data flows from bag to dashboard with acceptable loss.

```bash
bash demo/test-e2e-rosbag-2026-03-16.sh
```

**Ground truth from bag metadata:**

| Topic | Message count | Duration |
|-------|--------------|---------|
| `/robin/data/fronius` | 906 | 290.6 s |
| `/robin/weld_dimensions` | 6046 | 290.6 s |
| Aggregator output (`/robin/telemetry`) | ~290 (1 Hz) | — |

**Sample output:**
```
==================================================================
  END-TO-END PIPELINE MESSAGE COUNT REPORT
==================================================================
  Bag playback duration                  293s (bag: 291s)
  ------------------------------------------------------------------
  Bag /robin/data/fronius (ground truth) 906
  Bag /robin/weld_dimensions (ground truth) 6046
  ------------------------------------------------------------------
  Aggregator published (/robin/telemetry) 290  (expected ~291)
  Counter received (/robin/telemetry)    290
  ------------------------------------------------------------------
  TimescaleDB new rows (TROE)            278  (baseline was 0)
  Mintaka temporal entries               278
  Alert Engine count (source: mintaka)   278
==================================================================
LOSS ANALYSIS:
  DDS loss (aggregator→TimescaleDB): 4.1%  [278/290]
```

**Environment variable overrides:**
```bash
ORION_URL=http://127.0.0.1:1026 \
MINTAKA_URL=http://127.0.0.1:9090 \
ALERT_ENGINE_URL=http://127.0.0.1:8001 \
  bash demo/test-e2e-rosbag-2026-03-16.sh
```

**Post-run sanity plots:**
```bash
# TimescaleDB time series plot
python3 scripts/timescale_plot_sanity.py ros_bridge

# Mintaka temporal API plot
MINTAKA_URL=http://localhost:9090 python3 scripts/mintaka_plot_sanity.py ros_bridge
```

---

## 7. Alert Engine API spot-checks

All from the host machine. Base URL: `http://localhost:8001`

```bash
# Health — check all service connections
curl -s http://localhost:8001/health | python3 -m json.tool

# List all known processes
curl -s http://localhost:8001/processes/list | python3 -m json.tool

# Process snapshot (operation mode, status, wire speed, etc.)
curl -s http://localhost:8001/process/ros_bridge | python3 -m json.tool

# Last 5 measurements
curl -s 'http://localhost:8001/process/ros_bridge/measurements?last=5' | python3 -m json.tool

# All measurements (full count)
curl -s http://localhost:8001/process/ros_bridge/measurements | python3 -m json.tool

# Last 5 deviation alerts
curl -s 'http://localhost:8001/process/ros_bridge/alerts?last=5' | python3 -m json.tool

# Check deviation against current measurements
curl -s -X POST http://localhost:8001/check-deviation \
  -H 'Content-Type: application/json' \
  -d '{"process_id": "ros_bridge", "mode": "geometry_driven"}' | python3 -m json.tool

# Verify NGSI-LD subscription exists and is active
curl -s http://localhost:1026/ngsi-ld/v1/subscriptions \
  -H 'Accept: application/json' | python3 -m json.tool
# Look for id: "urn:ngsi-ld:Subscription:welding-intent-bridge"
# and status: "active"
```

---

## 8. Quick diagnostics

### Container logs

```bash
# welding_http_bridge + supervisor + skills (look for "Published intent" lines)
docker logs vulcanexus-bridge 2>&1 | tail -30
docker logs vulcanexus-bridge 2>&1 | grep -E "intent|orion-notify|ERROR" | tail -20

# Alert Engine (look for /intent request logs)
docker logs robin-alert-processor 2>&1 | tail -30

# Orion-LD (look for subscription notification dispatches)
docker logs fiware-orion 2>&1 | tail -30
```

### Orion-LD entity state

```bash
# Full Process entity (shows pendingIntent, processStatus, telemetry attrs)
curl -s 'http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:Process:ros_bridge' \
  -H 'Accept: application/json' | python3 -m json.tool

# Only the pendingIntent attribute
curl -s 'http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:Process:ros_bridge?attrs=pendingIntent' \
  -H 'Accept: application/json' | python3 -m json.tool

# List all Process entities
curl -s 'http://localhost:1026/ngsi-ld/v1/entities?type=urn:robin:Process' \
  -H 'Accept: application/json' | python3 -m json.tool
```

### TimescaleDB telemetry count

```bash
# Total rows for ros_bridge process
docker exec fiware-timescaledb psql -U orion -d orion -tAq -c \
  "SELECT COUNT(*) FROM attributes
   WHERE entityid='urn:ngsi-ld:Process:ros_bridge'
   AND id='urn:robin:processTelemetry';"

# Last 5 inserted timestamps
docker exec fiware-timescaledb psql -U orion -d orion -c \
  "SELECT observedat, ts FROM attributes
   WHERE entityid='urn:ngsi-ld:Process:ros_bridge'
   AND id='urn:robin:processTelemetry'
   ORDER BY ts DESC LIMIT 5;"
```

### Mintaka temporal entries

```bash
curl -s 'http://localhost:9090/temporal/entities/urn:ngsi-ld:Process:ros_bridge' \
  -G \
  --data-urlencode 'attrs=urn:robin:processTelemetry' \
  --data-urlencode 'timerel=between' \
  --data-urlencode 'timeAt=1970-01-01T00:00:00Z' \
  --data-urlencode 'endTimeAt=2030-01-01T00:00:00Z' \
  --data-urlencode 'lastN=10' \
  | python3 -m json.tool
```

### Telemetry aggregator log (rosbag runs)

```bash
docker exec vulcanexus-bridge tail -30 /tmp/aggregator.log
```

---

## 9. Rebuild cheatsheet

All commands run **inside the vulcanexus container** after `source ws_setup.sh`.

```bash
# Enter container
docker exec -it vulcanexus-bridge bash
source ws_setup.sh

# Rebuild only welding_http_bridge (picks up code changes without full rebuild)
colcon build --symlink-install --packages-select welding_http_bridge

# Rebuild only the welding intent pipeline packages
colcon build --symlink-install --packages-select \
  welding_msgs \
  welding_http_bridge \
  welding_supervisor \
  welding_home_skill \
  welding_seam_skill \
  welding_recommendation_skill \
  welding_manual_skill \
  welding_finetune_skill \
  welding_demo

# Rebuild Alert Engine container (after changes to robin/ or docker-compose.yaml)
# Run from host:
docker compose build alert-processor
docker compose up -d alert-processor

# Rebuild dashboard container (after changes to robin-dashboard/src/)
docker compose build robin-dashboard
docker compose up -d robin-dashboard

# Confirm ORION_URL is set correctly inside alert-processor
docker exec robin-alert-processor env | grep ORION
# Expected: ORION_URL=http://orion-ld:1026

# Full workspace build (first time or after adding packages)
colcon build --symlink-install
# Or release build for rosbag demos:
colcon build --packages-up-to robin_core_bringup robin_core_data \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```
