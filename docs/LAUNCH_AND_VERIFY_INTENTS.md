# Launch & Verify: ROBIN Intent Flow (ros4hri)

Quick guide to bring up the full ROBIN stack and confirm that UI button presses reach the `/intents` ROS 2 topic.

---

## Architecture

```
Dashboard button (React)
  └─► POST http://localhost:8001/intent   (alert-processor)
        └─► PATCH Orion-LD pendingIntent
              └─► NGSI-LD subscription notification
                    └─► POST http://localhost:8766/orion-notify  (welding_http_bridge)
                          └─► /intents  (ROS 2 topic)
                                └─► welding_supervisor → skill action servers
```

Alternative direct path for testing (bypasses Orion-LD):
```
curl POST http://localhost:8766/intent  →  /intents
```

---

## Step 1 — Start Docker services

```bash
cd ~/open-robin
docker compose up -d orion-ld mongo-db timescaledb mintaka alert-processor robin-dashboard
```

Wait ~15 s for Orion-LD and TimescaleDB to finish initialising.

Health check:
```bash
curl -s http://localhost:8001/health | jq
# Expected: {"status": "ok"}
```

---

## Step 2 — Start ROS 2 skill stack (inside vulcanexus container)

### 2a. Start the container (if not already running)
```bash
docker compose up -d vulcanexus
docker exec -it vulcanexus-bridge bash
```

### 2b. Build the workspace (first time only, or after code changes)
```bash
cd /workspace/ros2_packages
colcon build --symlink-install 2>&1 | tail -5
source install/setup.bash
```

### 2c. Launch the full intent pipeline (simulation mode — no real hardware)
```bash
ros2 launch welding_demo welding_robin_demo.launch.py
```

This starts in one command:
- `welding_home_skill`, `welding_zone_skill`, `welding_seam_skill`
- `welding_recommendation_skill`, `welding_manual_skill`, `welding_finetune_skill`
- `welding_http_bridge` (HTTP server on port 8766)
- `welding_supervisor` (intent router, delayed 2 s)

> **Hardware mode** (real robot): run `robin_main.launch.py` from `robin_core_bringup` with `robot_ip:=<ip>` first, then launch `welding_robin_demo.launch.py` in a second terminal inside the container.

---

## Step 3 — Open the dashboard

```
http://localhost:5174
```

---

## Step 4 — Watch the intent stream

In a second terminal inside the vulcanexus container:
```bash
docker exec -it vulcanexus-bridge bash
source /workspace/ros2_packages/install/setup.bash
ros2 topic echo /intents
```

---

## Step 5 — Trigger and verify each intent

| UI action | Expected intent on `/intents` |
|-----------|-------------------------------|
| **Start** button → Pre-Start modal → **Proceed and Start** | `START_PROCESS` |
| Deviation detected → **Manual Adjust** → fill values → **Apply and Continue** | `MANUAL_ADJUST` |
| Deviation detected → **AI Recommend** → **Apply and Continue** | `REQUEST_AI_RECOMMENDATION` |
| **Abort** button (no modal) | `ESTOP` |
| Job report modal → **Run New DOE** | `LAUNCH_NEW_DOE` |

`ros2 topic echo /intents` should print something like:
```
intent: 'START_PROCESS'
data: '{"seam_id": "seam_01", "weld_speed": 5.0, "wire_feed": 4.0}'
source: 2
modality: 1
```

---

## Manual curl tests (without the UI)

Direct to welding_http_bridge (port 8766):
```bash
# START_PROCESS
curl -s -X POST http://localhost:8766/intent \
     -H 'Content-Type: application/json' \
     -d '{"intent": "START_PROCESS", "data": {"seam_id": "seam_01"}}' | jq

# ESTOP
curl -s -X POST http://localhost:8766/intent \
     -H 'Content-Type: application/json' \
     -d '{"intent": "ESTOP", "data": {"reason": "test"}}' | jq

# MANUAL_ADJUST
curl -s -X POST http://localhost:8766/intent \
     -H 'Content-Type: application/json' \
     -d '{"intent": "MANUAL_ADJUST", "data": {"parameter_name": "weld_speed", "new_value": 6.0, "unit": "mm/s"}}' | jq
```

Via alert-processor (full production path, requires Orion-LD):
```bash
curl -s -X POST http://localhost:8001/intent \
     -H 'Content-Type: application/json' \
     -d '{"intent": "START_PROCESS", "process_id": "seam_01", "data": {"seam_id": "seam_01"}}' | jq
```

---

## Troubleshooting

| Symptom | Check |
|---------|-------|
| `/intents` receives nothing after button click | `docker logs vulcanexus-bridge` — is `welding_http_bridge` running? Is port 8766 reachable from host? |
| `POST /intent` → 500 from alert-processor | Orion-LD not ready — check `docker logs fiware-orion` |
| Skill action servers not found | `ros2 action list` inside container — did the build succeed and `install/setup.bash` sourced? |
| Dashboard shows "API unreachable" | `docker logs robin-alert-processor` — check port 8001 |
| Supervisor not routing intents | Wait an extra 5 s; supervisor delays 2 s after launch for action servers to register |
