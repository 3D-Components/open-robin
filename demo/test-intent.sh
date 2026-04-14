#!/usr/bin/env bash
# test-intent.sh — fire every intent type through the full OrionLD path
#
# Intent delivery path (Path A — production):
#   This script → POST /intent on Alert Engine (port 8001)
#                 → PATCH pendingIntent on OrionLD entity
#                 → Orion subscription notification
#                 → POST /orion-notify on welding_http_bridge (port 8766)
#                 → publish welding_msgs/Intent on /intents ROS2 topic
#                 → welding_supervisor → skill action server
#
# Usage:
#   bash demo/test-intent.sh                     # uses process "ros_bridge"
#   bash demo/test-intent.sh weld-1234567890     # use a specific process ID
#   ALERT_ENGINE_URL=http://localhost:8001 bash demo/test-intent.sh
#
# Prerequisites:
#   - Docker stack running: docker compose up -d
#   - ROS2 workspace launched: ros2 launch welding_demo welding_robin_demo.launch.py
#   - Subscriber running in another terminal:
#       ros2 topic echo /intents welding_msgs/msg/Intent
#   - Orion watcher (optional):
#       bash demo/watch-intents.sh

set -euo pipefail

API=${ALERT_ENGINE_URL:-http://localhost:8001}
PROCESS=${1:-ros_bridge}
PAUSE=${INTENT_PAUSE:-2}   # seconds between intents

echo "============================================================"
echo "  Intent test — Path A (Alert Engine → OrionLD → Bridge → ROS2)"
echo "  Alert Engine : $API"
echo "  Process ID   : $PROCESS"
echo "  Pause        : ${PAUSE}s between intents"
echo "============================================================"
echo ""

fire() {
  local intent="$1"
  local data="$2"
  echo "──────────────────────────────────────────"
  echo "  → $intent"
  echo "  data: $data"
  result=$(curl -s -X POST "$API/intent" \
    -H 'Content-Type: application/json' \
    -d "{\"intent\":\"$intent\",\"process_id\":\"$PROCESS\",\"data\":$data}")
  echo "  ← $(echo "$result" | python3 -m json.tool 2>/dev/null || echo "$result")"
  echo ""
  sleep "$PAUSE"
}

# --- motion intents ---
fire "MOVE_TO_HOME"              '{"fast": false}'
fire "MOVE_TO_ZONE"              '{"zone_id": "B", "approach_speed": 0.5}'

# --- seam / process intents ---
fire "START_PROCESS"             '{"seam_id": "seam-01", "weld_speed": 5.0, "wire_feed_rate": 4.2}'
fire "EXECUTE_SEAM"              '{"seam_id": "seam-02", "weld_speed": 4.8, "wire_feed_rate": 4.0}'

# --- AI / parameter intents ---
fire "REQUEST_AI_RECOMMENDATION" '{"mode": "parameter_driven"}'
fire "MANUAL_ADJUST"             '{"parameter_name": "current", "new_value": 180.0, "unit": "A"}'
fire "FINE_TUNE_MODEL"           "{\"dataset_tag\": \"run-$(date +%s)\"}"

# --- safety ---
fire "ESTOP"                     '{}'

echo "============================================================"
echo "  All intents fired."
echo "  Check:"
echo "    T3: ros2 topic echo /intents — each Intent msg"
echo "    T4: demo/watch-intents.sh   — OrionLD pendingIntent updates"
echo "============================================================"
