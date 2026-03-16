#!/bin/bash
# End-to-end pipeline test for the 2026-03-16 rosbag.
#
# Plays the bag ONCE (no loop) and measures message counts at every stage:
#   ROS2 bag -> telemetry_aggregator -> DDS bridge -> Orion-LD
#   -> TimescaleDB (TROE) -> Mintaka -> Alert Engine -> Dashboard
#
# Ground truth from bag metadata:
#   /robin/data/fronius       906  msgs  (WelderData)
#   /robin/weld_dimensions   6046  msgs  (BeadGeometry)
#   Duration                  290.6 s
#
# Expected aggregator output: ~290 msgs at 1 Hz (min_publish_period=1.0)
# Expected TimescaleDB rows:  ~290  (BEST_EFFORT DDS, <10% loss acceptable)
# Expected Mintaka entries:   same as TimescaleDB (TROE 1:1)
# Expected Alert Engine:      same as Mintaka

set -euo pipefail

BAG_NAME="bag_2026-03-16"
BAG_CONTAINER_PATH="/workspace/ros2_packages/${BAG_NAME}"
CONTAINER="vulcanexus-bridge"
PROCESS_ID="ros_bridge"
ENTITY_ID="urn:ngsi-ld:Process:${PROCESS_ID}"
ORION_URL="${ORION_URL:-http://127.0.0.1:1026}"
MINTAKA_URL="${MINTAKA_URL:-http://127.0.0.1:9090}"
ALERT_ENGINE_URL="${ALERT_ENGINE_URL:-http://127.0.0.1:8001}"

# Ground truth from bag metadata
BAG_FRONIUS_EXPECTED=906
BAG_GEOMETRY_EXPECTED=6046
BAG_DURATION_SECS=291

AGGREGATOR_ARGS="--ros-args \
    -p geometry_topic:=/robin/weld_dimensions \
    -p fronius_topic:=/robin/data/fronius \
    -p output_topic:=/robin/telemetry \
    -p min_publish_period:=1.0"

echo "============================================================"
echo "  ROS Bag -> DDS -> Orion-LD -> Dashboard  [E2E TEST]"
echo "  Bag: ${BAG_NAME}  (~${BAG_DURATION_SECS}s, one-shot)"
echo "============================================================"
echo "Entity  : ${ENTITY_ID}"
echo "Orion   : ${ORION_URL}"
echo "Mintaka : ${MINTAKA_URL}"
echo "Alert   : ${ALERT_ENGINE_URL}"
echo

# ---- 1. Ensure required containers are running --------------------------
echo "[1/9] Checking containers..."
for svc in fiware-orion fiware-timescaledb fiware-mintaka robin-alert-processor; do
  if ! docker ps --format '{{.Names}}' | grep -q "^${svc}$"; then
    echo "  Starting FIWARE stack..."
    docker compose -f "$(dirname "$0")/../docker-compose.yaml" up -d \
      orion-ld mongo-db timescaledb mintaka alert-processor
    echo "  Waiting 10s for services..."
    sleep 10
    break
  fi
done

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "  Starting vulcanexus container..."
  UID=$(id -u) GID=$(id -g) docker compose -f "$(dirname "$0")/../docker-compose.yaml" \
    up -d vulcanexus
  sleep 5
fi

# Restart Orion-LD to reload DDS config (picks up config-dds.json allowlist)
echo "  Reloading Orion-LD DDS config..."
docker restart fiware-orion
echo "  Waiting 10s for Orion-LD restart..."
sleep 10
echo "  Containers ready."

# ---- 2. Verify bag is accessible inside container -----------------------
echo
echo "[2/9] Verifying bag mount at ${BAG_CONTAINER_PATH}..."
if ! docker exec "${CONTAINER}" test -d "${BAG_CONTAINER_PATH}"; then
  echo "ERROR: Bag not found at ${BAG_CONTAINER_PATH} inside container."
  echo "  Ensure docker-compose.yaml has:"
  echo "    - ./data/rosbags/bag_2026-03-16:/workspace/ros2_packages/bag_2026-03-16:ro"
  echo "  Then run: UID=\$(id -u) GID=\$(id -g) docker compose up -d --force-recreate vulcanexus"
  exit 1
fi
echo "  Bag accessible."

# ---- 3. Setup TimescaleDB trigger for ROS timestamps --------------------
echo
echo "[3/9] Setting up DDS temporal support (TimescaleDB trigger)..."
docker exec fiware-timescaledb psql -U orion -d orion -q -c "
CREATE OR REPLACE FUNCTION set_observedat_ros()
RETURNS TRIGGER AS \$\$
BEGIN
    IF NEW.observedat IS NULL THEN
        IF NEW.compound IS NOT NULL
           AND NEW.compound ? 'header'
           AND (NEW.compound->'header') ? 'stamp'
           AND (NEW.compound->'header'->'stamp') ? 'sec' THEN
            NEW.observedat := to_timestamp(
                (NEW.compound->'header'->'stamp'->>'sec')::double precision +
                COALESCE((NEW.compound->'header'->'stamp'->>'nanosec')::double precision, 0) / 1000000000.0
            );
        ELSE
            NEW.observedat := NEW.ts;
        END IF;
    END IF;
    RETURN NEW;
END;
\$\$ LANGUAGE plpgsql;
" 2>/dev/null || true
docker exec fiware-timescaledb psql -U orion -d orion -q -c \
  "DROP TRIGGER IF EXISTS trigger_set_observedat ON attributes;" 2>/dev/null || true
docker exec fiware-timescaledb psql -U orion -d orion -q -c \
  "CREATE TRIGGER trigger_set_observedat BEFORE INSERT ON attributes FOR EACH ROW EXECUTE FUNCTION set_observedat_ros();" 2>/dev/null || true
echo "  Trigger ready."

# ---- 4. Baseline counts -------------------------------------------------
echo
echo "[4/9] Recording baseline counts..."

BASELINE_TS=$(docker exec fiware-timescaledb psql -U orion -d orion -tAq -c \
  "SELECT COUNT(*) FROM attributes
   WHERE entityid='${ENTITY_ID}' AND id='urn:robin:processTelemetry';" 2>/dev/null || echo 0)
BASELINE_TS="${BASELINE_TS:-0}"
echo "  TimescaleDB baseline: ${BASELINE_TS} rows"

# ---- 5. Start telemetry aggregator --------------------------------------
echo
echo "[5/9] Starting telemetry aggregator (background)..."
docker exec "${CONTAINER}" bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /workspace/ros2_packages/install/setup.bash &&
  export ROS_DOMAIN_ID=0 &&
  nohup ros2 run robin_core_data telemetry_aggregator_node.py \
    ${AGGREGATOR_ARGS} > /tmp/aggregator.log 2>&1 &
  echo \$!
" 2>/dev/null
echo "  Aggregator started (log: /tmp/aggregator.log inside container)"
sleep 3

# ---- 6. Start telemetry counter -----------------------------------------
echo
echo "[6/9] Starting /robin/telemetry message counter..."
docker exec "${CONTAINER}" bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /workspace/ros2_packages/install/setup.bash &&
  export ROS_DOMAIN_ID=0 &&
  rm -f /tmp/telemetry_count.txt &&
  python3 -c \"
import rclpy, signal, sys
from rclpy.node import Node
from robin_interfaces.msg import ProcessTelemetry

class Counter(Node):
    def __init__(self):
        super().__init__('telemetry_counter_e2e')
        self.count = 0
        self.create_subscription(ProcessTelemetry, '/robin/telemetry', self.cb, 10)
    def cb(self, _):
        self.count += 1
        open('/tmp/telemetry_count.txt','w').write(str(self.count))

rclpy.init()
n = Counter()
rclpy.spin(n)
\" > /tmp/telemetry_counter.log 2>&1 &
  echo \$! > /tmp/telemetry_counter.pid
  echo 'Counter started'
" 2>/dev/null
sleep 2

# ---- 7. Play bag once ---------------------------------------------------
echo
echo "[7/9] Playing bag once at 1x speed (~${BAG_DURATION_SECS}s — please wait)..."
PLAY_START=$(date +%s)
docker exec "${CONTAINER}" bash -lc "
  export ROS_DOMAIN_ID=0 &&
  cd /workspace/ros2_packages && source ws_setup.sh &&
  ros2 bag play ${BAG_CONTAINER_PATH} \
    --topics /robin/data/fronius /robin/weld_dimensions"
PLAY_END=$(date +%s)
ELAPSED=$(( PLAY_END - PLAY_START ))
echo "  Bag playback done in ${ELAPSED}s"

# ---- 8. Wait for pipeline to flush --------------------------------------
echo
echo "[8/9] Waiting 20s for DDS -> Orion -> TimescaleDB flush..."
sleep 20

# Stop aggregator and counter
docker exec "${CONTAINER}" pkill -f telemetry_aggregator_node.py 2>/dev/null || true
COUNTER_PID=$(docker exec "${CONTAINER}" cat /tmp/telemetry_counter.pid 2>/dev/null || echo "")
[ -n "${COUNTER_PID}" ] && docker exec "${CONTAINER}" kill "${COUNTER_PID}" 2>/dev/null || true
sleep 2

# ---- 9. Collect counts and print table ----------------------------------
echo
echo "[9/9] Collecting counts from each pipeline stage..."

# Aggregator publishes (from log)
AGG_PUBLISHED=$(docker exec "${CONTAINER}" \
  grep -c '\[AGG\] Published telemetry' /tmp/aggregator.log 2>/dev/null || echo 0)

# Counter received
TELEMETRY_RECEIVED=$(docker exec "${CONTAINER}" \
  cat /tmp/telemetry_count.txt 2>/dev/null || echo 0)
TELEMETRY_RECEIVED="${TELEMETRY_RECEIVED:-0}"

# TimescaleDB new rows
TOTAL_TS=$(docker exec fiware-timescaledb psql -U orion -d orion -tAq -c \
  "SELECT COUNT(*) FROM attributes
   WHERE entityid='${ENTITY_ID}' AND id='urn:robin:processTelemetry';" 2>/dev/null || echo "${BASELINE_TS}")
TOTAL_TS="${TOTAL_TS:-${BASELINE_TS}}"
NEW_TS=$(( TOTAL_TS - BASELINE_TS ))

# Mintaka entries
MINTAKA_JSON=$(curl -s --max-time 30 \
  "${MINTAKA_URL}/temporal/entities/${ENTITY_ID}" \
  -G \
  --data-urlencode "attrs=urn:robin:processTelemetry" \
  --data-urlencode "timerel=between" \
  --data-urlencode "timeAt=1970-01-01T00:00:00Z" \
  --data-urlencode "endTimeAt=2030-01-01T00:00:00Z" \
  --data-urlencode "options=temporalValues" \
  --data-urlencode "lastN=50000" 2>/dev/null || echo '{}')
MINTAKA_COUNT=$(echo "${MINTAKA_JSON}" | \
  python3 -c "import sys,json; d=json.load(sys.stdin); \
    vals=d.get('urn:robin:processTelemetry',{}).get('values',[]); \
    print(len(vals))" 2>/dev/null || echo "error")

# Alert Engine count (no lastN to get full count)
AE_JSON=$(curl -s --max-time 30 \
  "${ALERT_ENGINE_URL}/process/${PROCESS_ID}/measurements" 2>/dev/null || echo '{}')
AE_COUNT=$(echo "${AE_JSON}" | \
  python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('count',0))" 2>/dev/null || echo "error")
AE_SOURCE=$(echo "${AE_JSON}" | \
  python3 -c "import sys,json; d=json.load(sys.stdin); \
    print(d.get('debug_info',{}).get('source','unknown'))" 2>/dev/null || echo "unknown")

EXPECTED_AGG="${BAG_DURATION_SECS} (1 Hz x ${BAG_DURATION_SECS}s)"

echo
echo "=================================================================="
echo "  END-TO-END PIPELINE MESSAGE COUNT REPORT"
echo "=================================================================="
printf "  %-38s %s\n" "Bag playback duration" "${ELAPSED}s (bag: ${BAG_DURATION_SECS}s)"
echo "  ------------------------------------------------------------------"
printf "  %-38s %s\n" "Bag /robin/data/fronius (ground truth)" "${BAG_FRONIUS_EXPECTED}"
printf "  %-38s %s\n" "Bag /robin/weld_dimensions (ground truth)" "${BAG_GEOMETRY_EXPECTED}"
echo "  ------------------------------------------------------------------"
printf "  %-38s %s\n" "Aggregator published (/robin/telemetry)" "${AGG_PUBLISHED}  (expected ~${BAG_DURATION_SECS})"
printf "  %-38s %s\n" "Counter received (/robin/telemetry)" "${TELEMETRY_RECEIVED}"
echo "  ------------------------------------------------------------------"
printf "  %-38s %s\n" "TimescaleDB new rows (TROE)" "${NEW_TS}  (baseline was ${BASELINE_TS})"
printf "  %-38s %s\n" "Mintaka temporal entries" "${MINTAKA_COUNT}"
printf "  %-38s %s\n" "Alert Engine count (source: ${AE_SOURCE})" "${AE_COUNT}"
echo "=================================================================="
echo
echo "LOSS ANALYSIS:"
if [ "${AGG_PUBLISHED}" -ge 1 ] 2>/dev/null; then
  if [ "${NEW_TS}" -ge 1 ] 2>/dev/null; then
    LOSS_PCT=$(python3 -c "print(f'{100*(1-${NEW_TS}/${AGG_PUBLISHED}):.1f}%')" 2>/dev/null || echo "?")
    echo "  DDS loss (aggregator→TimescaleDB): ${LOSS_PCT}  [${NEW_TS}/${AGG_PUBLISHED}]"
  fi
fi
echo
echo "DIAGNOSTICS:"
echo "  Orion entity    : curl -s '${ORION_URL}/ngsi-ld/v1/entities/${ENTITY_ID}' | jq"
echo "  Aggregator log  : docker exec ${CONTAINER} tail -20 /tmp/aggregator.log"
echo "  Measurements API: curl -s '${ALERT_ENGINE_URL}/process/${PROCESS_ID}/measurements?last=5' | jq"
echo "  Dashboard       : http://localhost:5174  (process: ${PROCESS_ID})"
echo
echo "SANITY PLOTS (run separately):"
echo "  TimescaleDB : python3 scripts/timescale_plot_sanity.py ${PROCESS_ID}"
echo "  Mintaka     : MINTAKA_URL=http://localhost:9090 python3 scripts/mintaka_plot_sanity.py ${PROCESS_ID}"
echo
