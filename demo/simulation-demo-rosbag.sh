#!/bin/bash
# Replay a recorded ROS bag and stream telemetry into Orion-LD using DDS only.

set -euo pipefail

PROCESS_ID="ros_bridge"
ENTITY_ID="urn:ngsi-ld:Process:${PROCESS_ID}"
ORION_URL="${ORION_URL:-http://127.0.0.1:1026}"
BAG_NAME="exp001_rosbag_real"
BAG_HOST_PATH="Data_ROSBAGS/${BAG_NAME}"
BAG_CONTAINER_PATH="/workspace/ros2_packages/${BAG_NAME}"
CONTAINER="vulcanexus-bridge"

echo "============================================"
echo "  ROS Bag -> DDS -> Orion-LD -> Dashboard"
echo "  Mode: DDS only"
echo "============================================"
echo "Process ID          : ${PROCESS_ID}"
echo "Entity ID           : ${ENTITY_ID}"
echo "Orion URL           : ${ORION_URL}"
echo "ROS bag (host)      : ${BAG_HOST_PATH}"
echo "Container           : ${CONTAINER}"
echo

# 1) Ensure required containers are running
if ! docker ps --format '{{.Names}}' | grep -q "^fiware-orion$"; then
  echo "Starting FIWARE stack..."
  docker compose up -d orion-ld mongo-db timescaledb mintaka alert-processor
  echo "Waiting for Orion to be ready..."
  sleep 10
fi

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "Starting vulcanexus container..."
  UID=$(id -u) GID=$(id -g) docker compose up -d vulcanexus
  sleep 3
fi

# 2) Setup DDS temporal support (DB trigger for observedAt timestamps)
echo "Setting up DDS temporal support for Mintaka..."
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
docker exec fiware-timescaledb psql -U orion -d orion -q -c "DROP TRIGGER IF EXISTS trigger_set_observedat ON attributes;" 2>/dev/null || true
docker exec fiware-timescaledb psql -U orion -d orion -q -c "CREATE TRIGGER trigger_set_observedat BEFORE INSERT ON attributes FOR EACH ROW EXECUTE FUNCTION set_observedat_ros();" 2>/dev/null || true
echo "DDS temporal support ready."

# 3) Copy bag into container if missing
if ! docker exec "${CONTAINER}" test -d "${BAG_CONTAINER_PATH}"; then
  echo "Copying bag into container..."
  docker cp "${BAG_HOST_PATH}" "${CONTAINER}:${BAG_CONTAINER_PATH}"
fi

# 4) Start DDS telemetry aggregator
echo
echo "Starting telemetry aggregator (raw topics -> /robin/telemetry)..."
docker exec -d "${CONTAINER}" bash -lc "\
  export ROS_DOMAIN_ID=0 && \
  cd /workspace/ros2_packages && source ws_setup.sh && \
  ros2 run robin_core_data telemetry_aggregator_node.py --ros-args \
    -p geometry_topic:=/robin/weld_dimensions \
    -p fronius_topic:=/robin/data/fronius \
    -p pose_topic:=/tcp_pose_broadcaster/pose \
    -p output_topic:=/robin/telemetry \
    -p min_publish_period:=0.1"
echo "Telemetry aggregator started."
sleep 2

echo
echo "============================================"
echo "  DDS mode active (via Orion -wip dds)"
echo "  Topic configured in config-dds.json:"
echo "    - rt/robin/telemetry -> urn:robin:processTelemetry"
echo "============================================"
echo
echo "Dashboard: http://localhost:5174"
echo "Process ID: ${PROCESS_ID}"
echo
echo "Check measurements:"
echo "  curl -s 'http://localhost:8001/process/${PROCESS_ID}/measurements?last=5' | jq"
echo
echo "Check entity directly:"
echo "  curl -s 'http://localhost:1026/ngsi-ld/v1/entities/${ENTITY_ID}' | jq"
echo

# 5) Play bag in foreground
echo "Playing bag at 0.1x speed (~10Hz for DDS bridge)..."
echo "Bag duration: ~72 seconds real-time -> ~12 minutes at 0.1x"
docker exec -it "${CONTAINER}" bash -lc "\
  export ROS_DOMAIN_ID=0 && \
  cd /workspace/ros2_packages && source ws_setup.sh && \
  ros2 bag play ${BAG_CONTAINER_PATH} -r 0.1"

echo
echo "Playback stopped."
echo "Stopping telemetry aggregator..."
docker exec "${CONTAINER}" pkill -f telemetry_aggregator_node.py 2>/dev/null || true
