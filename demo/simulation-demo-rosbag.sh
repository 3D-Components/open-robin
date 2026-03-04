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

# 1c) Restart Orion-LD to reload DDS config (picks up config-dds.json allowlist)
echo "Reloading Orion-LD DDS config..."
docker restart fiware-orion
echo "Waiting for Orion-LD to restart..."
sleep 10

# 1b) Build ROS2 workspace if not already built
if ! docker exec "${CONTAINER}" test -d /workspace/ros2_packages/install/robin_core_data 2>/dev/null; then
  echo "ROS2 workspace not built — building packages (includes mesh assets for Lichtblick)..."
  docker exec "${CONTAINER}" bash -lc "
    source /opt/ros/jazzy/setup.bash &&
    source /opt/vulcanexus/jazzy/setup.bash &&
    cd /workspace/ros2_packages &&
    colcon build --symlink-install \
      --packages-up-to robin_core_bringup robin_core_data \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  "
  echo "Workspace build complete."
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

# 3) Verify bag is accessible inside container (mounted via docker-compose volume)
if ! docker exec "${CONTAINER}" test -f "${BAG_CONTAINER_PATH}/metadata.yaml"; then
  echo "ERROR: Rosbag not found at ${BAG_CONTAINER_PATH} inside container."
  echo "  The rosbag is mounted as a read-only volume via docker-compose.yaml."
  echo "  Recreate the container to pick up the mount:"
  echo "    UID=\$(id -u) GID=\$(id -g) docker compose up -d --force-recreate vulcanexus"
  exit 1
fi
echo "Rosbag accessible at ${BAG_CONTAINER_PATH}."

# 4) Start DDS telemetry aggregator
echo
echo "Smoke-testing telemetry aggregator (3s foreground run)..."
AGGREGATOR_ARGS="--ros-args \
    -p geometry_topic:=/robin/weld_dimensions \
    -p fronius_topic:=/robin/data/fronius \
    -p pose_topic:=/tcp_pose_broadcaster/pose \
    -p output_topic:=/robin/telemetry \
    -p min_publish_period:=1.0"
docker exec "${CONTAINER}" bash -lc "
  export ROS_DOMAIN_ID=10
  cd /workspace/ros2_packages && source ws_setup.sh
  timeout 3 ros2 run robin_core_data telemetry_aggregator_node.py ${AGGREGATOR_ARGS}
" 2>&1 | tail -5 || true

echo "Starting telemetry aggregator in background..."
docker exec "${CONTAINER}" bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /workspace/ros2_packages/install/setup.bash &&
  export ROS_DOMAIN_ID=10 &&
  nohup ros2 run robin_core_data telemetry_aggregator_node.py \
    ${AGGREGATOR_ARGS} > /tmp/aggregator.log 2>&1 &
  echo \$!
"
echo "Telemetry aggregator started."
sleep 3

# 4b) Start foxglove-bridge (exposes all ROS2 topics to Lichtblick on port 8765)
echo "Starting foxglove-bridge on ws://localhost:8765..."
docker exec -d "${CONTAINER}" bash -c "\
  export ROS_DOMAIN_ID=10 && \
  source /opt/ros/jazzy/setup.bash && \
  source /opt/vulcanexus/jazzy/setup.bash && \
  source /workspace/ros2_packages/install/setup.bash && \
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=8765 address:=0.0.0.0 send_buffer_limit:=10000000"
echo "foxglove-bridge started."
sleep 2

# 4c) Start robot_state_publisher with the current on-disk URDF.
# The rosbag contains a recorded /robot_description and /tf_static from the time of recording,
# so bag playback would replay stale URDF/transforms and ignore any edits to the .xacro files.
# We remap those topics away from the bag (step 5) and publish fresh ones here instead.
echo "Starting robot_state_publisher with current URDF..."
docker exec -d "${CONTAINER}" bash -c "\
  export ROS_DOMAIN_ID=10 && \
  source /opt/ros/jazzy/setup.bash && \
  source /workspace/ros2_packages/install/setup.bash && \
  URDF_FILE=\$(ros2 pkg prefix --share robin_core_bringup)/urdf/ur_fronius_garmo.urdf.xacro && \
  ROBOT_DESC=\$(xacro \${URDF_FILE}) && \
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:=\"\${ROBOT_DESC}\""
echo "robot_state_publisher started."
sleep 2

echo
echo "============================================"
echo "  DDS mode active (via Orion -wip dds)"
echo "  Topic configured in config-dds.json:"
echo "    - rt/robin/telemetry -> urn:robin:processTelemetry"
echo "============================================"
echo
echo "Dashboard:  http://localhost:5174"
echo "  → Switch visualization panel to 'Lichtblick' tab"
echo "Lichtblick: http://localhost:8080"
echo "  → Open Connection → Rosbridge WebSocket → ws://localhost:8765"
echo "Process ID: ${PROCESS_ID}"
echo
echo "Check measurements:"
echo "  curl -s 'http://localhost:8001/process/${PROCESS_ID}/measurements?last=5' | jq"
echo
echo "Check entity directly:"
echo "  curl -s 'http://localhost:1026/ngsi-ld/v1/entities/${ENTITY_ID}' | jq"
echo

# 5) Play bag in foreground (loops continuously — Ctrl+C to stop)
# /robot_description and /tf_static are remapped to dummy topics so the stale bag-recorded
# values don't override the live robot_state_publisher started in step 4c.
echo "Playing bag on loop at 1x speed (72s per cycle — Ctrl+C to stop)..."
docker exec -it "${CONTAINER}" bash -lc "\
  export ROS_DOMAIN_ID=10 && \
  cd /workspace/ros2_packages && source ws_setup.sh && \
  ros2 bag play ${BAG_CONTAINER_PATH} --loop \
    --remap /robot_description:=/robot_description_from_bag \
    --remap /tf_static:=/tf_static_from_bag"

echo
echo "Playback stopped."
echo "Stopping telemetry aggregator, foxglove-bridge and robot_state_publisher..."
docker exec "${CONTAINER}" pkill -f telemetry_aggregator_node.py 2>/dev/null || true
docker exec "${CONTAINER}" pkill -f foxglove_bridge 2>/dev/null || true
docker exec "${CONTAINER}" pkill -f robot_state_publisher 2>/dev/null || true
