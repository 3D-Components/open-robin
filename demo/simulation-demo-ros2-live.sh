#!/bin/bash
# ROS2-live welding demo: the dashboard intent buttons drive a simulated UR10e
# while synthetic telemetry streams ROS 2 -> DDS -> FIWARE (Orion-LD) and back to
# the dashboard.
#
#   Start  -> the arm welds the bead; telemetry flows to FIWARE
#   Pause  -> the arm freezes in place, telemetry stops
#   Resume -> the weld continues from where it froze
#   Stop   -> the arm returns to its home position
#   Abort  -> the arm returns home and an error appears in the Alerts panel
#
# Modes:
#   (default)        "lite" sim: pure-Python mock skills + robot_state_publisher +
#                    foxglove. No Gazebo, MoveIt, controllers, or GPU. Starts in
#                    seconds and is the recommended demo.
#   --gazebo         faithful sim: Gazebo + MoveIt (heavy, needs GPU; see notes).
#   --legacy-python  pure-Python telemetry path (demo/profiles/welding_profile.py).

set -euo pipefail

MODE="lite"
case "${1:-}" in
  --legacy-python)
    shift
    echo "Delegating to the legacy pure-Python welding demo..."
    exec python demo/profiles/welding_profile.py "$@"
    ;;
  --gazebo) MODE="gazebo"; shift ;;
  --lite)   MODE="lite";   shift ;;
esac

PROCESS_ID="${PROCESS_ID:-ros_bridge}"
ENTITY_ID="urn:ngsi-ld:Process:${PROCESS_ID}"
ORION_URL="${ORION_URL:-http://127.0.0.1:1026}"
API_URL="${API_URL:-http://127.0.0.1:8001}"
CONTAINER="vulcanexus-bridge"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"   # must match orion-ld + config-dds.json (0)

if [[ "${MODE}" == "gazebo" ]]; then
  BUILD_PKGS="robin_bringup robin_sim_gz welding_demo welding_supervisor"
  LAUNCH_CMD="ros2 launch robin_bringup robin_main.launch.py use_sim:=true gazebo_gui:=false launch_rviz:=false launch_foxglove:=true"
  MODE_DESC="faithful Gazebo + MoveIt"
else
  # robin_bringup brings the full cell description (UR10e + Fronius torch + Garmo
  # profilometer + welding table + platform). --packages-up-to pulls its deps
  # (robin_core, robin_hardware_*) as one-time ament_python/cmake installs; the lite
  # runtime never launches those nodes (no MoveIt/Gazebo) — it only needs the urdf+meshes.
  BUILD_PKGS="welding_demo welding_supervisor robin_bringup robin_hardware_ur"
  LAUNCH_CMD="ros2 launch welding_demo welding_robin_sim.launch.py"
  MODE_DESC="lite mock (no Gazebo/MoveIt)"
fi

echo "================================================================"
echo "  ROS2-live welding demo  (${MODE_DESC}, no hardware)"
echo "  Intent-driven arm + synthetic telemetry ROS2 -> DDS -> FIWARE"
echo "================================================================"
echo "Mode          : ${MODE}"
echo "Process ID    : ${PROCESS_ID}"
echo "Entity ID     : ${ENTITY_ID}"
echo "ROS_DOMAIN_ID : ${DOMAIN_ID}  (must match orion-ld and config-dds.json)"
echo "Container     : ${CONTAINER}"
echo

# 1) Bring up the FIWARE stack + dashboard + ROS 2 container
if ! docker ps --format '{{.Names}}' | grep -q "^fiware-orion$"; then
  echo "Starting FIWARE stack + dashboard..."
  docker compose up -d orion-ld mongo-db timescaledb mintaka alert-processor robin-dashboard lichtblick
  echo "Waiting for Orion-LD to be ready..."
  sleep 10
fi
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "Starting vulcanexus container..."
  # NOTE: UID is a readonly bash builtin (already the current uid) — export it,
  # never assign to it. Only GID needs to be set for compose's ${UID}:${GID} user.
  export GID="$(id -g)"
  export UID
  docker compose up -d vulcanexus
  sleep 3
fi

# 1b) Reload Orion-LD so it re-reads config-dds.json (DDS allowlist + mapping)
echo "Reloading Orion-LD DDS config..."
docker restart fiware-orion
sleep 10

# 2) DDS temporal support: derive observedAt from the ROS header stamp so
#    Mintaka serves the synthetic telemetry as a time series.
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

# 3) Build the ROS 2 workspace if needed
if ! docker exec "${CONTAINER}" test -f /workspace/ros2_packages/install/setup.bash 2>/dev/null; then
  echo "ROS 2 workspace not built — building packages (${BUILD_PKGS})..."
  docker exec --user root "${CONTAINER}" \
    chown -R "$(id -u):$(id -g)" /workspace/ros2_packages 2>/dev/null || true
  docker exec "${CONTAINER}" bash -lc "
    source /opt/ros/jazzy/setup.bash &&
    source /opt/vulcanexus/jazzy/setup.bash &&
    cd /workspace/ros2_packages &&
    colcon build --symlink-install \
      --packages-up-to ${BUILD_PKGS} \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  "
  echo "Workspace build complete."
fi

# 3b) Lite mode: ensure the scene description (torch + profilometer + table) is built
#     even on a workspace that was built before robin_bringup was added to the lite
#     set. --packages-up-to robin_bringup is incremental (~10 s cold, ~1 s if current).
if [[ "${MODE}" != "gazebo" ]] && \
   ! docker exec "${CONTAINER}" test -f \
     /workspace/ros2_packages/install/robin_bringup/share/robin_bringup/urdf/ur_fronius_garmo.urdf.xacro 2>/dev/null; then
  echo "Building the cell description (torch + profilometer + table)..."
  docker exec "${CONTAINER}" bash -lc "
    source /opt/ros/jazzy/setup.bash &&
    source /opt/vulcanexus/jazzy/setup.bash &&
    cd /workspace/ros2_packages &&
    colcon build --symlink-install \
      --packages-up-to robin_bringup \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  "
  echo "Cell description built."
fi

# 4) Seed the ros_bridge Process entity so the dashboard has a process to select
#    and deviation checks have a target. Telemetry from the DDS path updates the
#    same entity once the weld starts.
echo "Seeding the ${PROCESS_ID} process entity..."
curl -s -X POST "${API_URL}/create-process" \
  -H 'Content-Type: application/json' \
  -d "{\"process_id\": \"${PROCESS_ID}\", \"mode\": \"parameter_driven\"}" >/dev/null || true
curl -s -X POST "${API_URL}/process/${PROCESS_ID}/target" \
  -H 'Content-Type: application/json' \
  -d '{"height": 3.0, "width": 7.0}' >/dev/null || true
echo "Process ${PROCESS_ID} ready."

echo
echo "================================================================"
echo "  Open the dashboard and drive the demo with the buttons:"
echo "================================================================"
echo "  Dashboard : http://localhost:5174   (select process '${PROCESS_ID}')"
echo "  3D view   : Lichtblick http://localhost:8080  (ws://localhost:8765)"
echo
echo "  Start  -> the UR10e welds the bead; telemetry -> FIWARE"
echo "  Pause  -> arm freezes in place, telemetry stops"
echo "  Resume -> weld continues"
echo "  Stop   -> arm returns to home"
echo "  Abort  -> arm returns home + error in the Alerts panel"
echo
echo "  Inspect telemetry (second terminal):"
echo "    docker exec ${CONTAINER} bash -lc 'source /workspace/ros2_packages/install/setup.bash && ros2 topic echo /robin/telemetry'"
echo "    curl -s '${API_URL}/process/${PROCESS_ID}/measurements?last=5' | jq"
echo "    curl -s '${ORION_URL}/ngsi-ld/v1/entities/${ENTITY_ID}' | jq"
echo
if [[ "${MODE}" == "gazebo" ]]; then
  echo "NOTE (gazebo mode): the Garmo lidar needs GPU rendering for geometry telemetry."
  echo "  If you see 'libEGL ... Permission denied', recreate the container with the GPU override:"
  echo "    docker compose -f docker-compose.yaml -f docker-compose.linux-gpu.override.yaml up -d vulcanexus"
  echo "  First start takes ~15 s while Gazebo and MoveIt come up."
fi
echo "Launching the ${MODE_DESC} stack (headless). Ctrl+C to stop."
echo

# 4b) Clear any orphaned nodes from a previous run. A killed or frozen session can
#     leave foxglove_bridge (8765), welding_http_bridge (8766), and the skill nodes
#     alive inside the container, which then causes "address already in use" bind
#     errors and duplicate action servers on the shared ROS_DOMAIN_ID. pkill runs in
#     the container's PID namespace, so it only touches ${CONTAINER}.
#     NOTE: the patterns are bracketed (e.g. [w]elding_) so pkill does not match the
#     command line of this very cleanup shell — a plain "welding_" pattern would kill
#     the shell mid-cleanup, leaving the remaining processes alive.
echo "Clearing any orphaned demo processes in ${CONTAINER}..."
docker exec "${CONTAINER}" bash -lc '
  pkill -f "[r]os2 launch"          2>/dev/null
  pkill -f "[w]elding_"             2>/dev/null
  pkill -f "[f]oxglove_bridge"      2>/dev/null
  pkill -f "[r]obot_state_publisher" 2>/dev/null
  pkill -f "[u]r_rsp"               2>/dev/null
  sleep 1
' || true

# 5) Launch the stack in the foreground (Ctrl+C tears it down).
docker exec -it "${CONTAINER}" bash -lc "
  export ROS_DOMAIN_ID=${DOMAIN_ID}
  cd /workspace/ros2_packages && source ws_setup.sh
  ${LAUNCH_CMD}
"

echo
echo "Stack stopped."
