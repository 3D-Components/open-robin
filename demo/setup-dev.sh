#!/usr/bin/env bash
# One-time developer setup: link rosbag data and build ROS2 workspace.
# Run this once before using simulation-demo-rosbag.sh for the first time.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DATA_DIR="${REPO_ROOT}/data/rosbags"
BAG="${DATA_DIR}/exp001_rosbag_real"
CONTAINER="vulcanexus-bridge"

echo "============================================"
echo "  open-robin dev setup"
echo "============================================"

# 1. Verify rosbag is present
echo "Checking rosbag..."
if [ ! -d "${BAG}" ]; then
  echo "[ERROR] Rosbag not found at: ${BAG}"
  echo "  Place the exp001_rosbag_real directory inside data/rosbags/ in the repo."
  exit 1
fi
echo "[OK] Rosbag found at ${BAG}"

# 2. Build ROS2 workspace if container is running
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  if docker exec "${CONTAINER}" test -d /workspace/ros2_packages/install/robin_core_data 2>/dev/null; then
    echo "[OK] ROS2 workspace already built."
  else
    echo "Building ROS2 workspace (robin_interfaces + robin_core_data)..."
    docker exec "${CONTAINER}" bash -lc "
      source /opt/ros/jazzy/setup.bash &&
      source /opt/vulcanexus/jazzy/setup.bash &&
      cd /workspace/ros2_packages &&
      colcon build --symlink-install \
        --packages-select robin_interfaces robin_core_data \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    "
    echo "[OK] Workspace built."
  fi
else
  echo "[SKIP] vulcanexus container not running."
  echo "  Start it first, then re-run this script:"
  echo "    UID=\$(id -u) GID=\$(id -g) docker compose up -d vulcanexus"
  echo "    bash demo/setup-dev.sh"
fi

echo
echo "============================================"
echo "  Setup complete."
echo "  Next: bash demo/simulation-demo-rosbag.sh"
echo "============================================"
