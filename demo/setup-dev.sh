#!/usr/bin/env bash
# One-time developer setup: link rosbag data and build ROS2 workspace.
# Run this once before using simulation-demo-rosbag.sh for the first time.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROBIN_BAG="/home/virgilio/exp001_rosbag_real"
DATA_DIR="${REPO_ROOT}/Data_ROSBAGS"
CONTAINER="vulcanexus-bridge"

echo "============================================"
echo "  open-robin dev setup"
echo "============================================"

# 1. Symlink rosbag (fix broken symlink if present)
mkdir -p "${DATA_DIR}"
LINK="${DATA_DIR}/exp001_rosbag_real"
if [ -L "${LINK}" ] && [ ! -e "${LINK}" ]; then
  echo "[FIX] Removing broken symlink: ${LINK}"
  rm "${LINK}"
fi
if [ ! -e "${LINK}" ]; then
  if [ ! -d "${ROBIN_BAG}" ]; then
    echo "[ERROR] Rosbag not found at: ${ROBIN_BAG}"
    echo "  Please ensure exp001_rosbag_real exists at that path."
    exit 1
  fi
  ln -s "${ROBIN_BAG}" "${LINK}"
  echo "[OK] Symlinked rosbag:"
  echo "     ${LINK} -> ${ROBIN_BAG}"
else
  echo "[OK] Rosbag symlink already exists."
fi

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
