#!/usr/bin/env bash
# cleanup_ros2.sh — Kill all dangling ROS2 nodes/processes before a fresh bringup.
#
# Usage (from host):
#   docker compose exec vulcanexus bash /workspace/ros2_packages/cleanup_ros2.sh
# Or inside the container:
#   bash /workspace/ros2_packages/cleanup_ros2.sh

set -uo pipefail

echo "=== ROBIN ROS2 cleanup ==="

# Covers:
#   ros2     — launch system and all Python nodes
#   /opt/ros — every installed C++ executable (robot_state_publisher, controller_manager, etc.)
#   gz       — Gazebo
#   rviz2    — RViz
PATTERNS="ros2|/opt/ros|gz|rviz2"

# 1. Graceful shutdown (SIGINT)
if pgrep -f "$PATTERNS" > /dev/null 2>&1; then
    echo "Sending SIGINT to ROS2 processes..."
    pkill -INT -f "$PATTERNS" 2>/dev/null || true
    sleep 1
fi

# 2. Force-kill survivors
if pgrep -f "$PATTERNS" > /dev/null 2>&1; then
    echo "Force-killing remaining processes..."
    pkill -9 -f "$PATTERNS" 2>/dev/null || true
fi

# 3. Stop the ROS2 daemon
ros2 daemon stop 2>/dev/null || true

# 4. Reap zombie (defunct) processes by killing their parents
ZOMBIE_PARENTS=$(ps -eo ppid,stat | awk '$2 ~ /Z/ {print $1}' | sort -u || true)
if [ -n "$ZOMBIE_PARENTS" ]; then
    echo "Reaping zombie parents..."
    for ppid in $ZOMBIE_PARENTS; do
        [ "$ppid" -gt 1 ] && kill -9 "$ppid" 2>/dev/null || true
    done
fi

# 5. Clean DDS shared memory (Fast-DDS / Cyclone)
if [ -d /dev/shm ]; then
    find /dev/shm -maxdepth 1 \( -name "fastrtps_*" -o -name "Fast-DDS*" -o -name "cyclone*" \) \
        -exec rm -f {} + 2>/dev/null || true
fi

# 6. Summary
REMAINING=$(pgrep -cf "$PATTERNS" 2>/dev/null || echo "0")
if [ "$REMAINING" -gt 0 ]; then
    echo "WARNING: $REMAINING process(es) still running:"
    pgrep -af "$PATTERNS" 2>/dev/null || true
else
    echo "All ROS2 processes cleaned up. Ready for fresh bringup."
fi
