#!/bin/bash
# Compatibility entry point for the maintained ROS bag DDS demo.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "demo/simulation-demo-rosbag.sh is a compatibility wrapper."
echo "Running demo/simulation-demo-rosbag-wfs-alc-ts.sh instead."
echo

exec "${SCRIPT_DIR}/simulation-demo-rosbag-wfs-alc-ts.sh" "$@"
