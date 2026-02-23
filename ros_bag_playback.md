# ROS Bag Playback Cookbook (DDS Path)

This guide explains how to replay ROS bags into ROBIN using the current DDS telemetry path.

## 1. Start Required Containers

From repository root:

```bash
docker compose up -d orion-ld vulcanexus mintaka alert-processor
```

## 2. Build ROS Workspace (inside Vulcanexus)

```bash
docker exec -it vulcanexus-bridge bash
source /opt/vulcanexus/jazzy/setup.bash
cd /workspace/ros2_packages
colcon build --symlink-install
source ws_setup.sh
```

## 3. Start Telemetry Aggregator

In container:

```bash
source ws_setup.sh
ros2 run robin_core_data telemetry_aggregator_node.py --ros-args \
  -p geometry_topic:=/robin/weld_dimensions \
  -p welder_topic:=/robin/data/fronius \
  -p robot_pose_topic:=/tcp_pose_broadcaster/pose
```

This publishes combined data on `/robin/telemetry`.

## 4. Replay the Bag

```bash
source ws_setup.sh
ros2 bag play /workspace/ros2_packages/exp001_rosbag_real --rate 0.1 --loop
```

## 5. Verify End-to-End Flow

### ROS side

```bash
ros2 topic echo /robin/telemetry
```

### Orion side

```bash
curl -H "NGSILD-Tenant: robin" \
  "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:Process:ros_bridge"
```

### Alert Engine side

```bash
curl "http://localhost:8001/process/ros_bridge/measurements?last=5"
```

## 6. One-Command Demo

Use:

```bash
./demo/simulation-demo-rosbag.sh
```

This script orchestrates setup and replay using the same DDS-first approach.

## Notes

- The legacy HTTP NGSI bridge flow is deprecated in this repository baseline.
- The bag topic names can still reflect domain-specific capture sessions (for example `/robin/weld_dimensions`), while core storage remains process-agnostic.

