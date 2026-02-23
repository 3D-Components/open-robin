## Running the ROS2 Welding Simulator (Vulcanexus)

This section details how to build and run the Vulcanexus ROS2 container, which simulates welding data and publishes it to a DDS topic.

### 1\. Prerequisites

#### Environment Setup

Before launching, you must create a `.env` file in this directory. This file provides your local User and Group IDs to Docker Compose, which is **essential** for fixing file permissions on your mounted workspaces.

Run this command once in your terminal from this directory:

```bash
./setup_env.sh
```

#### Robot Setup (Universal Robot)
ROBIN uses the `headless` operation mode for UR. The robot therefore needs to be in `Remote Control` mode to receive the script code. Please  see https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/operation_modes.html for details.


### 2\. First-Time Setup (Building the Node)

Before you can run the simulator, you must get a shell inside the `vulcanexus` container and build the ROS2 packages. You only need to do this once.

1.  **Start all services:**

    ```bash
    docker compose up -d
    ```

2.  **Get a shell inside the `vulcanexus-bridge` container as root:**

    ```bash
    docker exec -it -u root vulcanexus-bridge bash
    ```

    This should bring you into `/workspace/ros2_packages`

3.  **Install Python dependencies:**
    Inside the container (as root), run the following to install `pandas`, `numpy`, and `matplotlib`:

    ```bash
    apt-get update
    apt-get install -y \
        python3-pandas \
        python3-numpy \
        python3-matplotlib
    ```

4.  **Build the ROS2 workspace:**
    Compile your ROS2 packages.

    ```bash
    # Source the main Vulcanexus environment
    source ws_setup.sh

    # Build the workspace. Note that the build, log and install directories exist only in the container as we mount the src folder, not the workspace root.
    colcon build --symlink-install
    ```

### 3\. Running the Simulator

Any time you want to start the simulation, run these commands from *inside* the `vulcanexus-bridge` container (use `docker exec -it vulcanexus-bridge bash` to get a shell).

```bash
# 1. Source the main Vulcanexus environment
source /opt/vulcanexus/jazzy/setup.bash

# 2. Navigate to your workspace and source your new build
cd /workspace/ros2_packages
source install/setup.bash

# 3. Run the node
# This will start publishing to the 'rt/welding/measurement' topic
ros2 run robin_simulation welding_sim_node --ros-args -p process_id:="sim-test-01"
```

### 4. Running the Robot with MoveIt Planner

```bash
source ws_setup.sh
```

```bash
ros2 launch robin_core_bringup robin_main.launch.py
```

To test it, send an action goal in another terminal.

> **Warning:** Ensure the goal positions and parameters are appropriate for your robot and workspace (joint limits, reachability, and collisions). Sending out-of-range or unsafe goals may cause unexpected motion or damage.

#### List of test commands (serves as a working scratchbook of commands for now)
```bash
source ws_setup.sh
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{weld_beads: [
    {start_point: {x: -1.2, y: 0.1, z: 0.0}, end_point: {x: -1.1, y: 0.1, z: 0.0}, target_speed: 0.01, target_voltage: 20.0, target_current: 150.0, wire_feed_speed: 5.0},
    {start_point: {x: -1.2, y: 0.2, z: 0.0}, end_point: {x: -1.1, y: 0.2, z: 0.0}, target_speed: 0.01, target_voltage: 20.0, target_current: 150.0, wire_feed_speed: 5.0},
    {start_point: {x: -1.2, y: 0.3, z: 0.0}, end_point: {x: -1.1, y: 0.3, z: 0.0}, target_speed: 0.01, target_voltage: 20.0, target_current: 150.0, wire_feed_speed: 5.0},
    {start_point: {x: -1.2, y: 0.4, z: 0.0}, end_point: {x: -1.1, y: 0.4, z: 0.0}, target_speed: 0.01, target_voltage: 20.0, target_current: 150.0, wire_feed_speed: 5.0},
    {start_point: {x: -1.2, y: 0.5, z: 0.0}, end_point: {x: -1.1, y: 0.5, z: 0.0}, target_speed: 0.01, target_voltage: 20.0, target_current: 150.0, wire_feed_speed: 5.0}
  ]}" \
  --feedback
```

```bash
source ws_setup.sh
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{weld_beads: [
    {start_point: {x: -1.3, y: -0.4, z: -0.18}, end_point: {x: -1.2, y: -0.4, z: -0.18}, target_speed: 0.01, target_voltage: 15.0, target_current: 125.0, wire_feed_speed: 5.0},
    {start_point: {x: -1.3, y: -0.35, z: -0.18}, end_point: {x: -1.2, y: -0.35, z: -0.18}, target_speed: 0.01, target_voltage: 16.0, target_current: 120.0, wire_feed_speed: 6.0}
  ]}" \
  --feedback
```

```bash
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{dry_run: true, weld_beads: [
    {start_point: {x: -1.3, y: -0.4, z: -0.20}, end_point: {x: -1.2, y: -0.4, z: -0.20}, target_speed: 0.005, target_voltage: 15.0, target_current: 125.0, wire_feed_speed: 5.0, stickout: 0.0},
    {start_point: {x: -1.3, y: -0.35, z: -0.20}, end_point: {x: -1.2, y: -0.35, z: -0.20}, target_speed: 0.005, target_voltage: 16.0, target_current: 120.0, wire_feed_speed: 6.0, stickout: 0.00}
  ]}" \
  --feedback
```

```bash
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{dry_run: false, weld_beads: [
    {start_point: {x: -1.3, y: -0.4, z: -0.20}, end_point: {x: -1.2, y: -0.4, z: -0.20}, target_speed: 0.01, target_voltage: 15.0, target_current: 125.0, wire_feed_speed: 5.0, stickout: 0.0},
    {start_point: {x: -1.3, y: -0.35, z: -0.20}, end_point: {x: -1.2, y: -0.35, z: -0.20}, target_speed: 0.01, target_voltage: 16.0, target_current: 120.0, wire_feed_speed: 6.0, stickout: 0.00}
  ]}" \
  --feedback
```

```bash
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{dry_run: true, weld_beads: [
    {start_point: {x: -1.3, y: -0.3, z: -0.205}, end_point: {x: -1.2, y: -0.3, z: -0.211}, target_speed: 0.005, target_voltage: 15.0, target_current: 125.0, wire_feed_speed: 5.0, stickout: 0.0},
    {start_point: {x: -1.3, y: -0.25, z: -0.205}, end_point: {x: -1.2, y: -0.25, z: -0.211}, target_speed: 0.005, target_voltage: 16.0, target_current: 120.0, wire_feed_speed: 5.0, stickout: 0.0}
  ]}" \
  --feedback
```
```bash
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{dry_run: true, weld_beads: [
    {start_point: {x: -1.3, y: -0.3, z: -0.208}, end_point: {x: -1.2, y: -0.3, z: -0.208}, target_speed: 0.005, target_voltage: 15.0, target_current: 125.0, wire_feed_speed: 5.0, stickout: 0.0}
  ]}" \
  --feedback
```

```bash
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{dry_run: false, weld_beads: [
    {start_point: {x: -1.3, y: -0.3, z: -0.208}, end_point: {x: -1.2, y: -0.3, z: -0.208}, target_speed: 0.005, target_voltage: 15.0, target_current: 125.0, wire_feed_speed: 5.0, stickout: 0.0}
  ]}" \
  --feedback
```


```bash
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{dry_run: false, weld_beads: [
    {start_point: {x: -1.3, y: -0.20, z: -0.206}, end_point: {x: -1.2, y: -0.20, z: -0.206}, target_speed: 0.008, target_voltage: 25.0, target_current: 175.0, wire_feed_speed: 9.0, stickout: 0.0}
  ]}" \
  --feedback
```

```bash
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{dry_run: false, weld_beads: [
    {start_point: {x: -1.2, y: -0.275, z: -0.206}, end_point: {x: -1.1, y: -0.275, z: -0.206}, target_speed: 0.006, target_voltage: 25.0, target_current: 220.0, wire_feed_speed: 9.0, stickout: 0.0}
  ]}" \
  --feedback
```

```bash
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{dry_run: true, weld_beads: [
    {bead_id: 'exp001_bead1', start_point: {x: -1.3, y: -0.7, z: -0.206}, end_point: {x: -1.2, y: -0.7, z: -0.206}, target_speed: 0.005, target_voltage: 25.0, target_current: 220.0, wire_feed_speed: 9.0, stickout: 0.0},
    {bead_id: 'exp001_bead2', start_point: {x: -1.3, y: -0.65, z: -0.206}, end_point: {x: -1.2, y: -0.65, z: -0.206}, target_speed: 0.005, target_voltage: 24.0, target_current: 210.0, wire_feed_speed: 9.0, stickout: 0.0}
  ]}" \
  --feedback
```

## OPC UA Configuration

### Fronius OPC UA Server

Endpoint: `opc.tcp://192.168.1.104:4840`

### WAGO PLC OPC UA Server
Default web interface credentials for `192.168.0.17`: `admin:wago` 
Endpoint: `opc.tcp://192.168.0.17:4840`

---

## Testing Weld Progression Tracking

The `weld_data_node` calculates progression (0.0-1.0) along the active bead based on TCP position. This enables alignment of data from different sources by matching progression values rather than timestamps.

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/robin/data/progression` | `WeldProgression` | Current progression along active bead |
| `/robin/data/fronius` | `FroniusSample` | Fronius data with progression added |
| `/robin/data/active_bead` | `ActiveBead` | Active bead info (published by robin_planner) |
| `/robin/data/is_welding` | `Bool` | Welding state (arc on/off) |

### Option 1: Full System Test (with hardware)

```bash
# Terminal 1: Launch the full system
source ws_setup.sh
ros2 launch robin_core_bringup robin_main.launch.py use_fronius:=true
```

```bash
# Terminal 2: Monitor progression data
source ws_setup.sh
ros2 topic echo /robin/data/progression
```

```bash
# Terminal 3: Monitor Fronius samples with progression
source ws_setup.sh
ros2 topic echo /robin/data/fronius
```

Then run a weld experiment via the action client (see examples above).

### Option 2: Isolated Node Test (no hardware)

```bash
# Terminal 1: Run weld_data_node standalone
source ws_setup.sh
ros2 run robin_core_data weld_data_node.py
```

```bash
# Terminal 2: Publish static TF (simulates robot at y=0.15, which is 50% along bead)
source ws_setup.sh
ros2 run tf2_ros static_transform_publisher 0.5 0.15 0.1 0 0 0 base_link weld_torch_tip
```

```bash
# Terminal 3: Simulate inputs
source ws_setup.sh

# Set active bead (300mm along Y axis)
ros2 topic pub /robin/data/active_bead robin_interfaces/msg/ActiveBead "{
  header: {frame_id: 'base_link'},
  bead_id: 'test_001',
  start_point: {x: 0.5, y: 0.0, z: 0.1},
  end_point: {x: 0.5, y: 0.3, z: 0.1},
  length: 0.3
}" --once

# Enable welding
ros2 topic pub /robin/data/is_welding std_msgs/msg/Bool "{data: true}" --once

# Simulate Fronius data (run in background)
ros2 topic pub /fronius/display_current std_msgs/msg/Float32 "{data: 180.0}" -r 10 &
ros2 topic pub /fronius/display_voltage std_msgs/msg/Float32 "{data: 24.5}" -r 10 &
```

```bash
# Terminal 4: Watch progression output
source ws_setup.sh
ros2 topic echo /robin/data/progression
```

**Expected output** (TCP at y=0.15 = 50% along 300mm bead):
```yaml
bead_id: 'test_001'
progression: 0.5
is_welding: true
tcp_position:
  x: 0.5
  y: 0.15
  z: 0.1
```

### Verification Checklist

| Check | Command | Expected |
|-------|---------|----------|
| Node running | `ros2 node list` | `/weld_data_node` |
| Topics exist | `ros2 topic list \| grep robin/data` | 4 topics listed |
| Progression at start | Move TCP to bead start | `progression: 0.0` |
| Progression at end | Move TCP to bead end | `progression: 1.0` |
| Fronius includes progression | `ros2 topic echo /robin/data/fronius` | Has `bead_id` and `progression` fields |
