# ROBIN ROS2 Packages Documentation

This document describes the ROS2 packages that make up the ROBIN welding robot system.

## System Architecture

```
robin_core_bringup (Main Orchestrator)
├── robin_hardware_ur (UR10e Robot)
│   └── ur_robot_driver / ur_simulation_gz
├── robin_moveit_control (Motion Planning)
│   └── robin_moveit_config (MoveIt Configuration)
├── robin_hardware_garmo (Laser Profilometer)
│   └── robin_core_sensor (Data Processing)
├── robin_hardware_opcua (OPC UA Bridge - Fronius & WAGO)
└── robin_hardware_fronius (Welding Coordinator)

robin_interfaces (Shared Messages/Services/Actions)
robin_simulation (Testing/Development)
```

---

## Package Overview

| Package | Type | Description |
|---------|------|-------------|
| `robin_core_bringup` | Python | Main system orchestration and launch files |
| `robin_core_sensor` | Python | Pointcloud processing for weld bead measurement |
| `robin_hardware_fronius` | Python | Welding coordinator (start/stop sequences) |
| `robin_hardware_garmo` | Python | Garmo laser profilometer driver |
| `robin_hardware_opcua` | C++ | OPC UA bridge for Fronius & WAGO (open62541) |
| `robin_hardware_ur` | Python | UR10e robot configuration and launch |
| `robin_interfaces` | CMake | Custom messages, services, and actions |
| `robin_moveit_config` | CMake | MoveIt configuration with Pilz planner |
| `robin_moveit_control` | CMake | MoveItPy motion planning and control |
| `robin_simulation` | Python | Welding process simulation |

---

## 1. robin_core_bringup

Main system orchestration package that brings up the complete ROBIN welding robot system.

### Launch Files

| Launch File | Description |
|-------------|-------------|
| `robin.launch.py` | Main bringup - orchestrates all subsystems |

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_ip` | `192.168.1.101` | UR10e robot IP address |
| `garmo_ip` | `192.168.1.212` | Garmo sensor IP address |
| `garmo_delay` | `3.0` | Delay before starting Garmo sensor (s) |
| `enable_garmo` | `true` | Enable Garmo sensor |
| `enable_fronius` | `true` | Enable Fronius welder |
| `simulation` | `false` | Launch in simulation mode |

### Usage

```bash
# Full system (real hardware)
ros2 launch robin_core_bringup robin.launch.py

# Simulation mode
ros2 launch robin_core_bringup robin.launch.py simulation:=true

# Without Garmo sensor
ros2 launch robin_core_bringup robin.launch.py enable_garmo:=false
```

---

## 2. robin_core_sensor

Processes pointcloud data from the Garmo laser profilometer to extract weld bead dimensions.

### Nodes

| Executable | Node Name | Description |
|------------|-----------|-------------|
| `process_data` | `weld_profile_processor` | RANSAC + DBSCAN weld bead extraction |

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/robin/pointcloud` | `sensor_msgs/PointCloud2` | Subscribe | Input pointcloud |
| `/robin/weld_dimensions` | `std_msgs/Float32MultiArray` | Publish | Output [width, height] |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `moving_average_window` | `5` | Filter window size |

### Algorithm

1. Fit base plane using RANSAC
2. Identify outliers (potential weld bead)
3. Cluster outliers using DBSCAN
4. Calculate width/height from largest cluster

---

## 3. robin_hardware_fronius

Welding coordinator that orchestrates start/stop sequences via OPC UA bridge services.

### Nodes

| Executable | Node Name | Description |
|------------|-----------|-------------|
| `welding_coordinator` | `welding_coordinator` | Welding sequence orchestration |

### Services Provided

| Service | Type | Description |
|---------|------|-------------|
| `/welding/start` | `robin_interfaces/StartWeld` | Set params + start welding sequence |
| `/welding/stop` | `std_srvs/Trigger` | Stop welding sequence |
| `/welding/set_params` | `robin_interfaces/StartWeld` | Set parameters only |

### Start Sequence
1. Set Fronius parameters (current, voltage, wire_speed)
2. `gas_on(true)`
3. `robot_ready(true)`
4. `welding_start(true)`

### Stop Sequence
1. `welding_start(false)`
2. `robot_ready(false)`
3. `gas_on(false)`

### Usage

```bash
ros2 launch robin_hardware_fronius welding_coordinator.launch.py

# Start welding with parameters
ros2 service call /welding/start robin_interfaces/srv/StartWeld \
  "{current: 150.0, voltage: 22.0, wire_speed: 8.0}"

# Stop welding
ros2 service call /welding/stop std_srvs/srv/Trigger
```

---

## 4. robin_hardware_garmo

Garmo laser profilometer driver for weld seam tracking and bead measurement.

### Nodes

| Executable | Node Name | Description |
|------------|-----------|-------------|
| `sensor_data` | `robin_sensor_publisher` | Pointcloud publisher |
| `sensor_cmd` | `sensor_cmd_node` | Sensor control services |

### Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/robin/pointcloud` | `sensor_msgs/PointCloud2` | Laser scan data |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `profilometer_activate` | `std_srvs/Trigger` | Start sensor streaming |
| `profilometer_deactivate` | `std_srvs/Trigger` | Stop sensor streaming |

### Parameters (sensor_data)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ip_address` | `192.168.1.212` | Sensor IP |
| `data_port` | `66` | Data stream port |
| `frame_id` | `garmo_laser_frame` | TF frame ID |
| `parent_frame` | `world` | Parent TF frame |
| `translation` | `[0,0,0]` | Sensor offset (m) |
| `rotation` | `[0,0,0]` | Sensor rotation (rad) |
| `output_units` | `m` | Units: raw/um/mm/m |

### Usage

```bash
ros2 launch robin_hardware_garmo garmo.launch.py

# Activate sensor
ros2 service call /profilometer_activate std_srvs/srv/Trigger
```

---

## 5. robin_hardware_opcua

High-performance OPC UA bridge using the **open62541** C library. Provides a unified interface for both Fronius welder and WAGO PLC via YAML configuration.

### Features
- **Batch reads** - All topics read in single OPC UA request (~20 Hz)
- **Pre-parsed NodeIDs** - Parsed once at startup
- **YAML configuration** - Easy to add new servers/topics/services
- **Multiple server support** - Handles different auth methods

### Nodes

| Executable | Node Name | Description |
|------------|-----------|-------------|
| `opcua_bridge_node` | `opcua_bridge` | Unified OPC UA bridge |

### Topics Published (Fronius)

| Topic | Type | Description |
|-------|------|-------------|
| `/fronius/actual_current` | `Float32` | Welding current (A) |
| `/fronius/actual_voltage` | `Float32` | Welding voltage (V) |
| `/fronius/actual_wfs` | `Float32` | Wire feed speed |
| `/fronius/actual_power` | `Float32` | Welding power (W) |
| `/fronius/actual_weldingtime` | `Float32` | Welding time |
| `/fronius/actual_gasflow` | `Float32` | Gas flow rate |

### Services (Fronius)

| Service | Type | Description |
|---------|------|-------------|
| `/fronius/set_current` | `robin_interfaces/SetFloat32` | Set welding current |
| `/fronius/set_voltage` | `robin_interfaces/SetFloat32` | Set welding voltage |
| `/fronius/set_wire_speed` | `robin_interfaces/SetFloat32` | Set wire feed speed |

### Services (WAGO PLC)

| Service | Type | Description |
|---------|------|-------------|
| `/wago/gas_on` | `std_srvs/SetBool` | Shielding gas control |
| `/wago/robot_ready` | `std_srvs/SetBool` | Robot ready signal |
| `/wago/welding_start` | `std_srvs/SetBool` | Welding start trigger |
| `/wago/wire_backward` | `std_srvs/SetBool` | Wire retract |
| `/wago/wire_forward` | `std_srvs/SetBool` | Wire feed |

### Configuration

Config file: `config/opcua_bridge.yaml`

```yaml
servers:
  fronius:
    url: "opc.tcp://192.168.1.104:4840"
    auth:
      type: anonymous
    poll_rate: 100.0  # Hz
    topics:
      - name: "fronius/actual_current"
        node_id: "ns=1;s=ACTUAL_CURRENT"
        type: float32
    services:
      - name: "fronius/set_current"
        node_id: "ns=1;s=CURRENT_RECOMMVALUE"
        type: float32

  wago:
    url: "opc.tcp://192.168.0.17:4840"
    auth:
      type: username_password
      username: "admin"
      password: "wago"
    services:
      - name: "wago/gas_on"
        node_id: "ns=4;s=|var|WAGO_OPCUA.Application.IO.GasOn"
        type: bool
```

### Usage

```bash
ros2 launch robin_hardware_opcua opcua_bridge.launch.py

# Fronius: Set current
ros2 service call /fronius/set_current robin_interfaces/srv/SetFloat32 "{data: 150.0}"

# WAGO: Turn on gas
ros2 service call /wago/gas_on std_srvs/srv/SetBool "{data: true}"

# Check topic rate
ros2 topic hz /fronius/actual_current
```

---

## 6. robin_hardware_ur

UR10e robot configuration and driver launch wrapper.

### Launch Files

| Launch File | Description |
|-------------|-------------|
| `ur.launch.py` | UR10e driver (real or simulation) |

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_ip` | `192.168.1.101` | Robot IP address |
| `initial_controller` | `scaled_joint_trajectory_controller` | Active controller |
| `activate_joint_controller` | `true` | Activate joint controller |
| `simulation` | `false` | Use Gazebo simulation |

### Available Controllers

- `scaled_joint_trajectory_controller` - Default, respects speed scaling
- `joint_trajectory_controller` - Standard trajectory controller
- `forward_velocity_controller` - Direct velocity control
- `forward_position_controller` - Direct position control
- `freedrive_mode_controller` - Manual guidance mode

---

## 7. robin_interfaces

Custom ROS2 message, service, and action definitions.

### Messages

| Message | Description |
|---------|-------------|
| `WeldBeadData` | Processed weld bead geometry (area, width, height, header) |
| `WeldBead` | Weld bead definition (pose, dimensions, parameters) |
| `WelderData` | Welder telemetry (current, voltage, power, wire speed, etc.) |
| `WeldPlate` | Collection of weld beads |
| `WeldingMeasurement` | Complete measurement record with timestamps |

### Services

| Service | Description |
|---------|-------------|
| `SensorControl` | Sensor command interface |
| `SetWeldingParameters` | Set current, voltage, wire speed |
| `TriggerWithResult` | Generic trigger with success/message |

### Actions

| Action | Description |
|--------|-------------|
| `ExecuteWeldBead` | Execute single weld bead with feedback |
| `WeldExperiment` | Execute multi-bead experiment |

---

## 8. robin_moveit_config

MoveIt configuration for UR10e + Fronius torch + Garmo sensor.

### Launch Files

| Launch File | Description |
|-------------|-------------|
| `move_group.launch.py` | MoveIt move_group with Pilz planner |

### Configuration Files

| File | Description |
|------|-------------|
| `joint_limits.yaml` | Position/velocity/acceleration limits |
| `kinematics.yaml` | IK solver configuration |
| `moveit_controllers.yaml` | Controller configuration |
| `moveit_cpp.yaml` | MoveItCpp configuration |
| `pilz_cartesian_limits.yaml` | Cartesian velocity/acceleration limits |
| `pilz_industrial_motion_planner.yaml` | Pilz planner configuration |

### Planning

- **Planner:** Pilz Industrial Motion Planner
- **Motion Types:** PTP (point-to-point), LIN (linear), CIRC (circular)
- **Planning Group:** `ur_manipulator`

---

## 9. robin_moveit_control

MoveItPy-based motion planning with weld experiment action server.

### Nodes

| Executable | Node Name | Description |
|------------|-----------|-------------|
| `robin_planner` | `robin_moveit_planner` | Motion planner with welding integration |

### Action Servers

| Action | Type | Description |
|--------|------|-------------|
| `weld_experiment` | `WeldExperiment` | Execute multi-bead experiments |

### Service Clients

| Service | Type | Description |
|---------|------|-------------|
| `set_welding_parameters` | `SetWeldingParameters` | Configure Fronius welder |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `trajectory_execution_timeout` | `30.0` | Execution timeout (s) |
| `approach_height` | `0.05` | Height above weld start (m) |
| `torch_tilt` | `0.0` | Torch tilt angle (degrees) |
| `default_velocity_scaling` | `0.3` | Velocity scaling factor |
| `default_acceleration_scaling` | `0.3` | Acceleration scaling factor |
| `max_cartesian_velocity` | `0.25` | Max Cartesian velocity (m/s) |

### Motion Planning

- Uses **Pilz planner** for industrial-grade motion
- **LIN** (linear) motions for welding paths
- **PTP** (point-to-point) for approach/retract
- Automatic velocity scaling based on `target_speed` parameter

### Usage

```bash
ros2 launch robin_moveit_control planner.launch.py

# Execute weld experiment via action
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{plate: {beads: [...]}}"
```

---

## 10. robin_simulation

Welding process simulation for testing and development.

### Nodes

| Executable | Node Name | Description |
|------------|-----------|-------------|
| `welding_sim_node` | `welding_sim_node` | Physics-based welding simulation |

### Topics Published

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/fronius_data` | `WelderData` | 10 Hz | Simulated welder telemetry |
| `/tool_velocity` | `Float32` | 100 Hz | Tool velocity |
| `/welding/bead_geometry` | `WeldBeadData` | 42 Hz | Simulated bead geometry |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `process_id` | `default_process` | Process identifier |
| `wire_feed_rate_m_min` | `8.0` | Wire feed rate (m/min) |
| `current` | `200.0` | Welding current (A) |
| `voltage` | `22.0` | Welding voltage (V) |

---

## Network Configuration

| Device | IP Address | Port | Protocol |
|--------|------------|------|----------|
| UR10e Robot | 192.168.1.101 | 50002 | UR RTDE |
| Fronius Welder | 192.168.1.104 | 4840 | OPC UA |
| Garmo Sensor | 192.168.1.212 | 66/5020 | TCP |
| WAGO PLC | 192.168.0.17 | 4840 | OPC UA |

---

## Quick Start

### 1. Start the full system

```bash
# Source workspace
source ws_setup.sh

# Launch everything
ros2 launch robin_core_bringup robin.launch.py
```

### 2. Run a weld experiment

```bash
# In another terminal
source ws_setup.sh

# Send weld experiment action goal
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{plate: {beads: [{...}]}}"
```

### 3. Manual control

```bash
# Gas control
ros2 service call /wago_plc/gas_on std_srvs/srv/SetBool "{data: true}"

# Set welding parameters
ros2 service call /set_welding_parameters robin_interfaces/srv/SetWeldingParameters \
  "{current: 200.0, voltage: 22.0, wire_feed_speed: 8.0}"

# Activate sensor
ros2 service call /profilometer_activate std_srvs/srv/Trigger
```

---

## Building

```bash
# Inside Docker container
source ws_setup.sh
colcon build --symlink-install

# Build specific package
colcon build --symlink-install --packages-select robin_hardware_opcua
```

## Dependencies

- **ROS2 Jazzy** (via Vulcanexus)
- **MoveIt 2** with Pilz Industrial Motion Planner
- **ur_robot_driver** for UR10e
- **open62541** v1.4.14 (built from source for WAGO PLC)
- **asyncua** Python library (for Fronius OPC UA)

---

## HRI Skill Layer (welding_*)

The `welding_*` packages implement a skill-based Human-Robot Interaction (HRI) layer on top of the core ROBIN system. Operator intents from the React dashboard are translated into ROS2 action goals and dispatched to specialised skill action servers.

### System Architecture

```
Robin Dashboard (React, port 5174)
        │
        │  POST /intent  (JSON)
        ▼
welding_http_bridge  (:8766)
        │
        │  /intents  (welding_msgs/Intent)
        ▼
welding_supervisor
        ├─► MOVE_TO_HOME             ──► welding_home_skill/execute
        ├─► EXECUTE_SEAM             ──► welding_seam_skill/execute
        ├─► ESTOP                    ──► cancel all active goals
        ├─► START_PROCESS            ──► welding_seam_skill/execute  (reuses seam)
        ├─► REQUEST_AI_RECOMMENDATION──► welding_recommendation_skill/execute
        ├─► MANUAL_ADJUST            ──► welding_manual_skill/execute
        ├─► LAUNCH_NEW_DOE           ──► /doe/launch  (std_msgs/String, JSON)
        ├─► PAUSE_PROCESS            ──► cancel seam → welding_home_skill/execute
        ├─► RESUME_PROCESS           ──► welding_home_skill/execute
        └─► STOP_PROCESS             ──► cancel all → welding_home_skill/execute
```

### Package Index

| Package | Build | Description |
|---------|-------|-------------|
| `welding_msgs` | CMake | Custom Intent message + action definitions |
| `welding_http_bridge` | Python | HTTP ↔ ROS2 intent bridge (port 8766) |
| `welding_supervisor` | Python | Intent-to-skill mission controller |
| `welding_home_skill` | Python | Move robot to home joint configuration |
| `welding_seam_skill` | Python | Execute a weld seam (simulation or hardware) |
| `welding_recommendation_skill` | Python | Request an AI process recommendation |
| `welding_manual_skill` | Python | Apply a manual parameter adjustment to hardware |
| `welding_demo` | Python | Master launch file for the full HRI demo stack |

---

### welding_msgs

Custom message and action definitions shared by all welding packages.

**Message:** `welding_msgs/msg/Intent`

| Field | Type | Description |
|-------|------|-------------|
| `intent` | `string` | One of the intent type constants below |
| `data` | `string` | JSON-encoded intent payload |
| `source` | `string` | `ROBOT` / `REMOTE` / `UNKNOWN` |
| `modality` | `string` | `TOUCHSCREEN` / `SPEECH` / `GESTURE` / `INTERNAL` |
| `priority` | `float32` | 0.0 (lowest) → 1.0 (highest) |
| `confidence` | `float32` | Always 1.0 for GUI button presses |

**Intent type constants:**

| Constant | Routes to |
|----------|-----------|
| `MOVE_TO_HOME` | `welding_home_skill` |
| `EXECUTE_SEAM` | `welding_seam_skill` |
| `ESTOP` | Cancel all active goals |
| `START_PROCESS` | `welding_seam_skill` (reuses seam) |
| `REQUEST_AI_RECOMMENDATION` | `welding_recommendation_skill` |
| `MANUAL_ADJUST` | `welding_manual_skill` |
| `LAUNCH_NEW_DOE` | `/doe/launch` topic |
| `PAUSE_PROCESS` | Cancel seam → `welding_home_skill` |
| `RESUME_PROCESS` | `welding_home_skill` |
| `STOP_PROCESS` | Cancel all → `welding_home_skill` |

**Action definitions:**

| Action | Goal fields | Result fields | Feedback fields |
|--------|-------------|---------------|-----------------|
| `MoveToHome` | `use_fast_speed: bool` | `success`, `message` | `progress_pct`, `current_joint` |
| `ExecuteSeam` | `seam_id`, `weld_speed`, `wire_feed_rate` | `success`, `message`, `seam_length_mm` | `progress_pct`, `current_speed`, `phase` |
| `RequestAIRecommendation` | `process_id`, `mode` | `success`, `message`, `recommendation_json` | `progress_pct`, `phase` |
| `ManualAdjust` | `parameter_name`, `new_value`, `unit` | `success`, `message`, `applied_value` | `progress_pct`, `phase` |

---

### welding_http_bridge

Translates HTTP requests from the React dashboard into ROS2 intent messages, and forwards NGSI-LD context broker notifications.

**Node:** `welding_http_bridge_node`  
**Port:** `8766` (aiohttp async server)

**Endpoints:**

| Method | Path | Description |
|--------|------|-------------|
| `POST` | `/intent` | Publish an intent from JSON body `{"intent": "...", "data": {...}}` |
| `POST` | `/orion-notify` | Receive NGSI-LD subscription callbacks and publish as intent |
| `GET` | `/health` | Returns `{"status": "ok"}` |

**Topics published:**

| Topic | Type | Description |
|-------|------|-------------|
| `/intents` | `welding_msgs/Intent` | Forwarded intent from HTTP or Orion-LD notification |

**Key behaviour:**
- CORS enabled for all origins (development: `localhost:5174`)
- Background daemon thread registers and refreshes Orion-LD subscription on `pendingIntent` attribute changes
- Thread-safe: aiohttp async handlers publish directly on the rclpy publisher

---

### welding_supervisor

Intent-to-skill mission controller. Subscribes to `/intents`, parses the JSON payload, and dispatches each intent to the matching skill action server.

**Node:** `welding_supervisor_node`  
**Executor:** `MultiThreadedExecutor(4 threads)` with `ReentrantCallbackGroup`

**Topics subscribed:**

| Topic | Type | Description |
|-------|------|-------------|
| `/intents` | `welding_msgs/Intent` | Incoming operator intents |

**Topics published:**

| Topic | Type | Description |
|-------|------|-------------|
| `/doe/launch` | `std_msgs/String` | DOE GUI launch notification (JSON) |

**Action clients:**

| Client | Action type | Skill |
|--------|-------------|-------|
| `welding_home_skill/execute` | `MoveToHome` | `welding_home_skill` |
| `welding_seam_skill/execute` | `ExecuteSeam` | `welding_seam_skill` |
| `welding_recommendation_skill/execute` | `RequestAIRecommendation` | `welding_recommendation_skill` |
| `welding_manual_skill/execute` | `ManualAdjust` | `welding_manual_skill` |

**Goal tracking:**
- `_active_goal_handles` — all currently accepted goals (cancelled by ESTOP / STOP_PROCESS)
- `_active_seam_goal_handles` — seam/start goals only (cancelled by PAUSE_PROCESS)

---

### welding_home_skill

Moves the robot to its home joint configuration.

**Node:** `welding_home_skill_node` (LifecycleNode)  
**Action server:** `welding_home_skill/execute` (`MoveToHome`)  
**Executor:** `MultiThreadedExecutor(2 threads)`

**Modes:**

| Mode | Behaviour |
|------|-----------|
| Simulation (default) | 3-second mock movement across 6 joints with per-joint progress feedback |
| Hardware | Delegates to `/move_home` action on `robin_moveit_control` (placeholder — falls back to simulation with warning) |

---

### welding_seam_skill

Executes a weld seam path. The primary skill for `START_PROCESS` and `EXECUTE_SEAM` intents.

**Node:** `welding_seam_skill_node` (LifecycleNode)  
**Action server:** `welding_seam_skill/execute` (`ExecuteSeam`)  
**Executor:** `MultiThreadedExecutor(4 threads)`

**Modes:**

| Mode | Behaviour |
|------|-----------|
| Simulation (default) | 6-second mock with phases: `IGNITING → WELDING×3 → FINISHING`, returns `seam_length_mm=150` |
| Hardware | Delegates to `/weld_experiment` action on `robin_moveit_control` (RobinPlanner). Builds a `WeldBead` from the seam registry. Default seam `seam_01`: start `[0.45, -0.20, 0.05]` → end `[0.45, 0.20, 0.05]`, 220 A / 26 V / 15 mm stickout. |

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `seam_registry` | `{}` | YAML map of seam IDs to coordinate pairs |
| `simulation_mode` | `true` | Set to `false` to use hardware path |

---

### welding_recommendation_skill

Requests an AI process recommendation from the ROBIN backend.

**Node:** `welding_recommendation_skill_node` (LifecycleNode)  
**Action server:** `welding_recommendation_skill/execute` (`RequestAIRecommendation`)  
**Executor:** `MultiThreadedExecutor(2 threads)`

**Modes:**

| Mode | Behaviour |
|------|-----------|
| Simulation (default) | 3-second mock with phases `FETCHING → PROCESSING → COMPLETE`. Returns stub recommendation: `weld_speed=4.8 mm/s, wire_feed=4.2 m/min, current=210 A, voltage=24.5 V, confidence=0.87` |
| Production | Replace stub with `POST /ai-recommendation` call to ROBIN backend |

---

### welding_manual_skill

Applies a manual process parameter adjustment, optionally writing to hardware.

**Node:** `welding_manual_skill_node` (LifecycleNode)  
**Action server:** `welding_manual_skill/execute` (`ManualAdjust`)  
**Executor:** `MultiThreadedExecutor(4 threads)`

**Modes:**

| Mode | Behaviour |
|------|-----------|
| Simulation (default) | 2-second mock with phases `VALIDATING → APPLYING → CONFIRMING` |
| Hardware | Calls Fronius `SetFloat32` services via the OPC UA bridge |

**Hardware parameter mapping (hardware mode):**

| `parameter_name` | ROS2 service | Unit | Safe range |
|------------------|-------------|------|------------|
| `current` | `/fronius/set_current` | A | 50 – 400 A |
| `voltage` | `/fronius/set_voltage` | V | 10 – 50 V |
| `wire_speed` | `/fronius/set_wire_speed` | m/min | 1 – 15 m/min |
| `weld_speed` | *(motion — logs warning)* | mm/s | — |

Values are clamped to their safe range before being sent to hardware.

---

### welding_demo

Master launch file that starts the complete HRI demo stack in the correct order.

**Launch file:** `welding_robin_demo.launch.py`

**Start order:**

| Time | Nodes launched |
|------|---------------|
| t = 0 s | `welding_home_skill`, `welding_seam_skill`, `welding_recommendation_skill`, `welding_manual_skill`, `welding_http_bridge` |
| t = 2 s | `welding_supervisor` (delayed so all action servers are ready) |

**Quick start:**

```bash
# Build
colcon build --packages-up-to welding_demo

# Launch full HRI stack
ros2 launch welding_demo welding_robin_demo.launch.py

# Test an intent manually
curl -s -X POST http://localhost:8766/intent \
     -H 'Content-Type: application/json' \
     -d '{"intent": "START_PROCESS", "data": {"seam_id": "seam_01"}}' | jq

# Monitor the intent stream
ros2 topic echo /intents
```
