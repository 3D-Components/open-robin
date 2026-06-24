# ROBIN ROS2 System — Node & Feature Reference

## Architecture Overview

```
robin_bringup (robin_main.launch.py)
├── STAGE 1 (immediate)
│   ├── robin_hardware_ur      → UR10e driver + robot_state_publisher
│   └── robin_hardware_opcua   → OPC UA bridge (WAGO PLC + Fronius)
├── STAGE 2 (delayed ~8s, waits for UR driver)
│   ├── robin_core planner     → MoveItPy + ExecuteBead action server
│   ├── robin_core experiment  → WeldExperiment action server + planning services
│   ├── robin_core tcp_manager → Wire-tip TF + stickout management
│   ├── robin_core plate_markers → RViz plate visualization
│   ├── moveit_servo           → Cartesian jogging
│   ├── rviz2                  → Visualization
│   └── robin_rqt              → Operator panel (RQt GUI)
├── STAGE 2 (delayed ~3s)
│   ├── robin_hardware_garmo   → Garmo profilometer (cmd + data)
│   └── robin_core process_data → Weld profile processing
└── STAGE 3 (delayed ~5s, waits for OPC UA bridge)
    ├── robin_hardware_fronius → Welding coordinator
    └── robin_core weld_data   → Progression-stamped weld data
```

---

## Packages & Nodes

### `robin_interfaces`

Custom ROS2 message, service, and action definitions.

| Type | Name | Purpose |
|------|------|---------|
| **msg** | `ActiveBead` | Active bead path + target params (for data node) |
| **msg** | `BeadGeometry` | Weld cross-section: height, width, area |
| **msg** | `ExperimentBead` | Dashboard-facing bead spec (params + plate assignment) |
| **msg** | `ExperimentBeadSpec` | System-agnostic bead input for experiment planning |
| **msg** | `PlateLayout` | Bead placement config (pitch, margins, stagger) |
| **msg** | `WeldPlate` | Plate definition (position, dimensions, calibration) |
| **msg** | `WeldProgression` | Normalized progression (0–1) along active bead |
| **msg** | `WelderData` | Fronius measurements with progression stamping |
| **srv** | `StartWeld` | Start arc with synergetic params (primary + overrides) |
| **srv** | `SetFloat32` / `SetInt32` | Generic typed setters for PLC registers |
| **srv** | `SetTcpMode` | Switch TCP: `"welding"` / `"scanning"` |
| **srv** | `SetStickout` | Set stickout value (optionally mark calibrated) |
| **srv** | `FindSurface` | Touch-sense probe at XY to find surface Z |
| **srv** | `CalibrateWireTip` | Wire feed until touch → measure wire tip distance |
| **srv** | `CalibratePlatePlane` | 4-point probe → fit plane z = ax + by + c |
| **srv** | `PlanExperiment` | Ingest bead specs + plates → compute placement |
| **srv** | `ApproveExperimentPlan` | Operator approval gate for a plan |
| **srv** | `ReserveSlots` | Mark plate regions as occupied (physical coords or grid+spacing) |
| **srv** | `ClearReservedSlots` | Remove slot reservations (by plate, bead_id, or all) |
| **action** | `ExecuteBead` | Single bead: approach → calibrate → weld → scan → retract |
| **action** | `WeldExperiment` | Execute a full set of beads across plates |

---

### `robin_hardware_ur`

**Node:** UR10e robot driver (via `ur_robot_driver`)

- Launches `ur_control.launch.py` (real) or `ur_sim_control.launch.py` (Gazebo)
- Robot type: **UR10e**
- Default IP: `192.168.1.101`
- Default controller: `scaled_joint_trajectory_controller`
- URDF: `robin_bringup/urdf/ur_fronius_garmo.urdf.xacro`
- Custom controller config: `robin_hardware_ur/config/ur10e_controllers.yaml`

---

### `robin_hardware_opcua`

**Node:** `opcua_bridge_node` (C++)

High-performance OPC UA ↔ ROS2 bridge. Connects to WAGO PLC and Fronius welder via OPC UA, exposing PLC signals as ROS2 topics and services.

| Server | URL | Poll Rate | Purpose |
|--------|-----|-----------|---------|
| `fronius` | `opc.tcp://192.168.1.104:4840` | 10 Hz | Welder display data + parameter writes |
| `wago` | `opc.tcp://192.168.0.17:4840` | 50 Hz | PLC I/O: welding control, touch sensing |

**Published topics (Fronius):**
- `/fronius/display_current` (Float32)
- `/fronius/display_voltage` (Float32)
- `/fronius/display_wfs` (Float32 — wire feed speed)
- `/fronius/display_power` (Float32)

**Published topics (WAGO OUT — PLC feedback):**
- `/wago/out/power_source_ready` (Bool)
- `/wago/out/process_active` (Bool)
- `/wago/out/touch_signal` (Bool)
- `/wago/out/robot_motion_release` (Bool)
- `/wago/out/current_flow` (Bool)
- `/wago/out/warning` (Bool)
- `/wago/out/arc_stable` (Bool)

**Services (Fronius — write parameters):**
- `/fronius/set_current` (SetFloat32)
- `/fronius/set_voltage` (SetFloat32)
- `/fronius/set_wire_speed` (SetFloat32)

**Services (WAGO IN — PLC control):**
- `/wago/in/robot_ready` (SetBool)
- `/wago/in/welding_start` (SetBool)
- `/wago/in/touch_sensing` (SetBool)
- `/wago/in/error_quit` (SetBool)
- `/wago/in/wire_forward` (SetBool)
- `/wago/in/wire_backward` (SetBool)
- `/wago/in/wire_move_length` (SetFloat32)
- `/wago/in/working_mode` (SetInt32)
- `/wago/in/welding_speed` (SetFloat32)
- `/wago/in/welding_simulation` (SetBool)

---

### `robin_hardware_fronius`

**Node:** `welding_coordinator`

Orchestrates welding start/stop sequences by coordinating WAGO PLC signals and Fronius parameters.

**Services provided:**
| Service | Type | Description |
|---------|------|-------------|
| `/welding/start` | StartWeld | Set params + start welding (waits for `robot_motion_release`) |
| `/welding/stop` | Trigger | Stop welding (waits for post-flow completion) |
| `/welding/set_params` | StartWeld | Set Fronius parameters without starting arc |
| `/welding/touch_probe` | Trigger | Activate touch sensing mode |
| `/welding/wire_feed_until_touch` | Trigger | Feed wire until touch signal fires |
| `/welding/wire_retract` | SetFloat32 | Retract wire by specified length |

**Welding start sequence:** Check power source ready → Set WorkingMode → Set Fronius params → Set robot_ready → Activate welding_start → Wait for robot_motion_release → Return success.

---

### `robin_hardware_garmo`

**Node 1:** `garmo_command_node` — Sensor control (start/stop/FPS)
- Services: `/profilometer_activate` (Trigger), `/profilometer_deactivate` (Trigger)
- Communicates via TCP socket to sensor at `192.168.1.212:5020`

**Node 2:** `sensor_data` — PointCloud2 publisher + TF broadcaster
- Connects to sensor data stream at `192.168.1.212:66`
- Publishes: `/robin/pointcloud` (PointCloud2)
- Broadcasts static TF: `laser_frame` → `garmo_laser_frame`
- Auto-reconnects on connection loss

**Spatial filtering:** Applies a 1D **bilateral filter** to the Z profile before publishing. This is an edge-preserving smoother — it aggressively removes noise on flat plate regions while keeping sharp bead-wall transitions intact. Each point's output is a weighted average of its neighbours, where weights combine spatial proximity (Gaussian over index distance) with value similarity (Gaussian over Z difference). Points across a steep Z edge get near-zero weight, so edges are not blurred.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `smoothing_window` | 51 | Filter window size (odd, ≥3; 0 to disable) |
| `smoothing_sigma_r` | 0.001 | Range sigma in data units (m). Smaller = sharper edge preservation |

Parameters are tunable at runtime: `ros2 param set /sensor_data smoothing_window 31`

---

### `robin_core` — Planner Node

**Node:** `robin_planner` (MoveItPy)

Single-bead motion execution via MoveIt Pilz planner.

**Action server:** `/execute_bead` (ExecuteBead)

Bead execution pipeline: approach clearance → pre-weld stickout calibration → move to weld start → start arc → LIN weld through waypoints → stop arc → scan pass → retract

**Calibration services (hosted by CalibrationManager):**
| Service | Type | Description |
|---------|------|-------------|
| `/calibration/find_surface` | FindSurface | Touch-probe to detect surface Z at XY |
| `/calibration/calibrate_wire_tip` | CalibrateWireTip | Wire-feed calibration of wire tip |
| `/calibration/calibrate_plate_plane` | CalibratePlatePlane | 4-point probe → plane fit |
| `/calibration/abort` | Trigger | Abort in-progress calibration |

**Key parameters** (from `robin_planner_params.yaml`):
- `approach_height`: 0.15 m
- `default_stickout`: 0.015 m (15 mm)
- `max_cartesian_velocity`: 0.25 m/s
- `end_effector_link`: `wire_tip`
- `base_frame`: `base_link`

---

### `robin_core` — Experiment Node

**Node:** `robin_experiment`

Manages experiment lifecycle: planning, operator approval, and orchestration of multiple beads.

**Action server:** `/weld_experiment` (WeldExperiment)

**Services:**
| Service | Type | Description |
|---------|------|-------------|
| `/experiment/plan` | PlanExperiment | Compute bead placement from specs + plates |
| `/experiment/approve` | ApproveExperimentPlan | Operator approval gate |
| `/experiment/terminate` | Trigger | Cancel running experiment |
| `/plate/reserve_slots` | ReserveSlots | Mark slots as occupied on partially-used plates |
| `/plate/clear_reserved_slots` | ClearReservedSlots | Remove slot reservations |

**Action client:** `/execute_bead` → sends per-bead goals to planner node

---

### `robin_core` — TCP Manager

**Node:** `tcp_manager`

Manages active TCP frame and wire stickout.

**Services:**
- `/tcp/set_mode` (SetTcpMode) — Switch between `welding` (wire_tip) / `scanning` (laser_frame)
- `/tcp/set_stickout` (SetStickout) — Update stickout value

**Published topics (TRANSIENT_LOCAL / latched):**
- `/tcp/active_frame` (String)
- `/tcp/stickout` (Float32)
- `/tcp/stickout_calibrated` (Bool)
- `/tf_static` — `contact_tip → wire_tip` transform

---

### `robin_core` — Weld Data Node

**Node:** `weld_data_node`

Progression-stamped weld data publisher. Projects TCP position onto the active bead path polyline to compute normalized progression (0.0–1.0).

**Subscribes:** `/fronius/display_*`, `/robin/data/active_bead`, `/robin/data/is_welding`, TF (`base_link → wire_tip`)

**Publishes:**
- `/robin/data/progression` (WeldProgression)
- `/robin/data/fronius` (WelderData)

---

### `robin_core` — Weld Profile Processor

**Node:** `weld_profile_processor`

Processes laser profilometer pointclouds into weld bead cross-section geometry.

**Subscribes:** `/robin/pointcloud`, `/robin/data/active_bead`, `/robin/data/progression`
**Publishes:** `/robin/weld_dimensions` (BeadGeometry — bead_id, progression, width_mm, height_mm)

**Algorithm** (single-frame, no temporal filtering):
1. **Sort** points by Y (cross-bead axis) to get a 1D profile.
2. **Low-Z prefilter** — take only the lower half of Z values (below median) as RANSAC candidates. The base plate is always the lowest surface, so this prevents wide flat bead tops from being mistaken for plate.
3. **Two-pass RANSAC** — Pass 1 fits a line to the low-Z subset. Inliers are then identified across all points (residual ≤ 0.2 mm). Pass 2 refits on inliers only for a clean base plane.
4. **Adaptive threshold** — `max(0.15 mm, 3σ)` where σ is the standard deviation of base-plate residuals.
5. **Toe detection** — find contiguous above-threshold segments, filter out noise segments (< 2 mm wide), select the bead closest to FOV centre.
6. **Sub-pixel interpolation** — linearly interpolate the exact Y position where the profile crosses the threshold at each toe.
7. **Output** — width = right toe − left toe; height = peak Z relative to base plane.

---

### `robin_core` — Profile Viewer

**Script:** `scripts/view_profile.py` (not a node — standalone matplotlib tool)

Live cross-section viewer for debugging. Replicates the processing pipeline and displays:
- Raw Z points (grey dots)
- Profile line (blue)
- Base-plate inliers (green circles)
- RANSAC base plane (green dashed)
- Bead threshold (red dashed)
- Detected toe positions (red vertical lines)
- Width/height dimension arrows
- Adjacent bead regions (grey shading)

**Axes:** Locked to Y ∈ [−15, 15] mm, Z ∈ [−5, 15] mm, aspect ratio 1:1. Throttled to 4 Hz.

```bash
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  python3 /workspace/ros2_packages/src/robin_core/scripts/view_profile.py"
```

---

### `robin_core` — Plate Markers

**Node:** `plate_markers`

Publishes RViz visualization markers for all plates defined in `plates.json`.

**Publishes:** `/robin/plates/markers` (MarkerArray)

---

### `robin_moveit_config`

MoveIt2 configuration for UR10e with Pilz Industrial Motion Planner. No standalone node — consumed by the planner launch file.

- Planning pipeline: `pilz_industrial_motion_planner` (PTP + LIN)
- SRDF: `robin.srdf.xacro`
- Includes MoveIt Servo config for Cartesian jogging

---

### `robin_rqt` — Operator Panel

**RQt plugin:** `OperatorPanel`

Tabbed GUI with:
- **Status tab** — system state overview
- **Experiment tab** — plan/approve/execute experiments
- **Manual tab** — direct PLC control (working mode, wire, weld start/stop)
- **Calibration tab** — stickout & plate-plane calibration
- **Plates tab** — plate management
- **Sensor tab** — laser profilometer control

Includes a **Jog component** for Cartesian jogging via MoveIt Servo.

---

### `robin_bringup`

Meta-package with the main launch file and URDF/config/RViz assets.

- `robin_main.launch.py` — launches the entire system
- `config/plates.json` — plate definitions
- `urdf/` — robot URDF xacro files
- `rviz/` — RViz config

---

## Quick Launch Commands

All commands run from the `vulcanexus_ws` directory inside the Docker container.

```bash
# Shorthand for running commands in the container
cd /home/mil-ai-pc/Repos/ROBIN/vulcanexus_ws
alias rex='docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && $@"'
```

### Full System Launch (real robot)

```bash
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 launch robin_bringup robin_main.launch.py robot_ip:=192.168.1.101"
```

### Full System Launch (simulation)

```bash
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 launch robin_bringup robin_main.launch.py is_simulation:=true"
```

### Launch Individual Components

```bash
# UR Robot driver only
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 launch robin_hardware_ur robot.launch.py robot_ip:=192.168.1.101"

# OPC UA bridge only
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 launch robin_hardware_opcua opcua_bridge.launch.py"

# Welding coordinator only (requires OPC UA bridge)
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 launch robin_hardware_fronius welding_coordinator.launch.py"

# MoveIt planner + servo only (requires UR driver)
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 launch robin_core moveit_planner.launch.py"

# Garmo sensor only
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 launch robin_hardware_garmo sensor.launch.py sensor_ip:=192.168.1.212"

# Experiment node only
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 launch robin_core experiment.launch.py"
```

### Launch Options for `robin_main.launch.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_ip` | `192.168.1.101` | UR10e IP address |
| `is_simulation` | `false` | Use Gazebo sim instead of real robot |
| `use_sensor` | `true` | Enable Garmo profilometer |
| `use_fronius` | `true` | Enable Fronius welder (OPC UA + coordinator) |
| `use_operator_panel` | `true` | Launch RQt operator panel |
| `record_bag` | `false` | Record rosbag with experiment data |
| `sensor_ip` | `192.168.1.212` | Garmo sensor IP |
| `moveit_start_delay` | `8.0` | Seconds to wait before launching MoveIt |
| `sensor_start_delay` | `3.0` | Seconds to wait before starting sensor |

### Build Commands

```bash
# Build all ROBIN packages
cd /home/mil-ai-pc/Repos/ROBIN/vulcanexus_ws && \
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  colcon build --packages-select robin_interfaces robin_core robin_bringup \
  robin_hardware_ur robin_hardware_opcua robin_hardware_fronius robin_hardware_garmo \
  robin_moveit_config robin_rqt --symlink-install"

# Build single package
cd /home/mil-ai-pc/Repos/ROBIN/vulcanexus_ws && \
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  colcon build --packages-select robin_core --symlink-install"

# Clean build (when symlink errors occur)
cd /home/mil-ai-pc/Repos/ROBIN/vulcanexus_ws && \
docker compose exec vulcanexus bash -c "rm -rf /workspace/ros2_packages/build/robin_core \
  /workspace/ros2_packages/install/robin_core && \
  source /workspace/ros2_packages/ws_setup.sh && \
  colcon build --packages-select robin_core --symlink-install"
```

### Useful Diagnostic Commands

```bash
# List active nodes
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && ros2 node list"

# List active topics
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && ros2 topic list"

# List active services
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && ros2 service list"

# Monitor Fronius welder data
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 topic echo /fronius/display_current"

# Check PLC status
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 topic echo /wago/out/power_source_ready"
```

---

## Plate Slot Reservation — Reusing Partially-Used Plates

When reusing a plate that already has beads from previous experiments, mark the
occupied regions so the planner skips them. Reservations are stored in
`beads.json` (default: `robin_bringup/config/beads.json`) alongside
completed bead records and are automatically excluded during experiment planning
via **physical coordinate overlap checking** — this correctly prevents
collisions even when experiments use different bead spacing.

### How It Works

Every completed bead and every reservation stores its **plate-local inward
coordinates** (`local_x_start`, `local_x_end`, `local_y`). When a new
experiment is planned, each candidate slot is checked for physical overlap
against all existing records. Two footprints conflict when their X segments
intersect **and** their Y centre-lines are closer than the current `spacing_y`.

This means you can safely:
- Run experiment 1 with `spacing_y=0.03` (beads at y=0.04, 0.07, 0.10)
- Run experiment 2 with `spacing_y=0.05` on the same plate — the planner will
  skip any position that would physically overlap an existing bead.

### Coordinate System

Coordinates are in the **plate-local inward frame** (metres from the touched
corner, positive inward along length and width):

- `x_start` / `x_end` — bead extent along plate length
- `y` — bead centre-line along plate width

Example for a 300×300 mm plate with 40 mm margins and 100 mm bead length:
- Row 0: `x_start=0.04, x_end=0.14, y=0.04`
- Row 1: `x_start=0.04, x_end=0.14, y=0.07` (with spacing_y=0.03)

### Adding Reservations

Two modes are supported via the `/plate/reserve_slots` service:

#### Mode A — Physical Coordinates (preferred)

Directly specify the inward coordinates of each region to reserve:

```bash
# Reserve two regions on plate_A by physical coordinates
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 service call /plate/reserve_slots robin_interfaces/srv/ReserveSlots \
  '{plate_id: \"plate_A\", x_starts: [0.04, 0.16, 0.04, 0.16, 0.04, 0.16, 0.04], x_ends: [0.14, 0.26, 0.14, 0.26, 0.14, 0.26, 0.14], y_centers: [0.04, 0.04, 0.06, 0.06, 0.08, 0.08, 0.1]}'"
```

Each entry in `x_starts`, `x_ends`, `y_centers` defines one reserved bead
footprint. Arrays must be the same length. Already-occupied positions (within
1 mm) are silently skipped.

#### Mode B — Grid Indices + Spacing

Specify `(row, col)` indices together with the spacing parameters used when
those beads were placed. The service computes the physical coordinates for you:

```bash
# Reserve slots (0,0) and (1,0) using grid mode — spacing params are required
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 service call /plate/reserve_slots robin_interfaces/srv/ReserveSlots \
  '{plate_id: \"plate_A\", slot_rows: [0, 1], slot_cols: [0, 0], \
    spacing_x: 0.03, spacing_y: 0.03, margin_x: 0.04, margin_y: 0.04, \
    bead_length: 0.1, plate_length: 0.3, plate_width: 0.3}'"
```

> **Important:** In grid mode you **must** provide the spacing/margin/bead_length
> that match the original experiment. The grid indices alone are not enough
> because different spacing parameters produce different physical positions.

### Removing Reservations

Reservations are removed via the `/plate/clear_reserved_slots` service.
Only records with `reserved: true` are removed — completed bead records are
**never** affected.

#### Clear All Reservations for a Plate

```bash
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 service call /plate/clear_reserved_slots robin_interfaces/srv/ClearReservedSlots \
  '{plate_id: \"plate_A\"}'"
```

#### Clear All Reservations Across All Plates

```bash
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 service call /plate/clear_reserved_slots robin_interfaces/srv/ClearReservedSlots \
  '{all_plates: true}'"
```

#### Clear Specific Reservations by bead_id

Each reservation is assigned a `bead_id` (e.g. `reserved_plate_A_0`). You can
clear individual reservations by name:

```bash
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
  ros2 service call /plate/clear_reserved_slots robin_interfaces/srv/ClearReservedSlots \
  '{bead_ids: [\"reserved_plate_A_0\"]}'"
```

### Inspecting the beads.json File

The file lives at `robin_bringup/config/beads.json` (inside the container:
`/workspace/ros2_packages/src/robin_bringup/config/beads.json`). You can
inspect it directly:

```bash
docker compose exec vulcanexus bash -c "cat /workspace/ros2_packages/src/robin_bringup/config/beads.json | python3 -m json.tool"
```

Each record contains:

| Field | Description |
|-------|-------------|
| `bead_id` | Unique identifier (e.g. `B001` or `reserved_plate_A_0`) |
| `plate_id` | Which plate the bead is on |
| `local_x_start` | Inward X start (m) |
| `local_x_end` | Inward X end (m) |
| `local_y` | Inward Y centre-line (m) |
| `slot_row` / `slot_col` | Grid indices (for display; `-1` if not applicable) |
| `reserved` | `true` for reservations, absent for completed beads |

### Removing Completed Beads

Completed bead records are **not** removable via the ROS2 service (safety
measure). To remove them, edit `beads.json` directly — delete the relevant
entries from the `beads` array and save. The planner will pick up the changes
on the next planning call.

```bash
# Edit beads.json inside the container
docker compose exec vulcanexus bash -c "nano /workspace/ros2_packages/src/robin_bringup/config/beads.json"
```

Or reset the entire file to start fresh:

```bash
docker compose exec vulcanexus bash -c "echo '{\"beads\": []}' > /workspace/ros2_packages/src/robin_bringup/config/beads.json"
```

### Visualization

Occupied beads are shown in RViz as thin cylinders on the plate surface:
- **Orange** — reserved slots
- **Copper** — completed beads

The visualization reads physical coordinates directly from `beads.json`, so
markers always appear at the correct position regardless of the current spacing
settings.

---

## Manual Welding — Bypass Automated Experiment Planning

To weld at a specific position without using the experiment planner, send `ExecuteBead` action goals directly to the **planner node**. This skips the experiment planning, approval, and multi-bead orchestration entirely.

### Option 1: Direct `ExecuteBead` Action Goal (CLI)

Define your weld bead as a straight line from start to end point in `base_link` frame:

```bash
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
ros2 action send_goal /execute_bead robin_interfaces/action/ExecuteBead '{
  bead_id: \"manual_test_1\",
  plate_id: \"manual\",
  path: [
    {x: -1.20, y: 0.45, z: -0.178},
    {x: -1.10, y: 0.45, z: -0.178}
  ],
  total_length: 0.10,
  target_speed: 0.005,
  primary_parameter: 0,
  primary_value: 120.0,
  target_current: 120.0,
  target_voltage: 18.0,
  wire_feed_speed: 5.0,
  requested_stickout: 0.015,
  scan_speed: 0.0,
  dry_run: false
}' --feedback"
```

**Key fields to set:**
- `path`: Array of `geometry_msgs/Point` — weld start and end positions in `base_link` frame (meters). Minimum 2 points. Get coordinates from RViz or `FindSurface`.
- `target_speed`: TCP travel speed during welding (m/s). Typical: 0.003–0.010.
- `primary_parameter`: 0=CURRENT, 1=VOLTAGE, 2=WIRE_FEED_SPEED
- `primary_value`: Value for the primary parameter
- `target_current` / `target_voltage` / `wire_feed_speed`: Override values (0.0 = synergetic)
- `dry_run: true`: Motion only, no arc — **use this first to verify the path!**

### Option 2: Dry Run First (Recommended)

Always test with `dry_run: true` first to verify the robot path is correct:

```bash
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
ros2 action send_goal /execute_bead robin_interfaces/action/ExecuteBead '{
  bead_id: \"test_dryrun\",
  plate_id: \"manual\",
  path: [
    {x: -1.20, y: 0.45, z: -0.178},
    {x: -1.10, y: 0.45, z: -0.178}
  ],
  total_length: 0.10,
  target_speed: 0.005,
  primary_parameter: 0,
  primary_value: 120.0,
  target_current: 120.0,
  target_voltage: 18.0,
  wire_feed_speed: 5.0,
  requested_stickout: 0.015,
  scan_speed: 0.0,
  dry_run: true
}' --feedback"
```

### Step-by-Step Manual Weld Workflow

1. **Find your surface Z** — probe the plate surface at the desired XY:
   ```bash
   docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
   ros2 service call /calibration/find_surface robin_interfaces/srv/FindSurface \
     '{x: -1.20, y: 0.45, z_start: -0.10, z_limit: -0.25, probe_speed: 0.005}'"
   ```
   Note the `surface_z` from the response.

2. **Dry run** to verify path (use the discovered `surface_z`):
   ```bash
   ros2 action send_goal /execute_bead robin_interfaces/action/ExecuteBead '{
     bead_id: "manual_dryrun", plate_id: "manual",
     path: [{x: -1.20, y: 0.45, z: <surface_z>}, {x: -1.10, y: 0.45, z: <surface_z>}],
     total_length: 0.10, target_speed: 0.005,
     primary_parameter: 0, primary_value: 120.0,
     target_current: 120.0, target_voltage: 18.0, wire_feed_speed: 5.0,
     requested_stickout: 0.015, scan_speed: 0.0, dry_run: true
   }' --feedback
   ```

3. **Weld** — change `dry_run` to `false`:
   ```bash
   # Same command but with dry_run: false
   ```

4. **Cancel mid-weld** (if needed):
   ```bash
   docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
   ros2 action send_goal /execute_bead robin_interfaces/action/ExecuteBead --cancel"
   ```
   Or call the terminate service:
   ```bash
   docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && \
   ros2 service call /experiment/terminate std_srvs/srv/Trigger"
   ```

### Option 3: Use the Operator Panel (GUI)

The **Manual tab** in the RQt operator panel allows direct control of:
- Working mode selection
- Fronius parameter setting (current, voltage, wire speed)
- Wire forward/backward
- Robot ready signal
- Weld start/stop

Combined with **Cartesian jogging** (Jog component), you can:
1. Jog the robot to your desired position
2. Set welding parameters on the Manual tab
3. Start welding manually

### Getting Coordinates for Manual Welding

- **From RViz:** Read TCP position from the TF display or interactive markers
- **From TF:** `ros2 topic echo /tf --once` and look for `wire_tip` → `base_link`
- **From FindSurface:** Use the calibration service to probe surface Z at known XY
- **From plates.json:** The plate corner positions and dimensions are in `robin_bringup/config/plates.json`
