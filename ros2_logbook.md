# ROBIN MVP - Technical Report (ROS 2 slice)

*This notebook is a living technical logbook for the ROS 2 side of the MVP. It explains what exists, how it works, how to run it, and what remains. Read it as if a senior engineer were handing the project to a junior engineer.*

---

## 1) Executive overview

This ROS 2 layer connects a Universal Robots arm, a Garmo profilometer, and a visualization stack (TF, RViz, PointCloud2). It provides:
- **Interfaces**: custom services for sensor activation (`robin_interfaces`).
- **Hardware adapters**: a driver/shim for the Garmo profilometer (`robin_hardware_garmo`).
- **Robot scene**: a UR scene Xacro with UR robot, weld torch, platform, table, and the profilometer mounted on the torch (`robin_hardware_ur`).
- **Launch flows**: standalone sensor visualization and integrated robot+sensor bringup.

The intent is to stream live point clouds from the profilometer while keeping TFs and frames coherent with the robot model, so the data lands correctly in the scene and can later be transformed into FIWARE entities.

---

## 2) Architecture, in plain language

- **ROS 2 graph.** Nodes include a sensor command service and a sensor data publisher. The command node exposes services to start/stop the profilometer; the data node reads ASCII frames from a TCP socket and publishes them as `sensor_msgs/PointCloud2`. A TF broadcaster optionally publishes the sensor frame relative to a parent frame.
- **UR scene.** A Xacro (`robin_scene.urdf.xacro`) includes the UR macro, ros2_control, and meshes for the weld torch, table, and platform. The profilometer is mounted to the torch tip via a macro include from the Garmo package, aligning the frames so RViz can render the point cloud where the sensor actually is.
- **Launch integration.** Standalone sensor launch files exist for quick bring‑up and visualization; an integrated launch (`robot.launch.py`) wires UR driver bring‑up with the Garmo sensor and defers sensor activation until the robot stack is running.

---

## 3) Packages and responsibilities

- **`robin_interfaces`**: defines service contracts.
  - `SensorCommand.srv`: request `{sensor_ip, ctrl_port, data_port, cmd, fps}`, response `{success, message}`.
  - `Trigger.srv`: request is empty; response `{success, message}`.
  - Built with `rosidl_default_generators`; service names used downstream: `/profilometer_activate`, `/profilometer_deactivate`.

- **`robin_hardware_garmo`**: profilometer integration.
  - Node `sensor_cmd` (`robin_hardware_garmo/sensor_cmd.py`):
    - Provides services: `/profilometer_activate`, `/profilometer_deactivate` (type `robin_interfaces/srv/SensorCommand`).
    - Sends CSV byte commands over TCP control port (default 5020) to set FPS, set joint, start/stop tracking.
    - Parameters: `sensor_ip`, `ctrl_port`, `data_port`, `fps`.
  - Node `sensor_data` (`robin_hardware_garmo/sensor_data.py`):
    - Connects to TCP data stream (default port 66), parses ASCII lines into XYZ points, publishes `PointCloud2`.
    - Parameters: `sensor_ip`, `data_port`, `frame_id`, `parent_frame`, `frame_xyz`, `frame_rpy`, `broadcast_hz`, `topic` (default `/robin/pointcloud`), `units` (raw/um/mm/m), `reconnect_interval`, `log_publish_debug`.
    - Optionally broadcasts TF from `parent_frame`→`frame_id`. If those frames are equal, broadcasting is skipped (use robot_state_publisher instead).
  - Launch:
    - `launch/sensor.launch.py`: parameterized bring‑up for command + data nodes, logs resolved config, and (after a delay) calls `/profilometer_activate` via `ros2 service call`.
    - `launch/view_sensor.launch.py`: builds a small URDF scene using the `garmo_garline` macro, runs `robot_state_publisher`, starts RViz with `rviz/debug_garmo.rviz`, starts nodes, and activates the sensor.

- **`robin_hardware_ur`**: UR scene and integrated bring‑up.
  - URDF/Xacro: `urdf/robin_scene.urdf.xacro` includes UR macros and meshes for weld torch, table, and a platform; it also includes and mounts the Garmo sensor macro on the `weld_torch_tip` using a fixed transform (see origin in the Xacro).
  - Launch:
    - `launch/rsp.launch.py`: generates `robot_description` by xacro‑processing `robin_scene.urdf.xacro` and starts `robot_state_publisher`.
    - `launch/robot.launch.py`: includes `ur_robot_driver/launch/ur_control.launch.py` with `description_launchfile:=rsp.launch.py`, and includes `robin_hardware_garmo/launch/sensor.launch.py` with a start delay so the robot stack is ready before activating the sensor.
  - Config: `config/robot_kinematics.yaml` with calibration parameters.

- **`robin_core_sensor`**: generic/placeholder sensor utilities.
  - Launch: `launch/view_sensor.launch.py` for a generic sensor pipeline in this package namespace.
  - Note: No `console_scripts` are exported here yet; this launch is illustrative unless nodes are added.

---

## 4) Topics, frames, and services

- **Topics**
  - `/robin/pointcloud` (`sensor_msgs/PointCloud2`) - emitted by `sensor_data`.
- **TF frames**
  - `frame_id` (default `garmo_laser_frame`) - the sensor’s child frame.
  - `parent_frame` (default `world` or `base_link`) - TF parent. The data node can broadcast `parent_frame`→`frame_id` based on `frame_xyz`/`frame_rpy`.
  - In the integrated robot launch, `frame_id` and `parent_frame` are both set to the macro’s `laser_frame` to disable the sensor’s TF broadcaster and rely entirely on `robot_state_publisher` (from URDF).
- **Services**
  - `/profilometer_activate` (`robin_interfaces/srv/SensorCommand`) - sets FPS, prepares joint mode, starts tracking.
  - `/profilometer_deactivate` (`robin_interfaces/srv/SensorCommand`) - stops tracking.

---

## 5) Runtime behavior and data flow

1) Bring‑up launches `sensor_cmd` and `sensor_data`.
2) `sensor_data` connects to the sensor’s TCP data stream and begins parsing frames into arrays of points, scaled into configured units.
3) Every completed frame publishes a `PointCloud2` on `/robin/pointcloud`.
4) Depending on configuration:
   - Either `sensor_data` broadcasts a TF transform from `parent_frame` to `frame_id` at `broadcast_hz`.
   - Or TFs come from `robot_state_publisher` when the URDF defines the sensor link (integrated robot path).
5) After a delay, the launch file triggers `/profilometer_activate` to set FPS and start stream on the control port.

---

## 6) URDF scene and mounting

- `robin_hardware_ur/urdf/robin_scene.urdf.xacro`:
  - Includes UR’s macros and ros2_control xacros.
  - Adds links: `weld_torch`, `weld_torch_tip`, `weld_table`, `robot_platform` with meshes and inertias.
  - Mounts `weld_torch` to `$(arg ur_type)_tool0` via a fixed joint.
  - Mounts `weld_torch_tip` to `weld_torch` via a fixed joint with a pitch offset; a small red sphere marks the torch tip.
  - Includes the `garmo_garline` sensor macro and attaches it to `weld_torch_tip` with a fixed `origin` so the laser frame aligns with the tool geometry.

This guarantees that when the robot moves, the sensor frame follows, and point clouds render at the correct position in RViz.

---

## 7) Launch files demystified

- `robin_hardware_garmo/launch/sensor.launch.py`
  - Declares args for IP, ports, FPS, frame IDs, pose (xyz/rpy), TF broadcast rate, and activation delay.
  - Starts `sensor_cmd` and `sensor_data` with parameters.
  - Logs resolved configuration and calls `/profilometer_activate` after `activate_delay`.

- `robin_hardware_garmo/launch/view_sensor.launch.py`
  - Builds a mini‑URDF scene from the Garmo macro, starts `robot_state_publisher` + RViz, and the two nodes; then activates the sensor.

- `robin_hardware_ur/launch/rsp.launch.py`
  - Xacro‑processes the full scene into `robot_description` and starts `robot_state_publisher`.

- `robin_hardware_ur/launch/robot.launch.py`
  - Brings up `ur_robot_driver` control with `rsp.launch.py` as the description source.
  - Starts the Garmo sensor launch after a configurable delay; sets `frame_id` and `parent_frame` equal so TF is sourced from URDF only.

---

## 8) How to build and run

Assumes ROS 2 (Humble or newer) with `colcon` installed.

- **Create a workspace and add this repository**
```bash
mkdir -p ~/robin_ws/src
# Option A: copy or symlink the ros2_packages folder into the workspace src
ln -s /Users/orpheus/Documents/Work/3DC/ROBIN2/ROBIN/ros2_packages ~/robin_ws/src/
cd ~/robin_ws
colcon build --symlink-install
source install/setup.bash
```

- **Standalone sensor visualization** (URDF+RViz from Garmo macro):
```bash
ros2 launch robin_hardware_garmo view_sensor.launch.py \
  sensor_ip:=192.168.1.212 ctrl_port:=5020 data_port:=66 fps:=42
```

- **Integrated robot + sensor bring‑up**:
```bash
ros2 launch robin_hardware_ur robot.launch.py \
  ur_type:=ur10e robot_ip:=192.168.1.101 sensor_ip:=192.168.1.212
```

- **Manual service calls** (if needed):
```bash
# Activate
ros2 service call /profilometer_activate robin_interfaces/srv/SensorCommand \
  '{sensor_ip: "192.168.1.212", ctrl_port: 5020, data_port: 66, fps: 42}'

# Deactivate
ros2 service call /profilometer_deactivate robin_interfaces/srv/SensorCommand \
  '{sensor_ip: "192.168.1.212", ctrl_port: 5020, data_port: 66, fps: 42}'
```

- **Inspect data**:
```bash
ros2 topic echo /robin/pointcloud  # will print headers; use RViz to visualize
```

Tips:
- If RViz doesn’t show the cloud, ensure fixed frame matches the TF tree. In integrated mode, use a frame from the URDF (e.g., `world` or `base_link`), and confirm the sensor link exists.
- If the data stream disconnects, the node will attempt to reconnect periodically (see `reconnect_interval`).

---

## 9) Operational notes and safety

- Network: ensure the robot and sensor IPs are reachable from your ROS machine; avoid NAT/firewalls blocking TCP 5020 (control) and 66 (data).
- FPS: the command service converts FPS into a frame period; invalid or zero values are rejected.
- TF loops: do not broadcast a TF for `parent_frame==frame_id` (already guarded). Prefer a single source of truth for TFs.
- Units: convert raw units to µm/mm/m via the `units` param; default is meters.

---

## 10) Status and what’s still missing

What’s done:
- Service layer for profilometer start/stop.
- PointCloud2 publisher with reconnection logic and optional TF broadcasting.
- UR scene with mounted sensor, meshes for torch/table/platform, and integrated bring‑up with `ur_robot_driver`.

What remains:
- Export `console_scripts` for any nodes to be provided by `robin_core_sensor` (currently none).
- Verify and tune the fixed transform from torch tip to sensor in the Xacro against real hardware.
- Add CI for message/service generation and smoke tests for launches.
- Provide a sample RViz config for the integrated scene (UR + sensor).

---

## 11) Quick reference: commands

```bash
# Build
cd ~/robin_ws && colcon build --symlink-install && source install/setup.bash

# Standalone Garmo visualization
ros2 launch robin_hardware_garmo view_sensor.launch.py

# Integrated UR + Garmo bring‑up
ros2 launch robin_hardware_ur robot.launch.py

# Services
ros2 service call /profilometer_activate robin_interfaces/srv/SensorCommand '{sensor_ip: "192.168.1.212", ctrl_port: 5020, data_port: 66, fps: 42}'
ros2 service call /profilometer_deactivate robin_interfaces/srv/SensorCommand '{sensor_ip: "192.168.1.212", ctrl_port: 5020, data_port: 66, fps: 42}'
```

---

### Closing note
This ROS 2 slice provides the kinematic/scene foundation and live sensing path to later translate into FIWARE entities and close the loop with the Alert Engine. Once validated on hardware, the next steps are wiring Measurement/Recommendation entities and aligning frames and units across ROS and FIWARE components.
