# ROBIN Quick Reference - ROS2 & Docker Commands

## Docker Commands

### Container Management

```bash
# Start all services (detached)
docker compose up -d

# Stop all services
docker compose down

# Restart a specific container
docker restart vulcanexus-bridge

# Get a shell inside the container
docker exec -it vulcanexus-bridge bash

# Get a root shell (for installing packages)
docker exec -it --user root vulcanexus-bridge bash

# View container logs
docker logs vulcanexus-bridge
docker logs -f vulcanexus-bridge  # Follow mode

# Check running containers
docker ps

# Check all containers (including stopped)
docker ps -a
```

### Building Inside Container

```bash
# Always source the setup first
source ws_setup.sh

# Build all packages
colcon build --symlink-install

# Build specific package(s)
colcon build --symlink-install --packages-select robin_hardware_opcua

# Clean build a package (removes build artifacts first)
rm -rf build/<package> install/<package>
colcon build --symlink-install --packages-select <package>

# Build with verbose output
colcon build --symlink-install --event-handlers console_direct+
```

---

## ROS2 Commands

### Node & Topic Inspection

```bash
# List all running nodes
ros2 node list

# List all topics
ros2 topic list

# Show topic info (type, publishers, subscribers)
ros2 topic info /fronius/actual_current

# Echo topic data
ros2 topic echo /fronius/actual_current

# Check topic publish rate
ros2 topic hz /fronius/actual_current

# Show message type definition
ros2 interface show std_msgs/msg/Float32
ros2 interface show robin_interfaces/srv/StartWeld
```

### Service Commands

```bash
# List all services
ros2 service list

# Show service type
ros2 service type /welding/start

# Call a service
ros2 service call /welding/start robin_interfaces/srv/StartWeld \
  "{current: 150.0, voltage: 22.0, wire_speed: 8.0}"

ros2 service call /welding/stop std_srvs/srv/Trigger

ros2 service call /wago/gas_on std_srvs/srv/SetBool "{data: true}"

ros2 service call /fronius/set_current robin_interfaces/srv/SetFloat32 "{data: 150.0}"
```

### Launch Commands

```bash
# Main robot bringup
ros2 launch robin_core_bringup robin_main.launch.py

# OPC UA bridge only
ros2 launch robin_hardware_opcua opcua_bridge.launch.py

# Welding coordinator only
ros2 launch robin_hardware_fronius welding_coordinator.launch.py

# With arguments
ros2 launch robin_core_bringup robin_main.launch.py use_fronius:=false
```

### Action Commands

```bash
# Send weld experiment action goal
ros2 action send_goal /weld_experiment robin_interfaces/action/WeldExperiment \
  "{weld_beads: [
    {start_point: {x: -1.3, y: -0.4, z: -0.18}, end_point: {x: -1.2, y: -0.4, z: -0.18}, target_speed: 0.01, target_voltage: 15.0, target_current: 125.0, wire_feed_speed: 5.0}
  ]}" --feedback

# List action servers
ros2 action list

# Show action type
ros2 action info /weld_experiment
```

### Parameter Commands

```bash
# List all parameters of a node
ros2 param list /opcua_bridge

# Get a parameter value
ros2 param get /opcua_bridge config_file

# Set a parameter
ros2 param set /node_name param_name value
```

---

## ROBIN-Specific Commands

### Welding Operations

```bash
# Start welding (sets params + activates gas/robot_ready/welding)
ros2 service call /welding/start robin_interfaces/srv/StartWeld \
  "{current: 150.0, voltage: 22.0, wire_speed: 8.0}"

# Stop welding
ros2 service call /welding/stop std_srvs/srv/Trigger

# Set parameters only (no start)
ros2 service call /welding/set_params robin_interfaces/srv/StartWeld \
  "{current: 150.0, voltage: 22.0, wire_speed: 8.0}"
```

### OPC UA Bridge (Low-Level)

```bash
# Fronius parameter control
ros2 service call /fronius/set_current robin_interfaces/srv/SetFloat32 "{data: 190.0}"
ros2 service call /fronius/set_voltage robin_interfaces/srv/SetFloat32 "{data: 22.0}"
ros2 service call /fronius/set_wire_speed robin_interfaces/srv/SetFloat32 "{data: 8.0}"

# WAGO PLC signals
ros2 service call /wago/gas_on std_srvs/srv/SetBool "{data: true}"
ros2 service call /wago/robot_ready std_srvs/srv/SetBool "{data: true}"
ros2 service call /wago/welding_start std_srvs/srv/SetBool "{data: true}"
ros2 service call /wago/wire_forward std_srvs/srv/SetBool "{data: true}"
ros2 service call /wago/wire_backward std_srvs/srv/SetBool "{data: true}"

# Monitor Fronius data
ros2 topic echo /fronius/actual_current
ros2 topic echo /fronius/actual_voltage
ros2 topic echo /fronius/actual_power
```

### Sensor Operations

```bash
# Activate Garmo profilometer
ros2 service call /profilometer_activate std_srvs/srv/Trigger

# Deactivate
ros2 service call /profilometer_deactivate std_srvs/srv/Trigger

# View pointcloud data
ros2 topic echo /robin/pointcloud
```

---

## Debugging Commands

### Process Inspection

```bash
# Check if node is running
ros2 node info /opcua_bridge

# Check node's topics/services
ros2 node info /welding_coordinator

# Graph visualization (requires rqt)
ros2 run rqt_graph rqt_graph
```

### Logging

```bash
# Set log level for a node
ros2 service call /opcua_bridge/set_logger_level rcl_interfaces/srv/SetLoggerLevel \
  "{logger_name: 'opcua_bridge', level: 10}"  # 10=DEBUG, 20=INFO, 30=WARN

# View ROS logs
cat ~/.ros/log/latest/launch.log
```

### Network Debugging

```bash
# Check DDS discovery
ros2 daemon status
ros2 daemon stop
ros2 daemon start

# List all discovered endpoints
ros2 topic list -t  # Shows types
ros2 service list -t
```

---

## Common Workflows

### Fresh Start

```bash
# From host machine
docker compose down
docker compose up -d
docker exec -it vulcanexus-bridge bash

# Inside container
source ws_setup.sh
ros2 launch robin_core_bringup robin_main.launch.py
```

### Test OPC UA Bridge Only

```bash
# Terminal 1: Launch bridge
source ws_setup.sh
ros2 launch robin_hardware_opcua opcua_bridge.launch.py

# Terminal 2: Test
ros2 topic hz /fronius/actual_current
ros2 service call /wago/gas_on std_srvs/srv/SetBool "{data: true}"
```

### Test Welding Sequence

```bash
# Terminal 1: OPC UA bridge
ros2 launch robin_hardware_opcua opcua_bridge.launch.py

# Terminal 2: Welding coordinator
ros2 launch robin_hardware_fronius welding_coordinator.launch.py

# Terminal 3: Test
ros2 service call /welding/start robin_interfaces/srv/StartWeld \
  "{current: 150.0, voltage: 22.0, wire_speed: 8.0}"
# ... do welding motion ...
ros2 service call /welding/stop std_srvs/srv/Trigger
```

---

## OPC UA Server Info

| Server | URL | Auth |
|--------|-----|------|
| Fronius | `opc.tcp://192.168.1.104:4840` | Anonymous |
| WAGO PLC | `opc.tcp://192.168.0.17:4840` | `admin:wago` |

---

## Useful Aliases (add to ~/.bashrc)

```bash
# Inside container
alias cb='colcon build --symlink-install'
alias cbs='colcon build --symlink-install --packages-select'
alias src='source install/setup.bash'
alias wsrc='source ws_setup.sh'

# Topics/services
alias tl='ros2 topic list'
alias sl='ros2 service list'
alias nl='ros2 node list'
```
