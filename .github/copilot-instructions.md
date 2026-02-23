# ROBIN Project - Copilot Instructions

## ROS2/Vulcanexus Development

### Building Packages

When running commands inside the Vulcanexus Docker container, **always source `ws_setup.sh`** instead of manually sourcing individual setup files:

```bash
# CORRECT - use ws_setup.sh
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && <command>"

# WRONG - don't manually source
docker compose exec vulcanexus bash -c "source /opt/vulcanexus/jazzy/setup.bash && cd /workspace/ros2_packages && <command>"
```

### Example Build Command

```bash
cd /home/milaipc/ros_ws/src/ROBIN/vulcanexus_ws && \
docker compose exec vulcanexus bash -c "source /workspace/ros2_packages/ws_setup.sh && colcon build --packages-select <package_names> --symlink-install"
```

### Clean Build (when symlink errors occur)

```bash
cd /home/milaipc/ros_ws/src/ROBIN/vulcanexus_ws && \
docker compose exec vulcanexus bash -c "rm -rf /workspace/ros2_packages/build/<package> /workspace/ros2_packages/install/<package> && source /workspace/ros2_packages/ws_setup.sh && colcon build --packages-select <package> --symlink-install"
```

## Project Structure

- `/vulcanexus_ws/` - ROS2 workspace (mounted at `/workspace/ros2_packages` in Docker)
- `/vulcanexus_ws/ws_setup.sh` - Sources all required ROS2/Vulcanexus setup files
- `/vulcanexus_ws/src/` - ROS2 packages
