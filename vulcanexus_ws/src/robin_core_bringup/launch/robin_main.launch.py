from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configurations
    robot_ip = LaunchConfiguration("robot_ip")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    sensor_ip = LaunchConfiguration("sensor_ip")
    use_sensor = LaunchConfiguration("use_sensor")
    use_fronius = LaunchConfiguration("use_fronius")
    sensor_start_delay = LaunchConfiguration("sensor_start_delay")
    is_simulation = LaunchConfiguration("is_simulation")

    declared_arguments = [
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([FindPackageShare("robin_core_bringup"), "rviz", "view_scene.rviz"]),
            description="Path to RViz config file.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.101",
            description="IP address by which the robot can be reached.",
        ),
        DeclareLaunchArgument(
            "sensor_ip",
            default_value="192.168.1.212",
            description="IP address of the Garmo sensor.",
        ),
        DeclareLaunchArgument(
            "sensor_start_delay",
            default_value="3.0",
            description="Delay (seconds) before starting the Garmo sensor to allow UR driver to initialize.",
        ),
        DeclareLaunchArgument(
            "use_sensor",
            default_value="true",
            description="Enable Garmo sensor",
        ),
        DeclareLaunchArgument(
            "use_fronius",
            default_value="true",
            description="Enable Fronius welder",
        ),
        DeclareLaunchArgument(
            "is_simulation",
            default_value="false",
            description="Whether to launch the robot in simulation mode.",
        ),
    ]

    # 1. UR Robot (driver + robot_state_publisher)
    ur_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("robin_hardware_ur"), "launch", "robot.launch.py"])
        ]),
        launch_arguments={
            "robot_ip": robot_ip,
            "is_simulation": is_simulation,
        }.items(),
    )

    # 1b. MoveIt Planner
    moveit_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("robin_moveit_control"), "launch", "moveit_planner.launch.py"])
        ]),
    )

    # 2. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # 2. Garmo Sensor (hardware driver)
    garmo_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("robin_hardware_garmo"), "launch", "sensor.launch.py"])
        ]),
        condition=IfCondition(use_sensor),
        launch_arguments={
            "sensor_ip": sensor_ip,
            "ctrl_port": "5020",
            "data_port": "66",
            "fps": "42",
            # Use the same frame names as URDF (macro creates 'laser_frame')
            # Set parent_frame==frame_id to disable sensor_data TF broadcast and rely on robot_state_publisher
            "frame_id": "laser_frame",
            "parent_frame": "laser_frame",
            "units": "m",
            "tf_broadcast_hz": "42.0",
            "activate_delay": "2.0",
        }.items(),
    )

    # Start the Garmo sensor after a configurable delay so UR driver can initialize first
    delayed_garmo_sensor = TimerAction(
        period=sensor_start_delay,
        actions=[garmo_sensor],
    )

    # 3. Sensor Data Processing
    sensor_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("robin_core_sensor"), "launch", "process_data.launch.py"])
        ]),
        condition=IfCondition(use_sensor),
    )

    # 4. OPC UA Bridge (required for Fronius + WAGO communication)
    opcua_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("robin_hardware_opcua"), "launch", "opcua_bridge.launch.py"])
        ]),
        condition=IfCondition(use_fronius),
    )

    # 5. Welding Coordinator (Fronius + WAGO) - depends on OPC UA bridge
    welding_coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("robin_hardware_fronius"), "launch", "welding_coordinator.launch.py"])
        ]),
        condition=IfCondition(use_fronius),
    )

    # 6. Weld Data Node (progression tracking) - depends on welding coordinator
    weld_data = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("robin_core_data"), "launch", "weld_data.launch.py"])
        ]),
        condition=IfCondition(use_fronius),
    )

    return LaunchDescription(
        declared_arguments
        + [
            ur_robot,
            moveit_planner,
            rviz_node,
            delayed_garmo_sensor,
            sensor_processing,
            opcua_bridge,
            welding_coordinator,
            weld_data,
        ]
    )
