from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    # default parameters (override via ros2 launch --ros-args -p ...)
    sensor_ip = '192.168.1.212'
    ctrl_port = 5020
    data_port = 66
    fps = 42

    # Node: sensor command service (starts/stops sensor via service)
    sensor_cmd_node = Node(
        package='robin_core_sensor',
        executable='sensor_cmd',
        name='sensor_command_service',
        output='screen',
        parameters=[{
            'sensor_ip': sensor_ip,
            'ctrl_port': ctrl_port,
            'data_port': data_port,
            'fps': fps
        }]
    )

    # Node: data publisher + TF broadcaster
    sensor_data_node = Node(
        package='robin_core_sensor',
        executable='sensor_data',
        name='sensor_data',
        output='screen',
        parameters=[{
            'sensor_ip': sensor_ip,
            'data_port': data_port,
            # Adjust these to match your robot/sensor mounting:
            'frame_id': 'robin_sensor_frame',
            'parent_frame': 'base_link',
            'frame_xyz': [0.0, 0.0, 0.0],
            'frame_rpy': [0.0, 0.0, 0.0],
            'units': 'm',
            'tf_broadcast_hz': 40.0,
        }]
    )

    # locate installed rviz config in share/<pkg>/rviz/debug_garmo.rviz
    rviz_config = Path(get_package_share_directory('robin_core_sensor')) / 'rviz' / 'debug_garmo.rviz'

    # Node: RViz2 (no config supplied; pass -d <path> in arguments if you have an rviz config)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config)],
    )

    # After a short delay, call the profilometer_activate service to start the sensor.
    # This uses the ros2 CLI; adjust the JSON/YAML payload if your service fields differ.
    service_call = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/profilometer_activate',
            'robin_interfaces/srv/SensorCommand',
            '{"sensor_ip":"%s","ctrl_port":%d,"data_port":%d,"fps":%d}' % (sensor_ip, ctrl_port, data_port, fps)
        ],
        output='screen'
    )
    delayed_activate = TimerAction(period=2.0, actions=[LogInfo(msg='Calling profilometer_activate service...'), service_call])

    return LaunchDescription([
        sensor_cmd_node,
        sensor_data_node,
        rviz_node,
        delayed_activate,
    ])
