from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, ExecuteProcess, OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments (other launchfiles can override these)
    declared_args = [
        DeclareLaunchArgument('sensor_ip', default_value='192.168.1.212', description='Sensor IP'),
        DeclareLaunchArgument('ctrl_port', default_value='5020', description='Control port'),
        DeclareLaunchArgument('data_port', default_value='66', description='Data port'),
        DeclareLaunchArgument('fps', default_value='42', description='Frames per second'),
        # Frame pose components as separate args to keep substitution simple
        DeclareLaunchArgument('frame_xyz_x', default_value='0.0'),
        DeclareLaunchArgument('frame_xyz_y', default_value='0.0'),
        DeclareLaunchArgument('frame_xyz_z', default_value='0.0'),
        DeclareLaunchArgument('frame_rpy_x', default_value='0.0'),
        DeclareLaunchArgument('frame_rpy_y', default_value='0.0'),
        DeclareLaunchArgument('frame_rpy_z', default_value='0.0'),
        DeclareLaunchArgument('frame_id', default_value='garmo_laser_frame'),
        DeclareLaunchArgument('parent_frame', default_value='base_link'),
        DeclareLaunchArgument('units', default_value='m'),
        DeclareLaunchArgument('tf_broadcast_hz', default_value='42.0'),
        DeclareLaunchArgument('activate_delay', default_value='2.0', description='Delay (s) before calling activation service'),
    ]

    # LaunchConfigurations to provide to nodes (can be overridden by includes)
    sensor_ip = LaunchConfiguration('sensor_ip')
    ctrl_port = LaunchConfiguration('ctrl_port')
    data_port = LaunchConfiguration('data_port')
    fps = LaunchConfiguration('fps')

    # Node: sensor command service (starts/stops sensor via service)
    sensor_cmd_node = Node(
        package='robin_hardware_garmo',
        executable='sensor_cmd',
        name='sensor_command_service',
        output='screen', # can be 'log' to capture to file
        parameters=[{
            'sensor_ip': sensor_ip,
            'ctrl_port': ctrl_port,
            'data_port': data_port,
            'fps': fps
        }]
    )

    # Node: data publisher + TF broadcaster (parameters passed through)
    # Build this node at runtime so we can resolve LaunchConfigurations into native
    # Python lists of floats for 'frame_xyz' and 'frame_rpy' (avoids STRING vs DOUBLE_ARRAY)
    def build_sensor_data(context):
        sip = LaunchConfiguration('sensor_ip').perform(context)
        dport = LaunchConfiguration('data_port').perform(context)
        fid = LaunchConfiguration('frame_id').perform(context)
        pframe = LaunchConfiguration('parent_frame').perform(context)
        units_val = LaunchConfiguration('units').perform(context)
        bhz = LaunchConfiguration('tf_broadcast_hz').perform(context)
        # resolve xyz/rpy components and convert to floats
        try:
            xyz = [float(LaunchConfiguration('frame_xyz_x').perform(context)),
                   float(LaunchConfiguration('frame_xyz_y').perform(context)),
                   float(LaunchConfiguration('frame_xyz_z').perform(context))]
        except Exception:
            xyz = [0.0, 0.0, 0.0]
        try:
            rpy = [float(LaunchConfiguration('frame_rpy_x').perform(context)),
                   float(LaunchConfiguration('frame_rpy_y').perform(context)),
                   float(LaunchConfiguration('frame_rpy_z').perform(context))]
        except Exception:
            rpy = [0.0, 0.0, 0.0]

        node = Node(
            package='robin_hardware_garmo',
            executable='sensor_data',
            name='sensor_data',
            output='screen',
            parameters=[{
                'sensor_ip': sip,
                'data_port': int(dport) if dport is not None else 66,
                'frame_id': fid,
                'parent_frame': pframe,
                'frame_xyz': xyz,
                'frame_rpy': rpy,
                'units': units_val,
                'tf_broadcast_hz': float(bhz) if bhz is not None else 0.0,
            }]
        )
        return [node]

    sensor_data_node = OpaqueFunction(function=build_sensor_data)

    # After a short delay, call the profilometer_activate service to start the sensor.
    # Use an OpaqueFunction so LaunchConfigurations are resolved before building the JSON.
    # Optional: log resolved configuration for debugging when included from other launch files
    def log_resolved_config(context):
        ip = LaunchConfiguration('sensor_ip').perform(context)
        ctrl = LaunchConfiguration('ctrl_port').perform(context)
        data = LaunchConfiguration('data_port').perform(context)
        fid = LaunchConfiguration('frame_id').perform(context)
        pframe = LaunchConfiguration('parent_frame').perform(context)
        return [
            LogInfo(msg=f"Sensor config -> ip={ip}, ctrl_port={ctrl}, data_port={data}, frame_id={fid}, parent_frame={pframe}")
        ]

    log_config_action = OpaqueFunction(function=log_resolved_config)

    def build_activation_action(context):
        ip = LaunchConfiguration('sensor_ip').perform(context)
        ctrl = LaunchConfiguration('ctrl_port').perform(context)
        data = LaunchConfiguration('data_port').perform(context)
        fps_val = LaunchConfiguration('fps').perform(context)
        delay_s = LaunchConfiguration('activate_delay').perform(context)
        payload = f'{{"sensor_ip":"{ip}","ctrl_port":{ctrl},"data_port":{data},"fps":{fps_val}}}'
        service_call = ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/profilometer_activate',
                'robin_interfaces/srv/SensorCommand',
                payload
            ],
            output='screen'
        )
        try:
            period = float(delay_s)
        except Exception:
            period = 2.0
        return [TimerAction(period=period, actions=[LogInfo(msg='Calling profilometer_activate service...'), service_call])]

    delayed_activate = OpaqueFunction(function=build_activation_action)

    # Assemble the launch description
    ld = LaunchDescription()
    for arg in declared_args:
        ld.add_action(arg)
    ld.add_action(sensor_cmd_node)
    ld.add_action(log_config_action)
    ld.add_action(sensor_data_node)
    ld.add_action(delayed_activate)
    return ld