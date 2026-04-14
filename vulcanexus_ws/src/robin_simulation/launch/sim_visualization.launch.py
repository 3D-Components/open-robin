import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    sim_pkg = get_package_share_directory('robin_simulation')

    xacro_file = os.path.join(sim_pkg, 'urdf', 'vis_ur_fronius_garmo.urdf.xacro')
    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={
            'name': 'ur10e',
            'ur_type': 'ur10e',
            'use_mock_hardware': 'true',
            'mock_sensor_commands': 'false',
        }
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    welding_sim_node = Node(
        package='robin_simulation',
        executable='welding_sim_node',
        name='welding_sim_node',
        output='screen',
    )

    welding_vis_node = Node(
        package='robin_simulation',
        executable='welding_vis_node',
        name='welding_vis_node',
        output='screen',
    )

    rviz_config = os.path.join(sim_pkg, 'rviz', 'welding_sim.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,
        }],
    )

    http_bridge = Node(
        package='welding_http_bridge',
        executable='welding_http_bridge_node',
        name='welding_http_bridge',
        output='screen',
        emulate_tty=True,
    )

    # GUI publishes to /joint_states_manual so the home skill can relay it to
    # /joint_states and override it cleanly during homing without a topic fight.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        remappings=[('joint_states', 'joint_states_manual')],
    )

    home_skill = Node(
        package='welding_home_skill',
        executable='welding_home_skill_node',
        name='welding_home_skill',
        output='screen',
        parameters=[{'use_simulation': True}],
    )

    supervisor = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='welding_supervisor',
                executable='welding_supervisor_node',
                name='welding_supervisor',
                output='screen',
                emulate_tty=True,
            ),
        ],
    )

    return [
        robot_state_publisher,
        joint_state_publisher_gui,
        welding_sim_node,
        welding_vis_node,
        rviz_node,
        foxglove_bridge,
        http_bridge,
        home_skill,
        supervisor,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
