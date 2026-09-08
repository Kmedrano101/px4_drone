from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_agent_arg = DeclareLaunchArgument(
        'run_agent', default_value='true',
        description=(
            'Arrancar el MicroXRCEAgent aqui. Ponlo a false si ya lo levanto '
            'sitl.launch.py, o dos agentes pelearan por el puerto 8888.'
        ),
    )

    micro_xrce_agent = ExecuteProcess(
        # SITL: el cliente uXRCE-DDS de PX4 sale por UDP al 8888, no por serie.
        # En el dron real seria: serial --dev /dev/ttyAMA0 -b 921600
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_agent',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_agent')),
    )

    offboard_streamer = Node(
        package='px4_drone',
        executable='offboard_streamer.py',
        name='offboard_streamer',
        output='screen',
    )

    return LaunchDescription([
        run_agent_arg,
        micro_xrce_agent,
        offboard_streamer,
    ])
