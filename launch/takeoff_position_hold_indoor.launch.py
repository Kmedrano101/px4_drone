from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    confirm_takeoff_arg = DeclareLaunchArgument(
        'confirm_takeoff', default_value='false',
        description='Debe ser "true" para que el nodo realmente despegue.',
    )
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height_m', default_value='1.0',
        description='Metros a subir (ENU, hacia arriba) desde el punto de armado.',
    )
    hold_seconds_arg = DeclareLaunchArgument(
        'hold_seconds', default_value='8.0',
        description='Segundos a mantener posicion antes de aterrizar.',
    )
    topic_version_suffix_arg = DeclareLaunchArgument(
        'topic_version_suffix', default_value='_v1',
        description=(
            'Sufijo de version que el firmware anade a los topics de mensajes '
            'versionados. En este SITL (PX4 main) es "_v1": el cliente uXRCE-DDS '
            'publica /fmu/out/vehicle_status_v1 y /fmu/out/vehicle_local_position_v1. '
            'OJO: dds_topics.yaml lista los nombres SIN sufijo, se anade en runtime '
            'segun MESSAGE_VERSION -- mirar el yaml enganna. Verificar siempre con '
            '"ros2 topic list | grep vehicle_local_position". Un nombre equivocado '
            'no da error: da silencio.'
        ),
    )
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

    takeoff_position_hold_indoor = Node(
        package='px4_drone',
        executable='takeoff_position_hold_indoor',
        name='takeoff_position_hold_indoor',
        output='screen',
        parameters=[{
            'confirm_takeoff': LaunchConfiguration('confirm_takeoff'),
            'takeoff_height_m': LaunchConfiguration('takeoff_height_m'),
            'hold_seconds': LaunchConfiguration('hold_seconds'),
            'topic_version_suffix': LaunchConfiguration('topic_version_suffix'),
        }],
    )

    return LaunchDescription([
        confirm_takeoff_arg,
        takeoff_height_arg,
        hold_seconds_arg,
        topic_version_suffix_arg,
        run_agent_arg,
        micro_xrce_agent,
        takeoff_position_hold_indoor,
    ])
