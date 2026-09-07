from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_version_suffix_arg = DeclareLaunchArgument(
        'topic_version_suffix', default_value='',
        description=(
            'Sufijo de version de mensaje para vehicle_status. "" (vacio, default) '
            'para firmware v1.14 (FC actual). "_v1" para firmware v1.17 (HKUST_NXT_DUAL).'
        ),
    )
    hold_seconds_arg = DeclareLaunchArgument(
        'hold_seconds', default_value='5.0',
        description=(
            'Segundos armado en OFFBOARD antes de que el nodo pida el desarme por su cuenta. '
            'Subir este valor para tener mas tiempo de activar el kill switch del RC a mano -- '
            'si el kill switch desarma antes de que se cumpla este tiempo, el nodo lo detecta '
            'como desarme externo y termina el test ahi mismo (no espera el resto de la ventana).'
        ),
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
        name='micro_xrce_agent',
        output='screen',
    )

    offboard_position_handshake = Node(
        package='px4_drone',
        executable='offboard_position_handshake',
        name='offboard_position_handshake',
        output='screen',
        parameters=[{
            'topic_version_suffix': LaunchConfiguration('topic_version_suffix'),
            'hold_seconds': LaunchConfiguration('hold_seconds'),
        }],
    )

    return LaunchDescription([
        topic_version_suffix_arg,
        hold_seconds_arg,
        micro_xrce_agent,
        offboard_position_handshake,
    ])
