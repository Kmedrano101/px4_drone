from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    confirm_takeoff_arg = DeclareLaunchArgument(
        'confirm_takeoff', default_value='false',
        description='Debe ser "true" para que el nodo realmente despegue.',
    )
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height_m', default_value='2.0',
        description='Metros a subir (ENU, hacia arriba) desde el punto de armado.',
    )
    hold_seconds_arg = DeclareLaunchArgument(
        'hold_seconds', default_value='10.0',
        description='Segundos a mantener posicion antes de aterrizar.',
    )
    topic_version_suffix_arg = DeclareLaunchArgument(
        'topic_version_suffix', default_value='',
        description=(
            'Sufijo de version de mensaje que agrega el firmware a ciertos topics '
            '(vehicle_status, vehicle_local_position). "" (vacio, default) para '
            'firmware v1.14 (FC actual, anterior al versionado de mensajes de PX4). '
            '"_v1" para firmware v1.17 (HKUST_NXT_DUAL) -- en ese caso, ademas, '
            'cambiar la branch de px4_msgs a fc-v17-82e3322e y recompilar.'
        ),
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
        name='micro_xrce_agent',
        output='screen',
    )

    takeoff_position_hold_outdoor = Node(
        package='px4_drone',
        executable='takeoff_position_hold_outdoor',
        name='takeoff_position_hold_outdoor',
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
        micro_xrce_agent,
        takeoff_position_hold_outdoor,
    ])
