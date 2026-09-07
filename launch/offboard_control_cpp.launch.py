from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_version_suffix_arg = DeclareLaunchArgument(
        'topic_version_suffix', default_value='',
        description=(
            'Sufijo de version de mensaje que agrega el firmware a ciertos topics '
            '(vehicle_status, vehicle_local_position, vehicle_attitude_setpoint). '
            '"" (vacio, default) para firmware v1.14 (FC actual, anterior al '
            'versionado de mensajes de PX4). "_v1" para firmware v1.17 '
            '(HKUST_NXT_DUAL) -- en ese caso, ademas, cambiar la branch de px4_msgs '
            'a fc-v17-82e3322e y recompilar.'
        ),
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
        name='micro_xrce_agent',
        output='screen',
    )

    offboard_control = Node(
        package='px4_drone',
        executable='offboard_control',
        name='offboard_control',
        output='screen',
        parameters=[{
            'topic_version_suffix': LaunchConfiguration('topic_version_suffix'),
        }],
    )

    return LaunchDescription([
        topic_version_suffix_arg,
        micro_xrce_agent,
        offboard_control,
    ])
