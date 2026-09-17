"""
Handshake de OFFBOARD (armado, hold en el origen, sin despegue) exigiendo el
LiDAR 2D (SLAM -> external vision) como fuente de posicion.

Lanza el stack completo: px4_drone_slam (LD19 + slam_toolbox) + MicroXRCEAgent
+ ev_odometry_bridge (via ev_odometry_bridge.launch.py) + ev_offboard_handshake.

Requiere EKF2_EV_CTRL=9 en el FC (ver
px4_drone_dev/docs/2026-09-16_test_lidar2d_ev_offboard.md). Puede correr sin
helices: nunca comanda un despegue.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hold_seconds_arg = DeclareLaunchArgument(
        'hold_seconds', default_value='10.0',
        description='Segundos armado antes de desarmar solo (si no se interrumpe antes).',
    )
    topic_version_suffix_arg = DeclareLaunchArgument(
        'topic_version_suffix', default_value='',
        description=(
            'Sufijo de version de mensaje que agrega el firmware a ciertos topics. '
            '"" (vacio, default) para firmware v1.14 (FC actual).'
        ),
    )

    bridge_share = get_package_share_directory('px4_drone')
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, 'launch', 'ev_odometry_bridge.launch.py')
        ),
        launch_arguments={
            'topic_version_suffix': LaunchConfiguration('topic_version_suffix'),
        }.items(),
    )

    ev_offboard_handshake = Node(
        package='px4_drone',
        executable='ev_offboard_handshake',
        name='ev_offboard_handshake',
        output='screen',
        parameters=[{
            'hold_seconds': LaunchConfiguration('hold_seconds'),
            'topic_version_suffix': LaunchConfiguration('topic_version_suffix'),
        }],
    )

    return LaunchDescription([
        hold_seconds_arg,
        topic_version_suffix_arg,
        bridge_launch,
        ev_offboard_handshake,
    ])
