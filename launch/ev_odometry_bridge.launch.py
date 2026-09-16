"""
Puente EV (LiDAR 2D -> EKF2), fase 1: solo el bridge + verificacion, sin
ningun nodo que arme el dron.

Lanza: el stack de SLAM completo de px4_drone_slam (LD19 + tf estatica +
slam_toolbox, sin cambios) + MicroXRCEAgent + ev_odometry_bridge, que
traduce la tf map->base_link a VehicleOdometry en
/fmu/in/vehicle_visual_odometry.

Uso: mover el dron a mano y revisar el log de ev_odometry_bridge para
confirmar que cs_ev_pos/cs_ev_yaw (en estimator_status_flags) pasan a
true. Requiere EKF2_EV_CTRL=9 en el FC (ver
px4_drone_dev/docs/2026-09-16_test_lidar2d_ev_offboard.md). El dron debe
permanecer desarmado durante esta prueba.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_version_suffix_arg = DeclareLaunchArgument(
        'topic_version_suffix', default_value='',
        description=(
            'Sufijo de version de mensaje que agrega el firmware a ciertos topics '
            '(vehicle_visual_odometry, estimator_status_flags). "" (vacio, default) para '
            'firmware v1.14 (FC actual). "_v1" para firmware v1.17.'
        ),
    )

    slam_share = get_package_share_directory('px4_drone_slam')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'slam_test.launch.py')
        )
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
        name='micro_xrce_agent',
        output='screen',
    )

    ev_odometry_bridge = Node(
        package='px4_drone',
        executable='ev_odometry_bridge',
        name='ev_odometry_bridge',
        output='screen',
        parameters=[{
            'topic_version_suffix': LaunchConfiguration('topic_version_suffix'),
        }],
    )

    return LaunchDescription([
        topic_version_suffix_arg,
        slam_launch,
        micro_xrce_agent,
        ev_odometry_bridge,
    ])
