"""
Ciclo completo (despegue, hold, patron cruz de 4 direcciones, aterrizaje) usando
el LiDAR 2D (SLAM -> external vision) para posicion/yaw y el LiDAR 1D + barometro para altura.

Recorrido de la cruz:
    arm -> takeoff (1.0m) -> hold -> adelante (1.0m) -> centro -> atras (1.0m) -> centro
        -> izquierda (1.0m) -> centro -> derecha (1.0m) -> centro -> land -> disarm

Vuelve al centro en cada recorrido.
Techo de 1.2 m (el nodo se niega a arrancar si se pide mas).

Lanza el stack completo si run_bridge:=true: px4_drone_slam (LD19 + slam_toolbox)
+ MicroXRCEAgent + ev_odometry_bridge + takeoff_position_hold_ev.

Requiere EKF2_EV_CTRL=9 en el FC. VA A DESPEGAR DE VERDAD: helices puestas,
area despejada (minimo 3x3 m para 1.0 m de recorrido), confirmar con confirm_takeoff:=true.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    confirm_takeoff_arg = DeclareLaunchArgument(
        'confirm_takeoff', default_value='false',
        description='Debe ser "true" para que el nodo realmente despegue.',
    )
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height_m', default_value='1.0',
        description='Metros a subir (ENU, hacia arriba) desde el punto de armado. Techo 1.2 m.',
    )
    hold_seconds_arg = DeclareLaunchArgument(
        'hold_seconds', default_value='5.0',
        description='Segundos a mantener posicion antes de iniciar el recorrido.',
    )
    pattern_distance_arg = DeclareLaunchArgument(
        'pattern_distance_m', default_value='1.0',
        description='Metros a desplazar en cada direccion (volviendo al centro entre tramos).',
    )
    pattern_settle_arg = DeclareLaunchArgument(
        'pattern_settle_seconds', default_value='3.0',
        description='Pausa de estabilizacion en segundos al llegar a cada punto.',
    )
    run_bridge_arg = DeclareLaunchArgument(
        'run_bridge', default_value='true',
        description='Si es true, lanza tambien ev_odometry_bridge, slam y MicroXRCEAgent.',
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
        condition=IfCondition(LaunchConfiguration('run_bridge')),
    )

    takeoff_position_hold_ev = Node(
        package='px4_drone',
        executable='takeoff_position_hold_ev',
        name='cross_pattern_ev',
        output='screen',
        parameters=[{
            'confirm_takeoff': LaunchConfiguration('confirm_takeoff'),
            'takeoff_height_m': LaunchConfiguration('takeoff_height_m'),
            'hold_seconds': LaunchConfiguration('hold_seconds'),
            'pattern_distance_m': LaunchConfiguration('pattern_distance_m'),
            'pattern_settle_seconds': LaunchConfiguration('pattern_settle_seconds'),
            'topic_version_suffix': LaunchConfiguration('topic_version_suffix'),
        }],
    )

    return LaunchDescription([
        confirm_takeoff_arg,
        takeoff_height_arg,
        hold_seconds_arg,
        pattern_distance_arg,
        pattern_settle_arg,
        run_bridge_arg,
        topic_version_suffix_arg,
        bridge_launch,
        takeoff_position_hold_ev,
    ])
