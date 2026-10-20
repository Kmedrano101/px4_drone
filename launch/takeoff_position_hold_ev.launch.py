"""
Ciclo completo (despegue, hold, aterrizaje) usando el LiDAR 2D (SLAM ->
external vision) para posicion/yaw y el LiDAR 1D + barometro para altura.
Techo de 1.0 m (el nodo se niega a arrancar si se pide mas).

Lanza el stack completo si run_bridge:=true: px4_drone_slam (LD19 + slam_toolbox)
+ MicroXRCEAgent + ev_odometry_bridge + takeoff_position_hold_ev.
Si run_bridge:=false, asume que el puente y SLAM ya estan corriendo y solo
lanza el nodo de vuelo.

Requiere EKF2_EV_CTRL=9 en el FC. VA A DESPEGAR DE VERDAD: helices puestas,
area despejada, confirmar antes de correr con confirm_takeoff:=true.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    confirm_takeoff_arg = DeclareLaunchArgument(
        'confirm_takeoff', default_value='false',
        description='Debe ser "true" para que el nodo realmente despegue.',
    )
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height_m', default_value='1.0',
        description='Metros a subir (ENU, hacia arriba) desde el punto de armado. Techo 1.0 m.',
    )
    hold_seconds_arg = DeclareLaunchArgument(
        'hold_seconds', default_value='5.0',
        description='Segundos a mantener posicion antes de aterrizar.',
    )
    obstacle_stop_arg = DeclareLaunchArgument(
        'obstacle_stop_distance_m', default_value='1.0',
        description=('Distancia (m) a la que el dron debe quedar parado de cualquier obstaculo, '
                     'activa desde TAKEOFF, fuera del circulo ciego de 0.70 m de diametro. '
                     'Comprueba espacio y barrido antes de armar. 0 = desactivada; frena y aterriza.'),
    )
    blind_radius_arg = DeclareLaunchArgument(
        'obstacle_blind_radius_m', default_value='0.35',
        description='Radio ciego en los 360 grados, centrado en el LiDAR (0.35 m = diametro 0.70 m).',
    )
    debounce_arg = DeclareLaunchArgument(
        'obstacle_debounce_scans', default_value='3',
        description=(
            'Barridos seguidos disparando para dar la parada. Con 3 se descarta la rafaga de retornos que levantan las helices al despegar; con 2 abortaba el ciclo (bags del 2026-10-16).'),
    )
    slam_health_arg = DeclareLaunchArgument(
        'slam_health_enabled', default_value='false',
        description=('Guarda de sigma y frescura de /pose del SLAM. Desactivada para '
                     'las pruebas de ciclo PID/MPC; mantiene las comprobaciones del FC.'),
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
        name='takeoff_position_hold_ev',
        output='screen',
        parameters=[{
            'confirm_takeoff': LaunchConfiguration('confirm_takeoff'),
            'takeoff_height_m': LaunchConfiguration('takeoff_height_m'),
            'hold_seconds': LaunchConfiguration('hold_seconds'),
            'obstacle_stop_distance_m': LaunchConfiguration('obstacle_stop_distance_m'),
            'obstacle_blind_radius_m': ParameterValue(
                LaunchConfiguration('obstacle_blind_radius_m'), value_type=float),
            'obstacle_debounce_scans': ParameterValue(
                LaunchConfiguration('obstacle_debounce_scans'), value_type=int),
            'slam_health_enabled': ParameterValue(
                LaunchConfiguration('slam_health_enabled'), value_type=bool),
            'topic_version_suffix': LaunchConfiguration('topic_version_suffix'),
        }],
    )

    return LaunchDescription([
        confirm_takeoff_arg,
        takeoff_height_arg,
        hold_seconds_arg,
        obstacle_stop_arg,
        blind_radius_arg,
        debounce_arg,
        slam_health_arg,
        run_bridge_arg,
        topic_version_suffix_arg,
        bridge_launch,
        takeoff_position_hold_ev,
    ])
