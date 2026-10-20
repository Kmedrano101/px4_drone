"""
Ciclo completo con control MPC: arm, takeoff, hold, descenso controlado, land y
disarm. Misma mision y mismas protecciones que takeoff_position_hold_ev; lo que
cambia es el controlador (publica ACELERACION, no posicion), para poder comparar
PID contra MPC con todo lo demas igual.

Requiere que el solver este generado EN ESTA MAQUINA:
    python3 scripts/mpc/generate_cycle_mpc.py

Requiere EKF2_EV_CTRL=9 en el FC. VA A DESPEGAR DE VERDAD.
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
    mpc_v_up_arg = DeclareLaunchArgument(
        'mpc_v_up', default_value='0.6',
        description='Velocidad maxima de subida (m/s) que el MPC puede planificar.',
    )
    mpc_v_dn_arg = DeclareLaunchArgument(
        'mpc_v_dn', default_value='0.4',
        description='Velocidad maxima de bajada (m/s): acota la velocidad de contacto.',
    )
    mpc_j_max_arg = DeclareLaunchArgument(
        'mpc_j_max', default_value='4.0',
        description='Jerk maximo (m/s3). Es el parametro que da la suavidad del vuelo.',
    )
    mpc_descent_agl_arg = DeclareLaunchArgument(
        'mpc_descent_agl_m', default_value='0.25',
        description=('Metros sobre el suelo hasta los que baja el MPC antes de entregar el '
                     'contacto a PX4 (NAV_LAND), que detecta el aterrizaje y desarma.'),
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
    min_width_arg = DeclareLaunchArgument(
        'obstacle_min_width_m', default_value='0.15',
        description=(
            'Ancho real minimo (cuerda) de un grupo de rayos contiguos para creerse que hay un objeto. Un pilar de 0.40 m da 0.40 m; el polvo que levantan las helices dio 0.03-0.09 m en los bags del 2026-10-16. 0 desactiva el filtro.'),
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

    mpc_node = Node(
        package='px4_drone',
        executable='takeoff_hold_land_mpc',
        name='takeoff_hold_land_mpc',
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
            'obstacle_min_width_m': ParameterValue(
                LaunchConfiguration('obstacle_min_width_m'), value_type=float),
            'slam_health_enabled': ParameterValue(
                LaunchConfiguration('slam_health_enabled'), value_type=bool),
            'topic_version_suffix': LaunchConfiguration('topic_version_suffix'),
            'mpc_v_up': LaunchConfiguration('mpc_v_up'),
            'mpc_v_dn': LaunchConfiguration('mpc_v_dn'),
            'mpc_j_max': LaunchConfiguration('mpc_j_max'),
            'mpc_descent_agl_m': LaunchConfiguration('mpc_descent_agl_m'),
        }],
    )

    return LaunchDescription([
        confirm_takeoff_arg,
        takeoff_height_arg,
        hold_seconds_arg,
        obstacle_stop_arg,
        mpc_v_up_arg,
        mpc_v_dn_arg,
        mpc_j_max_arg,
        mpc_descent_agl_arg,
        blind_radius_arg,
        debounce_arg,
        min_width_arg,
        slam_health_arg,
        run_bridge_arg,
        topic_version_suffix_arg,
        bridge_launch,
        mpc_node,
    ])
