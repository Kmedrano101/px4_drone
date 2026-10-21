"""Ciclo completo con el NMPC de empuje colectivo + velocidades angulares (CTBR).

Misma mision y mismas protecciones que los otros dos ciclos; lo que cambia es que
el controlador incluye la ACTITUD en su modelo y publica TASAS + EMPUJE, que es
lo que hace la literatura agil (HPA-MPC arXiv 2411.11982).

⚠️ El CTBR se engancha SOLO durante el hold. Con offboard_control_mode.body_rate,
PX4 1.14.3 no ejecuta mc_pos_control (control_mode.cpp:169-175 del tag): no hay
maquina de despegue, ni limite de inclinacion, ni estimador de empuje de hover, y
el empuje va del mezclador a los motores. El despegue y el aterrizaje los hace
PX4 con setpoints de posicion, igual que el ciclo PID.

Requiere que el solver este generado EN ESTA MAQUINA:
    python3 scripts/mpc/generate_ctbr_mpc.py

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
    blind_radius_arg = DeclareLaunchArgument(
        'obstacle_blind_radius_m', default_value='0.35',
        description='Radio ciego en los 360 grados, centrado en el LiDAR (0.35 m = diametro 0.70 m).',
    )
    debounce_arg = DeclareLaunchArgument(
        'obstacle_debounce_scans', default_value='3',
        description=(
            'Barridos seguidos disparando para dar la parada. Con 3 se descarta la rafaga de retornos que levantan las helices al despegar; con 2 abortaba el ciclo (bags del 2026-10-16).'),
    )
    rate_arg = DeclareLaunchArgument(
        'ctbr_rate_hz', default_value='50.0',
        description=('Frecuencia del lazo rapido. Al comandar tasas el lazo de actitud es '
                     'nuestro: 20 Hz no basta. Medido en la Pi 4, el solver tarda 2.7 ms de '
                     'media y 5.9 ms en el peor caso, asi que 50 Hz usa el 29% del ciclo y '
                     '100 Hz el 59%.'),
    )
    thr_hover_arg = DeclareLaunchArgument(
        'ctbr_thr_hover', default_value='0.30',
        description=('Empuje normalizado de hover, el ancla del mapeo m/s2 -> [0,1]. '
                     'HAY QUE CALIBRARLO: depende de THR_MDL_FAC, que en este FC esta en 0.'),
    )
    engage_arg = DeclareLaunchArgument(
        'ctbr_engage_delay_s', default_value='3.0',
        description='Segundos dentro del hold antes de tomar el mando, para que PX4 estabilice.',
    )
    tilt_arg = DeclareLaunchArgument(
        'ctbr_max_tilt_deg', default_value='20.0',
        description='Inclinacion a partir de la cual se suelta y vuelve a mandar PX4.',
    )
    bench_arg = DeclareLaunchArgument(
        'bench_disarm_after_s', default_value='0.0',
        description=(
            'MODO BANCO SIN HELICES: segundos tras el TAKEOFF en los que el nodo manda un '
            'desarme FORZADO y termina. 0 = apagado (campo). En banco el CTBR nunca engancha '
            '(solo lo hace en el hold); lo que se prueba es que el nodo arranca y no se cae.'),
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

    ctbr_node = Node(
        package='px4_drone',
        executable='takeoff_hold_land_ctbr',
        name='takeoff_hold_land_ctbr',
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
            'ctbr_rate_hz': ParameterValue(
                LaunchConfiguration('ctbr_rate_hz'), value_type=float),
            'ctbr_thr_hover': ParameterValue(
                LaunchConfiguration('ctbr_thr_hover'), value_type=float),
            'ctbr_engage_delay_s': ParameterValue(
                LaunchConfiguration('ctbr_engage_delay_s'), value_type=float),
            'ctbr_max_tilt_deg': ParameterValue(
                LaunchConfiguration('ctbr_max_tilt_deg'), value_type=float),
            'bench_disarm_after_s': ParameterValue(
                LaunchConfiguration('bench_disarm_after_s'), value_type=float),
            'obstacle_min_width_m': ParameterValue(
                LaunchConfiguration('obstacle_min_width_m'), value_type=float),
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
        rate_arg,
        thr_hover_arg,
        engage_arg,
        tilt_arg,
        bench_arg,
        min_width_arg,
        slam_health_arg,
        run_bridge_arg,
        topic_version_suffix_arg,
        bridge_launch,
        ctbr_node,
    ])
