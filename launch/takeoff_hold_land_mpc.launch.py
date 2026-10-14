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
        description='Segundos a mantener posicion antes de aterrizar.',
    )
    obstacle_stop_arg = DeclareLaunchArgument(
        'obstacle_stop_distance_m', default_value='1.0',
        description=('Distancia (m) a la que el dron debe quedar parado de cualquier obstaculo, '
                     'medida con el LiDAR 2D en 360 grados. 0 = desactivada. No arma si hay algo '
                     'mas cerca; en vuelo frena y aterriza.'),
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
        run_bridge_arg,
        topic_version_suffix_arg,
        bridge_launch,
        mpc_node,
    ])
