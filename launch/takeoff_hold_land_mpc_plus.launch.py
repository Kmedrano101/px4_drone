"""Tercer ciclo: el MISMO nodo MPC con el observador de perturbacion y la
compensacion de retardo activados.

No duplica nada: incluye takeoff_hold_land_mpc.launch.py y solo cambia esos
parametros. El ejecutable, la mision, las guardas y el registro son identicos,
que es la condicion para que comparar los tres ciclos signifique algo.

Por que estas dos cosas y no un NMPC de dinamica completa: el MPC de este dron
es un triple integrador cinematico (lazo exterior), y lo que le falta para no
dejar sesgo permanente en el hold es un estado de perturbacion -- la
construccion estandar de MPC sin desplazamiento permanente (arXiv 2011.11104).
Los NMPC de empuje + velocidades angulares que publica la literatura agil
necesitan >=100 Hz y un modelo con actitud: eso es otro nodo, no un parametro.
Ver docs/MPC_CICLO_COMPLETO.md.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base = PathJoinSubstitution([
        FindPackageShare('px4_drone'), 'launch', 'takeoff_hold_land_mpc.launch.py'])

    # Valores por defecto del tercer ciclo. El retardo sale APAGADO a proposito:
    # hay que medirlo con la edad del estado que registra el nodo antes de
    # compensarlo, porque compensar un retardo equivocado empeora el lazo.
    reenviados = [
        ('takeoff_height_m', '0.8'),
        ('hold_seconds', '30.0'),
        ('obstacle_stop_distance_m', '1.0'),
        ('confirm_takeoff', 'false'),
        ('mpc_dist_gain', '0.5'),
        ('mpc_dist_max', '1.5'),
        ('mpc_dist_min_agl_m', '0.35'),
        ('mpc_delay_comp_s', '0.0'),
        ('topic_version_suffix', ''),
        ('bench_disarm_after_s', '0.0'),
    ]

    args = [
        DeclareLaunchArgument(nombre, default_value=valor)
        for nombre, valor in reenviados
    ]
    return LaunchDescription(args + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([base]),
            launch_arguments=[
                (nombre, LaunchConfiguration(nombre)) for nombre, _ in reenviados
            ],
        ),
    ])
