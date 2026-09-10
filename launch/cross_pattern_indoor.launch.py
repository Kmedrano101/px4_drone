"""
Ciclo completo indoor MAS recorrido en las 4 direcciones cardinales.

    arm -> takeoff -> hold -> adelante -> centro -> atras -> centro
        -> izquierda -> centro -> derecha -> centro -> land -> disarm

Es el mismo ejecutable que takeoff_position_hold_indoor.launch.py: lo unico que
cambia es `pattern_distance_m`, que con valor > 0 activa la fase de recorrido.
Con 0 (el defecto del nodo) se comporta como el test de solo hold.

Las direcciones son del CUERPO del dron, no del norte magnetico: "adelante" es
hacia donde apunta el morro en el momento del hold, porque el yaw se congela ahi
durante todo el patron. Asi el test es repetible aunque el dron se arme mirando
a otro lado.

Se vuelve al centro entre cada direccion a proposito: interesa medir la ida y la
vuelta de cada eje por separado, no dibujar un recorrido continuo.

ESPACIO NECESARIO: pattern_distance_m libres en las cuatro direcciones alrededor
del punto de despegue, mas el margen de los protectores (0.325 m de radio) y la
deriva del estimador. Con 1.0 m eso son unos 2.7 x 2.7 m despejados.

SIMULACION (esta rama, gazebo-sim / PX4 v1.17):

    # terminal A
    ros2 launch px4_drone sitl.launch.py
    # terminal B
    ros2 launch px4_drone cross_pattern_indoor.launch.py \
        confirm_takeoff:=true run_agent:=false
"""
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'confirm_takeoff', default_value='false',
            description='Debe ser "true" para que el nodo realmente despegue.'),
        DeclareLaunchArgument(
            'takeoff_height_m', default_value='1.0',
            description='Metros a subir (ENU, hacia arriba) desde el punto de armado.'),
        DeclareLaunchArgument(
            'hold_seconds', default_value='8.0',
            description='Segundos de hold en el centro ANTES de empezar el recorrido.'),
        DeclareLaunchArgument(
            'pattern_distance_m', default_value='1.0',
            description=(
                'Metros a desplazar en cada direccion. 0 desactiva el recorrido y deja '
                'el test igual que takeoff_position_hold_indoor.')),
        DeclareLaunchArgument(
            'pattern_settle_seconds', default_value='3.0',
            description=(
                'Pausa al llegar a cada punto antes de salir al siguiente. Sin ella se '
                'encadenan los tramos con el dron todavia oscilando y la posicion '
                'alcanzada no significa nada.')),
        DeclareLaunchArgument(
            'topic_version_suffix', default_value='_v1',
            description=(
                'Sufijo de version que el firmware anade a los topics versionados. En '
                'este SITL (PX4 main) es "_v1"; en el dron real con v1.14 es "". '
                'Verificar siempre con "ros2 topic list | grep vehicle_local_position": '
                'un nombre equivocado no da error, da silencio.')),
        DeclareLaunchArgument(
            'run_agent', default_value='true',
            description=(
                'Arrancar el MicroXRCEAgent aqui. Ponlo a false si ya lo levanto '
                'sitl.launch.py, o dos agentes pelearan por el puerto 8888.')),
    ]

    micro_xrce_agent = ExecuteProcess(
        # SITL: el cliente uXRCE-DDS de PX4 sale por UDP al 8888, no por serie.
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_agent',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_agent')),
    )

    nodo = Node(
        package='px4_drone',
        executable='takeoff_position_hold_indoor',
        name='cross_pattern_indoor',
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

    return LaunchDescription(args + [micro_xrce_agent, nodo])
