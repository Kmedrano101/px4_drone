"""
Ciclo completo indoor MAS recorrido en las 4 direcciones cardinales. DRON REAL.

    arm -> takeoff -> hold -> adelante -> centro -> atras -> centro
        -> izquierda -> centro -> derecha -> centro -> land -> disarm

Es el mismo ejecutable que takeoff_position_hold_indoor.launch.py: lo unico que
cambia es `pattern_distance_m`, que con valor > 0 activa la fase de recorrido.
Con 0 se comporta como el test de solo hold.

Las direcciones son del CUERPO del dron, no del norte magnetico: "adelante" es
hacia donde apunta el morro en el momento del hold, porque el yaw se congela ahi
durante todo el patron. Asi el test es repetible aunque el dron se arme mirando
a otro lado -- que en interior, sin brujula fiable, es lo normal.

Se vuelve al centro entre cada direccion a proposito: interesa medir la ida y la
vuelta de cada eje por separado, no dibujar un recorrido continuo.


ESPACIO LIBRE NECESARIO -- leer antes de volar
----------------------------------------------
El nodo decide que ha llegado a un punto usando la posicion del EKF, y con
navegacion por flujo optico el EKF DERIVA. Medido en SITL contra ground truth:
el nodo no dio ni un aviso, pero la posicion real llego a estar 0.44 m fuera del
objetivo en uno de los tramos. En el dron real, sin la IMU perfecta de la
simulacion, sera igual o peor.

Asi que para pattern_distance_m = 1.0 hay que contar:

    1.00 m  desplazamiento
  + 0.44 m  deriva del estimador (medida en SITL, tomar como minimo)
  + 0.33 m  radio del dron con protectores
  --------
   ~1.8 m de radio libre  ->  unos 3.6 x 3.6 m despejados

Empezar con pattern_distance_m:=0.5 la primera vez, y subir solo cuando se haya
visto la deriva real de ESTE dron en ESTE local.


USO
---
    # el agente ya va incluido aqui (serie, no UDP como en simulacion)
    source ~/drone_ws/env.sh v14
    ros2 launch px4_drone cross_pattern_indoor.launch.py \
        confirm_takeoff:=true pattern_distance_m:=0.5

Para la version de simulacion, la misma orden en la rama gazebo-sim: alli el
agente sale por UDP y topic_version_suffix es "_v1".
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
                'el test igual que takeoff_position_hold_indoor. Ver el calculo de '
                'espacio libre en la cabecera de este fichero: 1.0 m pide ~3.6 x 3.6 m '
                'despejados. La primera vez, 0.5.')),
        DeclareLaunchArgument(
            'pattern_settle_seconds', default_value='4.0',
            description=(
                'Pausa al llegar a cada punto antes de salir al siguiente. Mas alta que '
                'en simulacion (3 s) porque el dron real tarda mas en estabilizarse.')),
        DeclareLaunchArgument(
            'topic_version_suffix', default_value='',
            description=(
                'Sufijo de version de mensaje que agrega el firmware a ciertos topics '
                '(vehicle_status, vehicle_local_position). "" (vacio, default) para '
                'firmware v1.14 (FC actual, anterior al versionado de mensajes de PX4). '
                '"_v1" para firmware v1.17 (HKUST_NXT_DUAL) -- en ese caso, ademas, '
                'cambiar la branch de px4_msgs a fc-v17-82e3322e y recompilar.')),
        DeclareLaunchArgument(
            'run_agent', default_value='true',
            description=(
                'Arrancar el MicroXRCEAgent aqui. Ponlo a false si ya tienes uno en '
                'marcha (por ejemplo lanzado por systemd en la Pi): dos agentes se '
                'pelean por /dev/ttyAMA0 y el sintoma es que no llega ningun topic.'),
        ),
    ]

    micro_xrce_agent = ExecuteProcess(
        # Dron real: el cliente uXRCE-DDS del FC sale por serie. En SITL seria
        # udp4 -p 8888.
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
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
