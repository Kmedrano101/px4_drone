"""
Levanta el simulador: PX4 SITL + Gazebo Sim con el modelo del dron real.

Solo el simulador y el agente uXRCE-DDS. Los nodos de control se lanzan aparte,
en otra terminal:

    ros2 launch px4_drone sitl.launch.py
    ros2 launch px4_drone takeoff_position_hold_indoor.launch.py confirm_takeoff:=true run_agent:=false

Requiere haber instalado el airframe y los modelos en PX4-Autopilot:
    ~/src/drone_tunning/gazebo/sync_px4.sh restore
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


def _flag_if_true(arg_name):
    """PX4 mira si la variable esta VACIA, no su contenido.

    `[ -z "$HEADLESS" ]` -> con GUI;  no vacia -> headless. Lo mismo con
    PX4_GZ_NO_FOLLOW. Asi que hay que mandar "1" o cadena vacia, nunca
    "true"/"false": pasar la cadena "false" ACTIVARIA la opcion.
    """
    return PythonExpression(["'1' if '", LaunchConfiguration(arg_name), "' == 'true' else ''"])


def generate_launch_description():
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot'),
        description='Ruta al clon de PX4-Autopilot.',
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value='indoor_zone',
        description='Mundo de Gazebo. "indoor_zone" es la zona de pruebas escaneada.',
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='rjx_f450_indoor',
        description='Modelo del dron (RJX F450 + MTF-01P + LDROBOT D500).',
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description=(
            'OJO: con headless=true, en maquinas NVIDIA el servidor de Gazebo falla al '
            'crear el contexto EGL y los sensores GPU (los dos LiDAR y el flujo optico) '
            'dejan de publicar SIN dar ningun error. Dejar en false salvo que sepas que '
            'tu EGL headless funciona.'
        ),
    )
    follow_arg = DeclareLaunchArgument(
        'camera_follow', default_value='false',
        description=(
            'true = PX4 fija la camara al modelo (modo FOLLOW), lo que anula el zoom '
            'con la rueda del raton. false = camara libre.'
        ),
    )

    # El uXRCE-DDS de PX4 en SITL habla UDP; en el dron real es serie por ttyAMA0.
    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_agent',
        output='screen',
    )

    # `make px4_sitl gz_<modelo>` arranca Gazebo y PX4 juntos, en lockstep.
    sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', ['gz_', LaunchConfiguration('model')]],
        cwd=LaunchConfiguration('px4_dir'),
        name='px4_sitl',
        output='screen',
        additional_env={
            'PX4_GZ_WORLD': LaunchConfiguration('world'),
            'HEADLESS': _flag_if_true('headless'),
            # invertido a proposito: camera_follow=false -> NO_FOLLOW=1
            'PX4_GZ_NO_FOLLOW': PythonExpression(
                ["'' if '", LaunchConfiguration('camera_follow'), "' == 'true' else '1'"]),
        },
    )

    return LaunchDescription([
        px4_dir_arg, world_arg, model_arg, headless_arg, follow_arg,
        agent, sitl,
    ])
