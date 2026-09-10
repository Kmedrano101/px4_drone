"""
Levanta el simulador: PX4 SITL + Gazebo Sim con el modelo del dron real.

Solo el simulador y el agente uXRCE-DDS. Los nodos de control se lanzan aparte,
en otra terminal:

    ros2 launch px4_drone sitl.launch.py
    ros2 launch px4_drone takeoff_position_hold_indoor.launch.py confirm_takeoff:=true run_agent:=false

Requiere haber instalado el airframe y los modelos en PX4-Autopilot:
    ~/src/drone_tunning/gazebo/sync_px4.sh restore
y haber compilado PX4 al menos una vez:
    cd ~/PX4-Autopilot && make px4_sitl


Por que NO se usa `make px4_sitl gz_<modelo>`
---------------------------------------------
Dos fallos, los dos reales y medidos:

1. `make` compila. Con el entorno de ROS 2 activo el build de PX4 revienta
   ("ninja: build stopped: subcommand failed"), porque hereda PYTHONPATH y el
   python del venv.

2. No se podia cerrar con Ctrl-C. `px4-rc.gzsim` arranca `gz sim -s` y
   `gz sim -g` con `&` DESDE DENTRO de PX4, y esos procesos se reparentan al
   instante (PPID 1, no px4). Al pulsar Ctrl-C morian `make` y el agente, pero
   px4 y los dos `gz` seguian vivos heredando el mismo pipe de stdout; como
   `ros2 launch` espera el EOF de ese pipe para terminar, se quedaba colgado
   ignorando los Ctrl-C siguientes.

La solucion a los dos es la misma: aqui se lanza el binario ya compilado y se
usa PX4_GZ_STANDALONE, que le dice a PX4 que NO arranque Gazebo. El servidor y
la GUI los arranca este launch, asi que `ros2 launch` es el padre de los tres
procesos y el Ctrl-C los alcanza a todos.

Cada proceso va por `bash -c '. ./gz_env.sh && exec ...'`:

  - gz_env.sh lo genera el propio build de PX4 (GZ_SIM_RESOURCE_PATH,
    GZ_SIM_SYSTEM_PLUGIN_PATH y, sobre todo, GZ_SIM_SERVER_CONFIG_PATH, que es
    lo que carga libOpticalFlowSystem.so). Se reutiliza en vez de duplicar esas
    rutas aqui para que no se queden desfasadas.
  - PX4 tambien lo necesita: en modo standalone su script NO lo carga, y sin
    PX4_GZ_MODELS no sabe de donde sacar el SDF del dron al hacer el spawn.
  - `exec` no es decorativo: sin el, bash se queda de intermediario y la senal
    de Ctrl-C no llega al proceso de verdad.
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    headless = LaunchConfiguration('headless')

    args = [
        DeclareLaunchArgument(
            'px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot'),
            description='Ruta al clon de PX4-Autopilot (ya compilado).'),
        DeclareLaunchArgument(
            'world', default_value='indoor_zone',
            description='Mundo de Gazebo. "indoor_zone" es la zona de pruebas escaneada.'),
        DeclareLaunchArgument(
            'model', default_value='rjx_f450_indoor',
            description='Modelo del dron (RJX F450 + MTF-01P + LDROBOT D500).'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description=(
                'Sin ventana de Gazebo. Verificado que los sensores GPU (los dos LiDAR '
                'y el flujo optico) siguen publicando en headless: 4/4 corridas.')),
        DeclareLaunchArgument(
            'camera_follow', default_value='false',
            description=(
                'true = PX4 fija la camara al modelo (modo FOLLOW), lo que anula el '
                'zoom con la rueda del raton. false = camara libre.')),
        DeclareLaunchArgument(
            'run_agent', default_value='true',
            description='Arrancar el agente uXRCE-DDS. false si ya lo tienes en marcha.'),
    ]

    # El binario y gz_env.sh viven en el rootfs del build; PX4 espera ese cwd.
    rootfs = PathJoinSubstitution([px4_dir, 'build', 'px4_sitl_default', 'rootfs'])

    def en_rootfs(orden, **kw):
        # `orden` puede traer sustituciones (LaunchConfiguration). Se aplana: el
        # cmd de launch admite una lista de trozos POR ARGUMENTO, pero no listas
        # anidadas dentro de un argumento.
        trozos = list(orden) if isinstance(orden, (list, tuple)) else [orden]
        return ExecuteProcess(
            cmd=['bash', '-c', ['. ./gz_env.sh && exec '] + trozos],
            cwd=rootfs, output='screen', **kw)

    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_agent', output='screen',
        # el agente ignora el SIGINT; sin bajar esta espera, el cierre tarda los
        # 5 s por defecto hasta que launch escala a SIGTERM
        sigterm_timeout='2',
        condition=IfCondition(LaunchConfiguration('run_agent')),
    )

    gz_server = en_rootfs(
        ['gz sim --verbose=1 -r -s "$PX4_GZ_WORLDS/', world, '.sdf"'],
        name='gz_server')

    gz_gui = en_rootfs(
        'gz sim -g', name='gz_gui', condition=UnlessCondition(headless))

    # PX4 espera solo a que el mundo este listo (30 intentos de 1 s en
    # px4-rc.gzsim), asi que no hace falta retrasarlo a mano.
    px4 = en_rootfs(
        './../bin/px4', name='px4',
        additional_env={
            'PX4_GZ_STANDALONE': '1',          # no arranques Gazebo, ya esta
            'PX4_SIM_MODEL': ['gz_', model],
            'PX4_GZ_WORLD': world,
            'GZ_IP': '127.0.0.1',
            # invertido a proposito: PX4 mira si la variable esta VACIA, no su
            # contenido, asi que "false" ACTIVARIA la opcion.
            'PX4_GZ_NO_FOLLOW': PythonExpression(
                ["'' if '", LaunchConfiguration('camera_follow'), "' == 'true' else '1'"]),
        })

    # Si PX4 termina, se cierra todo lo demas: dejar Gazebo huerfano es
    # exactamente el problema que este fichero viene a resolver.
    al_morir_px4 = RegisterEventHandler(
        OnProcessExit(target_action=px4, on_exit=[Shutdown(reason='PX4 ha terminado')]))

    return LaunchDescription(args + [agent, gz_server, gz_gui, px4, al_morir_px4])
