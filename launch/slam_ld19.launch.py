#!/usr/bin/env python3
"""LD19 + relay a tamano fijo + slam_toolbox (online async), en la Pi.

Portado desde px4_drone_slam/launch/slam_test.launch.py el 2026-10-21, que vivia
sin versionar en la SD de la Pi. Dos cambios respecto a aquel:

1. El driver del LD19 publica en /scan_raw y el relay scan_fixed_size publica
   /scan con 500 rayos fijos. El LD19 entrega un numero variable de puntos por
   vuelta (498-506) y karto rechaza todo barrido cuyo numero no coincida con el
   primero: solo pasaba el 20-30 % y la pose llegaba al EKF a ~1 Hz. Ver
   include/px4_drone/scan_fixed_size.hpp y el reporte del 2026-10-21.
2. minimum_time_interval de slam_toolbox es argumento de launch, para poder
   subir la tasa por pasos midiendo CPU sin editar el yaml.

No hay odometria real: odom->base_link es la identidad y el scan matching
estima el movimiento. async_slam_toolbox_node es lifecycle: se le lleva a
configure->activate desde aqui, porque sin eso nunca publica /map ni "map".
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = get_package_share_directory('px4_drone')

    min_interval_arg = DeclareLaunchArgument(
        'slam_min_time_interval', default_value='0.1',
        description=('minimum_time_interval de slam_toolbox (s). Medido en banco con el relay: 0.2 -> 3.8 Hz de pose y 41% de un nucleo; 0.1 -> 6.3 Hz y 64%. '
                     'Bajar por pasos midiendo la CPU de slam_toolbox.'),
    )
    n_rays_arg = DeclareLaunchArgument(
        'scan_n_rays', default_value='500',
        description='Rayos fijos del /scan remuestreado (500 = 0.72 grados).',
    )

    # Mismos parametros que ld19.launch.py del paquete ldlidar, salvo el topic.
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan_raw'},
            {'frame_id': 'base_laser'},
            {'enable_serial_or_network_communication': True},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'server_ip': '192.168.1.200'},
            {'server_port': '2000'},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
            {'measure_point_freq': 4500},
        ],
    )

    scan_relay = Node(
        package='px4_drone',
        executable='scan_fixed_size',
        name='scan_fixed_size',
        output='screen',
        parameters=[{
            'input_topic': '/scan_raw',
            'output_topic': '/scan',
            'n_rays': ParameterValue(LaunchConfiguration('scan_n_rays'), value_type=int),
        }],
    )

    base_link_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld19',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser'],
    )
    odom_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            os.path.join(share, 'config', 'slam_toolbox_params.yaml'),
            {'minimum_time_interval': ParameterValue(
                LaunchConfiguration('slam_min_time_interval'), value_type=float)},
        ],
    )

    configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda node: node is slam_toolbox_node,
                transition_id=Transition.TRANSITION_CONFIGURE))],
        )
    )
    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda node: node is slam_toolbox_node,
                transition_id=Transition.TRANSITION_ACTIVATE))],
        )
    )

    return LaunchDescription([
        min_interval_arg,
        n_rays_arg,
        ldlidar_node,
        scan_relay,
        base_link_to_laser_tf,
        odom_to_base_link_tf,
        slam_toolbox_node,
        configure_on_start,
        activate_on_configured,
    ])
