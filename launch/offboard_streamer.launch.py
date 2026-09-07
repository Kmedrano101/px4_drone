from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
        name='micro_xrce_agent',
        output='screen',
    )

    offboard_streamer = Node(
        package='px4_drone',
        executable='offboard_streamer.py',
        name='offboard_streamer',
        output='screen',
    )

    return LaunchDescription([
        micro_xrce_agent,
        offboard_streamer,
    ])
