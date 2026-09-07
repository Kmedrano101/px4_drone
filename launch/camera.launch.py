from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

# camera_ros (paquete de ROS) enlaza contra libcamera 0.7.1 vendorizado en
# /opt/ros/jazzy, que tiene un bug conocido para la OV5647 (assert en
# ipa_base.cpp / control_serializer.cpp, ver github.com/raspberrypi/libcamera
# issue #258): la camara nunca llega a publicar un frame. Se compilo
# libcamera desde el fork de Raspberry Pi (con el fix) en /usr/local; estas
# variables de entorno hacen que camera_ros cargue esa version en vez de la
# vendorizada, sin tocar el paquete de apt.
LIBCAMERA_FIXED_LIB_DIR = '/usr/local/lib/aarch64-linux-gnu'
LIBCAMERA_FIXED_IPA_DIR = LIBCAMERA_FIXED_LIB_DIR + '/libcamera/ipa'


def generate_launch_description():
    width_arg = DeclareLaunchArgument(
        'width', default_value='640',
        description='Ancho en pixeles del stream de la camara.',
    )
    height_arg = DeclareLaunchArgument(
        'height', default_value='480',
        description='Alto en pixeles del stream de la camara.',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='camera_link',
        description='Frame TF de la camara (debe tener una transformada estatica desde base_link).',
    )

    fix_ld_library_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=[
            TextSubstitution(text=LIBCAMERA_FIXED_LIB_DIR + ':'),
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ],
    )
    fix_ipa_module_path = SetEnvironmentVariable(
        name='LIBCAMERA_IPA_MODULE_PATH',
        value=LIBCAMERA_FIXED_IPA_DIR,
    )

    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'format': 'RGB888',
            'frame_id': LaunchConfiguration('frame_id'),
        }],
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        frame_id_arg,
        fix_ld_library_path,
        fix_ipa_module_path,
        camera_node,
    ])
