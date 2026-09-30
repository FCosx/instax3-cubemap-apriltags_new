"""Run the X3 ROS driver, ERP conversion, and cubemap publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = 'da360_ros_pipeline'


def generate_launch_description():
    package_share = FindPackageShare(PACKAGE_NAME)
    compressed_topic = LaunchConfiguration('compressed_topic')
    equirect_topic = LaunchConfiguration('equirect_topic')
    equirect_compressed_topic = LaunchConfiguration('equirect_compressed_topic')
    equirect_config = PathJoinSubstitution([package_share, 'config', 'equirectangular.yaml'])
    equirect_width = LaunchConfiguration('equirect_width')
    equirect_height = LaunchConfiguration('equirect_height')
    hardware_decoder = LaunchConfiguration('hardware_decoder')
    cubemap_face_size = LaunchConfiguration('cubemap_face_size')
    cubemap_max_fps = LaunchConfiguration('cubemap_max_fps')
    cubemap_image_encoding = LaunchConfiguration('cubemap_image_encoding')
    cubemap_gui = LaunchConfiguration('cubemap_gui')
    white_balance = LaunchConfiguration('white_balance_kelvin')

    panorama = Node(
        package=PACKAGE_NAME,
        executable='panorama_node',
        name='panorama_node',
        output='screen',
        parameters=[
            equirect_config,
            {
                'compressed_topic': compressed_topic,
                'output_topic': equirect_topic,
                'compressed_output_topic': equirect_compressed_topic,
                'skip_frame': 0,
                'i_frame_only': False,
                'use_hardware_decoder': ParameterValue(hardware_decoder, value_type=bool),
                'out_width': ParameterValue(equirect_width, value_type=int),
                'out_height': ParameterValue(equirect_height, value_type=int),
            },
        ],
    )

    driver = Node(
        package='insta360_ros_driver',
        executable='insta360_ros_driver',
        name='insta360_ros_driver',
        output='screen',
        parameters=[{
            'compressed_topic': compressed_topic,
            'white_balance_kelvin': ParameterValue(
                white_balance,
                value_type=int,
            ),
        }],
    )

    cubemap = Node(
        package=PACKAGE_NAME,
        executable='cubemap_node.py',
        name='cubemap',
        output='screen',
        arguments=[
            '--topic', equirect_compressed_topic,
            '--face-size', cubemap_face_size,
            '--max-fps', cubemap_max_fps,
            '--image-encoding', cubemap_image_encoding,
            '--gui', cubemap_gui,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'compressed_topic',
            default_value='/dual_fisheye/image/compressed',
            description='H.264 CompressedImage topic published by the bundled X3 ROS driver.',
        ),
        DeclareLaunchArgument('equirect_topic', default_value='/equirectangular/image'),
        DeclareLaunchArgument(
            'equirect_compressed_topic',
            default_value='/equirectangular/image/compressed',
        ),
        DeclareLaunchArgument('equirect_width', default_value='1440'),
        DeclareLaunchArgument('equirect_height', default_value='720'),
        DeclareLaunchArgument('hardware_decoder', default_value='false'),
        DeclareLaunchArgument('cubemap_face_size', default_value='360'),
        DeclareLaunchArgument('cubemap_max_fps', default_value='10'),
        DeclareLaunchArgument('cubemap_image_encoding', default_value='bgr8'),
        DeclareLaunchArgument('cubemap_gui', default_value='false'),
        DeclareLaunchArgument('white_balance_kelvin', default_value='5000'),
        driver,
        RegisterEventHandler(
            OnProcessExit(
                target_action=driver,
                on_exit=[EmitEvent(event=Shutdown(reason='X3 driver exited'))],
            )
        ),
        panorama,
        cubemap,
    ])
