from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import ConcatSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    camera_index = LaunchConfiguration('camera_index')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    pixel_format = LaunchConfiguration('pixel_format')

    enable_rviz_republish = LaunchConfiguration('enable_rviz_republish')
    in_topic = LaunchConfiguration('in_topic')
    out_topic = LaunchConfiguration('out_topic')

    # camera_ros (real)
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'camera': camera_index,
            'width': width,
            'height': height,
            'format': pixel_format,
        }]
    )

    # image_transport republish: raw -> compressed (topic dedicado a RViz/web)
    # OJO: camera_ros ya publica /camera/image_raw/compressed; esto crea uno aparte y estable:
    # /camera/image_rviz/compressed (por defecto)
    in_arg = ConcatSubstitution([TextSubstitution(text='in:='), in_topic])
    out_arg = ConcatSubstitution([TextSubstitution(text='out:='), out_topic])

    rviz_republish = Node(
        package='image_transport',
        executable='republish',
        name='camera_rviz_republish',
        output='screen',
        arguments=[
            'raw', in_arg,
            'compressed', out_arg,
        ],
        condition=IfCondition(enable_rviz_republish),
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='360'),
        DeclareLaunchArgument('pixel_format', default_value='RGB888'),

        # Republish para RViz/Web
        DeclareLaunchArgument('enable_rviz_republish', default_value='true'),
        DeclareLaunchArgument('in_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('out_topic', default_value='/camera/image_rviz/compressed'),

        camera_node,
        rviz_republish,
    ])
