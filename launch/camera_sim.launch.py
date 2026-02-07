from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import ConcatSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    enable_rviz_republish = LaunchConfiguration('enable_rviz_republish')
    in_topic = LaunchConfiguration('in_topic')
    out_topic = LaunchConfiguration('out_topic')

    in_arg = ConcatSubstitution([TextSubstitution(text='in:='), in_topic])
    out_arg = ConcatSubstitution([TextSubstitution(text='out:='), out_topic])

    rviz_republish = Node(
        package='image_transport',
        executable='republish',
        name='sim_camera_rviz_republish',
        output='screen',
        arguments=[
            'raw', in_arg,
            'compressed', out_arg,
        ],
        condition=IfCondition(enable_rviz_republish),
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz_republish', default_value='true'),

        # Ajusta esto a tu topic real de cámara en Gazebo si no es /camera/image_raw
        DeclareLaunchArgument('in_topic', default_value='/camera/image_raw'),

        # Topic dedicado para RViz/Web (igual que en real)
        DeclareLaunchArgument('out_topic', default_value='/camera/image_rviz/compressed'),

        rviz_republish,
    ])
