#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Argumentos del launch
    video_device = LaunchConfiguration('video_device', default='/dev/video0')
    frame_id = LaunchConfiguration('camera_frame_id', default='camera_optical_link')
    image_width = LaunchConfiguration('image_width', default='640')
    image_height = LaunchConfiguration('image_height', default='480')
    framerate = LaunchConfiguration('framerate', default='15.0')
    pixel_format = LaunchConfiguration('pixel_format', default='YUYV')  # o 'MJPG' si quieres más fps

    return LaunchDescription([
        # Declarar argumentos (puedes cambiarlos al lanzar)
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Dispositivo de video (/dev/video0 recomendado para Pi Camera 3)'
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_optical_link',
            description='Frame ID para la imagen (debe coincidir con tu URDF)'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Ancho de la imagen'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Alto de la imagen'
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='15.0',
            description='Frecuencia de publicación (fps)'
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='YUYV',
            description='Formato de píxeles: YUYV (más compatible) o MJPG (más rápido)'
        ),

        # Nodo de la cámara
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'video_device': video_device,
                'camera_frame_id': frame_id,
                'image_size': [image_width, image_height],
                'pixel_format': pixel_format,
                'framerate': framerate,
                'output_encoding': 'rgb8',          # 'bgr8' o 'rgb8' según tu preferencia
                'qos_overrides./image_raw.publisher.reliability': 'best_effort',
                'qos_overrides./image_raw.publisher.durability': 'volatile',
                # Opcional: descomenta si quieres más control
                # 'brightness': 50,
                # 'contrast': 50,
                # 'sharpness': 0,
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),     # mismo topic que en simulación
                ('/camera_info', '/camera/camera_info'),
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])