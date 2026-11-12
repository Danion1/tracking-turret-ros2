"""
Rastreio no Gazebo (2026-11-10): a torreta com a WEBCAM simulada + o "drone" alvo voando.
Uso: ros2 launch torreta_description rastreio.launch.py [modo:=vaivem|circulo|oito|parado] [gui:=false]

Sobe o gazebo.launch.py (mundo com o sistema de sensores, webcam:=true, dardo e controladores como sempre),
o alvo (urdf/alvo.sdf), o scripts/mover_alvo.py e a ponte da câmera para o ROS:
  /webcam/image_raw    sensor_msgs/Image (rgb8, 640 × 360, 30 Hz simulados, frame webcam_optico)
  /webcam/camera_info  sensor_msgs/CameraInfo (fx = fy ≈ 453,4 px, centro 320 × 180)
  /alvo/verdade        geometry_msgs/PointStamped (posição real do alvo; só para MEDIR, não para rastrear)
O nó de comando (comando_torreta) e o de detecção/rastreio são do Daniel e rodam à parte.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('torreta_description')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'mundo': os.path.join(share, 'worlds', 'torreta_rastreio.sdf'),
            'webcam': 'true',
            'gui': LaunchConfiguration('gui'),
            'urdf': LaunchConfiguration('urdf'),
        }.items(),
    )
    alvo = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-file', os.path.join(share, 'urdf', 'alvo.sdf'), '-name', 'alvo',
                   '-x', LaunchConfiguration('distancia'), '-y', '0.0', '-z', LaunchConfiguration('altura')],
    )
    mover = ExecuteProcess(
        cmd=['python3', os.path.join(share, 'scripts', 'mover_alvo.py'), '--ros-args',
             '-p', ['modo:=', LaunchConfiguration('modo')],
             '-p', ['distancia:=', LaunchConfiguration('distancia')],
             '-p', ['altura:=', LaunchConfiguration('altura')],
             '-p', ['periodo:=', LaunchConfiguration('periodo')]],
        output='screen',
    )
    # "[" = só do Gazebo para o ROS
    ponte_camera = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/webcam/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/webcam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='vaivem', description='parado | vaivem | circulo | oito'),
        DeclareLaunchArgument('distancia', default_value='3.0', description='x do centro da trajetória (m)'),
        DeclareLaunchArgument('altura', default_value='0.8', description='z do centro da trajetória (m)'),
        DeclareLaunchArgument('periodo', default_value='8.0', description='período da trajetória (s simulados)'),
        DeclareLaunchArgument('gui', default_value='true', description='janela do Gazebo'),
        DeclareLaunchArgument('urdf', default_value='torreta.urdf', description='torreta_video.urdf só no vídeo'),
        gazebo,
        ponte_camera,
        # o alvo nasce depois que o mundo existe; o mover espera o /clock sozinho
        TimerAction(period=3.0, actions=[alvo, mover]),
    ])
