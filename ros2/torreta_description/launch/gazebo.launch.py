"""
A torreta no Gazebo (DART; ver worlds/torreta.sdf) com ros2_control e o dardo solto.
Uso: ros2 launch torreta_description gazebo.launch.py [gui:=false]
Adaptado do gazebo.launch.py do Projeto 2 (receita gazebo-ros2control.md).
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('torreta_description')
    # Os dois podem ser trocados por argumento (o pick and place usa o mundo com o
    # objeto e a descrição com a pega); os padrões são os da etapa do Gazebo.
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', LaunchConfiguration('descricao')])
    world_path = LaunchConfiguration('mundo')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    gui = LaunchConfiguration('gui')

    # O xacro resolve o $(find torreta_description) que está dentro da
    # tag <gazebo> do URDF (caminho do controladores.yaml).
    # urdf:=torreta_video.urdf no vídeo (mesma física; sem gaiola/funil e com marca nos volantes)
    robot_description = ParameterValue(Command(['xacro ', urdf_path, ' urdf:=', LaunchConfiguration('urdf'), ' webcam:=', LaunchConfiguration('webcam')]), value_type=str)

    # O Gazebo procura "package://torreta_description/meshes/..." dentro
    # das pastas listadas nesta variável — precisa da pasta que CONTÉM o
    # pacote, por isso o '..'.
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, '..'))

    # -r = já começa rodando (sem isso a simulação abre pausada).
    # -s = só servidor, sem janela.
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-r -s ', world_path]}.items(),
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'torreta', '-allow_renaming', 'true'],
    )

    # Formato de cada ponte: tópico@tipo_ROS[tipo_Gazebo. O "[" quer dizer
    # "só do Gazebo pro ROS".
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
    )

    # O dardo da câmara: corpo solto (urdf/dardo.sdf, gerado do CAD), nascido na pose do CAD com a
    # torreta na pose zero. Quem faz o papel do atrito dos volantes é o scripts/simular_volantes.py
    # (só simulação; o nó de comando é o do Daniel). dardo:=false tira os dois.
    dardo_cfg = json.load(open(os.path.join(pkg_share, 'config', 'dardo.json'), encoding='utf-8'))
    x, y, z = (str(v) for v in dardo_cfg['nascimento_xyz_m'])
    com_dardo = IfCondition(LaunchConfiguration('dardo'))
    spawn_dardo = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'dardo.sdf'), '-name', 'dardo', '-x', x, '-y', y, '-z', z],
        condition=com_dardo,
    )
    simular_volantes = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_share, 'scripts', 'simular_volantes.py')],
        output='screen',
        condition=com_dardo,
    )

    # um spawner por controlador (controladores.yaml), em fila
    spawners = [Node(package='controller_manager', executable='spawner', output='screen', arguments=[nome])
                for nome in ('pan_tilt_controller', 'gatilho_controller', 'volantes_controller')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'mundo',
            default_value=os.path.join(pkg_share, 'worlds', 'torreta.sdf'),
            description='Mundo do Gazebo (nome conhecido do Gazebo ou caminho completo)'
        ),
        DeclareLaunchArgument(
            'descricao',
            default_value='torreta_gazebo.urdf.xacro',
            description='Arquivo da pasta urdf/ com a descrição da torreta'
        ),
        DeclareLaunchArgument(
            'urdf',
            default_value='torreta.urdf',
            description='URDF incluído pelo xacro (torreta_video.urdf só no vídeo)'
        ),
        DeclareLaunchArgument(
            'dardo',
            default_value='true',
            description='Põe o dardo solto na câmara e o simulador dos volantes'
        ),
        DeclareLaunchArgument(
            'webcam',
            default_value='false',
            description='Webcam simulada no berço (true no rastreio; o mundo precisa do sistema de sensores)'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Abre a janela 3D do Gazebo se true'
        ),
        set_resource_path,
        gazebo_gui,
        gazebo_headless,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        # Os controladores só podem subir depois que o robô existe no
        # Gazebo (é o plugin dele que cria o controller_manager), e um de
        # cada vez.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner, spawn_dardo, simular_volantes],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[spawners[0]],
        )),
        RegisterEventHandler(OnProcessExit(target_action=spawners[0], on_exit=[spawners[1]])),
        RegisterEventHandler(OnProcessExit(target_action=spawners[1], on_exit=[spawners[2]])),
    ])
