import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('lidar_docking')

    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_start = LaunchConfiguration('auto_start')
    dock_destination_sequence = LaunchConfiguration('dock_destination_sequence')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg, 'config', 'lidar_docking_params.yaml'),
            description='Arquivo YAML com parametros do docking por LiDAR'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usa clock da simulacao'),
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='Inicia retorno ao dock automaticamente ao subir o launch'),
        DeclareLaunchArgument(
            'dock_destination_sequence',
            default_value='16',
            description='Sequence do ponto de staging do dock no route graph'),
        Node(
            package='lidar_docking',
            executable='lidar_dock_detector.py',
            name='lidar_dock_detector',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}]),
        Node(
            package='lidar_docking',
            executable='lidar_docking_manager.py',
            name='lidar_docking_manager',
            output='screen',
            parameters=[config_file, {
                'use_sim_time': use_sim_time,
                'auto_start': ParameterValue(auto_start, value_type=bool),
                'dock_destination_sequence': ParameterValue(dock_destination_sequence, value_type=int),
            }]),
    ])
