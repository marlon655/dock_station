import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_text(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _configured_nodes(context):
    pkg = get_package_share_directory('aruco_docking_real')
    config_path = LaunchConfiguration('config_file').perform(context)
    if not config_path:
        config_path = os.path.join(pkg, 'config', 'docking_params_real.yaml')

    use_sim_time = _bool_text(LaunchConfiguration('use_sim_time').perform(context))
    start_charging_manager = LaunchConfiguration('start_charging_manager')
    battery = LaunchConfiguration('battery')

    camera_image_topic = LaunchConfiguration('camera_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    with open(config_path, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}

    server_params = params.get('docking_server', {}).get('ros__parameters', {})
    docks = server_params.get('docks', ['base_carregamento'])
    dock_id = docks[0] if docks else 'base_carregamento'
    dock_pose = server_params.get(dock_id, {}).get('pose')
    if not dock_pose or len(dock_pose) < 3:
        raise RuntimeError(
            f'Missing docking_server.ros__parameters.{dock_id}.pose in {config_path}')

    dock_yaw = float(dock_pose[2])
    fixed_frame = str(server_params.get('fixed_frame', server_params.get(dock_id, {}).get('frame', 'odom')))

    return [
        Node(
            package='aruco_docking_real',
            executable='dock_pose_estimator.py',
            name='dock_pose_estimator',
            output='screen',
            parameters=[config_path, {
                'use_sim_time': use_sim_time,
                'target_frame': fixed_frame,
                'dock_yaw': dock_yaw,
            }],
            remappings=[
                ('camera/image', camera_image_topic),
                ('camera/camera_info', camera_info_topic),
                ('scan', scan_topic),
            ]),
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[config_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_docking',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['docking_server'],
            }]),
        GroupAction(
            condition=IfCondition(start_charging_manager),
            actions=[Node(
                package='aruco_docking_real',
                executable='charging_manager.py',
                name='charging_manager',
                output='screen',
                parameters=[config_path, {
                    'use_sim_time': use_sim_time,
                    'battery': battery,
                }],
                remappings=[
                    ('/odom', odom_topic),
                    ('/cmd_vel', cmd_vel_topic),
                ])]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('start_charging_manager', default_value='true'),
        DeclareLaunchArgument('battery', default_value='100.0'),
        DeclareLaunchArgument('camera_image_topic', default_value='/camera/image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        OpaqueFunction(function=lambda context: _configured_nodes(context)),
    ])
