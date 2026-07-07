import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_text(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _configured_nodes(context, *, use_sim_time_default, config_file_name, spawn_gazebo_dock_default):
    pkg = get_package_share_directory('aruco_docking')
    config_path = LaunchConfiguration('config_file').perform(context)
    if not config_path:
        config_path = os.path.join(pkg, 'config', config_file_name)

    use_sim_time = _bool_text(LaunchConfiguration('use_sim_time').perform(context))
    start_charging_manager = LaunchConfiguration('start_charging_manager')
    spawn_gazebo_dock = LaunchConfiguration('spawn_gazebo_dock')
    battery = LaunchConfiguration('battery')

    with open(config_path, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}

    server_params = params.get('docking_server', {}).get('ros__parameters', {})
    docks = server_params.get('docks', ['base_carregamento'])
    dock_id = docks[0] if docks else 'base_carregamento'
    dock_pose = server_params.get(dock_id, {}).get('pose')
    if not dock_pose or len(dock_pose) < 3:
        raise RuntimeError(
            f'Missing docking_server.ros__parameters.{dock_id}.pose in {config_path}')

    dock_x, dock_y, dock_yaw = [float(value) for value in dock_pose[:3]]
    fixed_frame = str(server_params.get('fixed_frame', server_params.get(dock_id, {}).get('frame', 'odom')))

    nodes = []

    if spawn_gazebo_dock_default is not None:
        models_path = os.path.join(pkg, 'models')
        dock_model_path = os.path.join(models_path, 'charging_dock')
        dock_sdf = os.path.join(dock_model_path, 'model.sdf')
        with open(dock_sdf, 'r', encoding='utf-8') as f:
            dock_sdf_text = f.read().replace(
                'model://charging_dock/', f'file://{dock_model_path}/')
        fd, dock_sdf_abs = tempfile.mkstemp(prefix='charging_dock_', suffix='.sdf')
        os.close(fd)
        with open(dock_sdf_abs, 'w', encoding='utf-8') as f:
            f.write(dock_sdf_text)
        nodes.append(AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH', value=models_path, separator=':'))
        nodes.append(LogInfo(
            condition=IfCondition(spawn_gazebo_dock),
            msg=f'Spawning charging_dock in Gazebo at x={dock_x:.2f}, y={dock_y:.2f}, yaw={dock_yaw:.2f}'))
        nodes.append(GroupAction(
            condition=IfCondition(spawn_gazebo_dock),
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-file', dock_sdf_abs,
                    '-name', 'charging_dock',
                    '-x', str(dock_x),
                    '-y', str(dock_y),
                    '-z', '0.0',
                    '-Y', str(dock_yaw),
                ])]))

    nodes.extend([
        Node(
            package='aruco_docking',
            executable='dock_pose_estimator.py',
            name='dock_pose_estimator',
            output='screen',
            parameters=[config_path, {
                'use_sim_time': use_sim_time,
                'target_frame': fixed_frame,
                'dock_yaw': dock_yaw,
            }]),
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
                package='aruco_docking',
                executable='charging_manager.py',
                name='charging_manager',
                output='screen',
                parameters=[config_path, {
                    'use_sim_time': use_sim_time,
                    'battery': battery,
                }])]),
    ])
    return nodes


def _launch_description(*, use_sim_time, config_file_name, spawn_gazebo_dock):
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=str(use_sim_time).lower()),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('start_charging_manager', default_value='true'),
        DeclareLaunchArgument('battery', default_value='100.0'),
        DeclareLaunchArgument('spawn_gazebo_dock', default_value=str(spawn_gazebo_dock).lower()),
        OpaqueFunction(function=lambda context: _configured_nodes(
            context,
            use_sim_time_default=use_sim_time,
            config_file_name=config_file_name,
            spawn_gazebo_dock_default=spawn_gazebo_dock)),
    ])


def generate_launch_description():
    return _launch_description(
        use_sim_time=False,
        config_file_name='docking_params_real.yaml',
        spawn_gazebo_dock=False)
