import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _as_launch_bool(value):
    return 'True' if bool(value) else 'False'


def _resolve_package_path(package_share, value):
    if value is None:
        return ''

    value = os.path.expanduser(os.path.expandvars(str(value)))
    if value == '' or os.path.isabs(value):
        return value

    return os.path.join(package_share, value)


def _load_launch_defaults(package_share):
    config_path = os.path.join(package_share, 'config', 'launch_params.yaml')
    if not os.path.exists(config_path):
        return {}

    with open(config_path, 'r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file) or {}


def generate_launch_description():
    sim_bot_share = get_package_share_directory('sim_bot')
    nav_hub_share = get_package_share_directory('nav_hub')
    launch_defaults = _load_launch_defaults(sim_bot_share)

    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    robot_urdf = LaunchConfiguration('robot_urdf')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    bridge_config = LaunchConfiguration('bridge_config')
    rviz_config = LaunchConfiguration('rviz_config')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    joy = LaunchConfiguration('joy')
    tof_lidar = LaunchConfiguration('tof_lidar')
    camera = LaunchConfiguration('camera')
    nav_params_file = LaunchConfiguration('nav_params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    world_default = _resolve_package_path(
        sim_bot_share,
        launch_defaults.get('world', os.path.join(sim_bot_share, 'worlds', 'aceleradora.world')))
    robot_urdf_default = _resolve_package_path(
        sim_bot_share,
        launch_defaults.get('robot_urdf', os.path.join(sim_bot_share, 'description', 'palmares_bot.urdf.xacro')))
    bridge_config_default = _resolve_package_path(
        sim_bot_share,
        launch_defaults.get('bridge_config', os.path.join(sim_bot_share, 'config', 'gz_bridge.yaml')))
    rviz_config_default = _resolve_package_path(
        sim_bot_share,
        launch_defaults.get('rviz_config', os.path.join(sim_bot_share, 'rviz', 'bot.rviz')))
    nav_params_file_default = os.path.join(nav_hub_share, 'config', 'sim_nav_params.yaml')

    sim_essentials = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_bot_share, 'launch', 'sim_essentials.launch.py')),
        launch_arguments={
            'world': world,
            'robot_name': robot_name,
            'robot_urdf': robot_urdf,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'bridge_config': bridge_config,
            'rviz_config': rviz_config,
            'headless': headless,
            'rviz': rviz,
            'joy': joy,
            'tof_lidar': tof_lidar,
            'camera': camera,
            'use_sim_time': 'true',
        }.items())

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_hub_share, 'launch', 'localization_only.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': autostart,
            'params_file': nav_params_file,
            'log_level': log_level,
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_default),
        DeclareLaunchArgument(
            'robot_name',
            default_value=str(launch_defaults.get('robot_name', 'palmares_bot'))),
        DeclareLaunchArgument('robot_urdf', default_value=robot_urdf_default),
        DeclareLaunchArgument(
            'spawn_x',
            default_value=str(launch_defaults.get('spawn_x', -0.2621192932128906))),
        DeclareLaunchArgument(
            'spawn_y',
            default_value=str(launch_defaults.get('spawn_y', 12.350018501281738))),
        DeclareLaunchArgument(
            'spawn_z',
            default_value=str(launch_defaults.get('spawn_z', 0.15))),
        DeclareLaunchArgument('bridge_config', default_value=bridge_config_default),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config_default),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument(
            'rviz',
            default_value=_as_launch_bool(launch_defaults.get('rviz', True))),
        DeclareLaunchArgument(
            'joy',
            default_value=_as_launch_bool(launch_defaults.get('joy', False))),
        DeclareLaunchArgument(
            'tof_lidar',
            default_value=_as_launch_bool(launch_defaults.get('tof_lidar', True))),
        DeclareLaunchArgument(
            'camera',
            default_value=_as_launch_bool(launch_defaults.get('camera', True))),
        DeclareLaunchArgument('nav_params_file', default_value=nav_params_file_default),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        sim_essentials,
        localization,
    ])
