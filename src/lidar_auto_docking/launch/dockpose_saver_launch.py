import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lidar_auto_docking')
    config = os.path.join(package_share, 'config', 'autodock_params.yaml')
    load_file_path = os.path.join(
        package_share,
        'initial_dock_pose',
        'dock_ws_dock.json',
    )

    def launch_nodes(context):
        parameter_overrides = {
            'reset_goal_button': 1,
            'initial_dock_distance': 1.5,
        }
        scan_rotation_deg = LaunchConfiguration('scan_rotation_deg').perform(
            context
        )
        if scan_rotation_deg:
            parameter_overrides['scan_rotation_deg'] = float(
                scan_rotation_deg
            )

        return [
            Node(
                package='lidar_auto_docking',
                executable='dock_coordinates',
                name='dock_coordinates',
                parameters=[config, parameter_overrides],
                output='screen',
            ),
            Node(
                package='lidar_auto_docking',
                executable='dock_saver.py',
                name='dock_pose_saver',
                parameters=[config, {'load_file_path': load_file_path}],
                output='screen',
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_rotation_deg',
            default_value='',
            description=(
                'Optional CLI override in degrees. When omitted, the value '
                'from autodock_params.yaml is used.'
            ),
        ),
        OpaqueFunction(function=launch_nodes),
    ])
