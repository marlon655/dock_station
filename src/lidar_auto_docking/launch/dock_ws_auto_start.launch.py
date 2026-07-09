import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lidar_auto_docking')
    load_file_path = os.path.join(
        package_share,
        'initial_dock_pose',
        'dock_ws_dock.json',
    )

    return LaunchDescription([
        Node(
            package='lidar_auto_docking',
            executable='dock_ws_auto_start.py',
            name='dock_ws_auto_start',
            emulate_tty=True,
            parameters=[{
                'dock_destination': 99,
                'load_file_path': load_file_path,
            }],
            output='screen',
        ),
    ])
