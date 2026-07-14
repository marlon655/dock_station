import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lidar_auto_docking')
    dock_stl = os.path.join(package_share, 'Dock.stl')
    config = os.path.join(package_share, 'config', 'autodock_params.yaml')

    dock_x = -7.57
    dock_y = 13.21
    dock_z = 0.0
    mesh_roll = 1.57
    mesh_yaw = 1.57
    dock_yaw = 2.9462 + mesh_yaw

    dock_sdf = f'''<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="lidar_auto_dock">
    <static>true</static>
    <link name="body">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>file://{dock_stl}</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>file://{dock_stl}</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
'''

    fd, dock_sdf_path = tempfile.mkstemp(
        prefix='lidar_auto_dock_',
        suffix='.sdf',
    )
    os.close(fd)
    with open(dock_sdf_path, 'w', encoding='utf-8') as sdf_file:
        sdf_file.write(dock_sdf)

    return LaunchDescription([
        LogInfo(
            msg=f'Spawning lidar_auto_dock at x={dock_x:.2f}, y={dock_y:.2f}, yaw={dock_yaw:.2f}'),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-file', dock_sdf_path,
                '-name', 'lidar_auto_dock',
                '-x', str(dock_x),
                '-y', str(dock_y),
                '-z', str(dock_z),
                '-R', str(mesh_roll),
                '-P', '0.0',
                '-Y', str(dock_yaw),
            ],
        ),
        Node(
            package='lidar_auto_docking',
            executable='auto_dock',
            parameters=[config],
            output='screen',
        ),
    ])
