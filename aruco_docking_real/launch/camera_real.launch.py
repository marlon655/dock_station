import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('aruco_docking_real')
    cam_params = os.path.join(pkg, 'config', 'usb_cam_params.yaml')
    camera_image_topic = LaunchConfiguration('camera_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    # Resolve o caminho da calibração sem hardcodar o usuário
    home = os.path.expanduser('~')
    calib_url = f'file://{home}/.ros/camera_info/c270.yaml'

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            cam_params,
            {'camera_info_url': calib_url},    # sobrescreve o caminho do YAML
        ],
        remappings=[
            # dock_pose_estimator assina /camera/image e /camera/camera_info
            ('/image_raw',   '/camera/image'),
            ('/camera_info', '/camera/camera_info'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_image_topic', default_value='/camera/image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        camera_node,
    ])
