"""ROS conversions and TF lookups shared by the dock pose saver nodes."""

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import Pose
from lidar_auto_docking.dock_pose_saver_core import DetectionSnapshot
from lidar_auto_docking.dock_pose_saver_core import PoseData
from lidar_auto_docking.dock_pose_saver_core import TransformUnavailable
from lidar_auto_docking_messages.msg import DockDetection
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose


def pose_data_from_pose(pose: Pose) -> PoseData:
    """Copy a ROS Pose into an immutable representation."""
    return PoseData(
        x=float(pose.position.x),
        y=float(pose.position.y),
        z=float(pose.position.z),
        qx=float(pose.orientation.x),
        qy=float(pose.orientation.y),
        qz=float(pose.orientation.z),
        qw=float(pose.orientation.w),
    )


def snapshot_from_message(message: DockDetection) -> DetectionSnapshot:
    """Create a thread-safe immutable snapshot from DockDetection."""
    stamp_ns = (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )
    return DetectionSnapshot(
        stamp_ns=stamp_ns,
        frame_id=message.header.frame_id,
        detected=bool(message.detected),
        width=float(message.width),
        point_count=int(message.point_count),
        fitness=float(message.fitness),
        pose=pose_data_from_pose(message.pose),
        lidar_distance=float(message.lidar_distance),
    )


def _pose_from_data(data: PoseData) -> Pose:
    pose = Pose()
    pose.position.x = data.x
    pose.position.y = data.y
    pose.position.z = data.z
    pose.orientation.x = data.qx
    pose.orientation.y = data.qy
    pose.orientation.z = data.qz
    pose.orientation.w = data.qw
    return pose


def lookup_map_poses(
    tf_buffer,
    snapshot: DetectionSnapshot,
    map_frame: str,
    base_frame: str,
    timeout_sec: float,
):
    """Return dock and robot poses in map or raise TransformUnavailable."""
    stamp_message = TimeMessage()
    stamp_message.sec = snapshot.stamp_ns // 1_000_000_000
    stamp_message.nanosec = snapshot.stamp_ns % 1_000_000_000
    detection_time = Time.from_msg(stamp_message)
    timeout = Duration(seconds=timeout_sec)

    try:
        dock_transform = tf_buffer.lookup_transform(
            map_frame,
            snapshot.frame_id,
            detection_time,
            timeout=timeout,
        )
        dock_map = do_transform_pose(
            _pose_from_data(snapshot.pose),
            dock_transform,
        )
        robot_transform = tf_buffer.lookup_transform(
            map_frame,
            base_frame,
            Time(),
            timeout=timeout,
        )
    except Exception as error:  # TF exception classes vary between ROS releases.
        raise TransformUnavailable(
            f'TF necessario indisponivel: {error}'
        ) from error

    robot_pose = Pose()
    robot_pose.position.x = robot_transform.transform.translation.x
    robot_pose.position.y = robot_transform.transform.translation.y
    robot_pose.position.z = robot_transform.transform.translation.z
    robot_pose.orientation = robot_transform.transform.rotation
    return pose_data_from_pose(dock_map), pose_data_from_pose(robot_pose)
