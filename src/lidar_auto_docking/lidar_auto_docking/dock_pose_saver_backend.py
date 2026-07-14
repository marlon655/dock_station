"""ROS backend for validated, service-driven dock pose persistence."""

import threading

from lidar_auto_docking.dock_pose_saver_core import save_snapshot
from lidar_auto_docking.dock_pose_saver_ros import lookup_map_poses
from lidar_auto_docking.dock_pose_saver_ros import snapshot_from_message
from lidar_auto_docking_messages.msg import DockDetection
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DockPoseSaverBackend(Node):
    """Keep the latest detection and expose an atomic save service."""

    def __init__(self):
        super().__init__('dock_pose_saver')
        self.declare_parameter('load_file_path', '')
        self.declare_parameter('detection_timeout_sec', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self._output_path = self.get_parameter(
            'load_file_path'
        ).get_parameter_value().string_value
        self._detection_timeout_sec = self.get_parameter(
            'detection_timeout_sec'
        ).get_parameter_value().double_value
        self._map_frame = self.get_parameter(
            'map_frame'
        ).get_parameter_value().string_value
        self._base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value

        self._lock = threading.Lock()
        self._latest_detection = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            DockDetection,
            '/dock_detection_info',
            self._detection_callback,
            qos,
        )
        self._service = self.create_service(
            Trigger,
            '/save_dock_pose',
            self._save_callback,
        )
        self.get_logger().info(
            f'Dock pose saver backend pronto. Saida: {self._output_path}'
        )

    def _detection_callback(self, message):
        snapshot = snapshot_from_message(message)
        with self._lock:
            self._latest_detection = snapshot

    def _pose_provider(self, snapshot):
        return lookup_map_poses(
            self._tf_buffer,
            snapshot,
            self._map_frame,
            self._base_frame,
            timeout_sec=0.5,
        )

    def _save_current_detection(self):
        with self._lock:
            snapshot = self._latest_detection
        return save_snapshot(
            snapshot=snapshot,
            now_ns=self.get_clock().now().nanoseconds,
            timeout_sec=self._detection_timeout_sec,
            output_path=self._output_path,
            pose_provider=self._pose_provider,
        )

    def _save_callback(self, request, response):
        del request
        result = self._save_current_detection()
        response.success = result.success
        response.message = result.message
        if result.success:
            self.get_logger().info(result.message.replace('\n', ' | '))
        else:
            self.get_logger().error(result.message.replace('\n', ' | '))
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseSaverBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
