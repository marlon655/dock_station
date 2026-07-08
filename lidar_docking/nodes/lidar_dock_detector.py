#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan


class LidarDockDetector(Node):
    """Detecta um painel plano frontal usando apenas LaserScan 2D.

    A pose publicada em detected_dock_pose usa frame_id=base_link:
      position.x = distancia do centro do robo ate a face do painel
      position.y = erro lateral do centro do painel
      yaw        = inclinacao da face detectada em relacao ao robo
    """

    def __init__(self):
        super().__init__('lidar_dock_detector')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('dock_pose_topic', 'detected_dock_pose')
        self.declare_parameter('panel_width', 0.40)
        self.declare_parameter('panel_width_tolerance', 0.12)
        self.declare_parameter('sector_half_deg', 70.0)
        self.declare_parameter('min_detect_range', 0.18)
        self.declare_parameter('max_detect_range', 1.80)
        self.declare_parameter('max_lateral_search', 0.75)
        self.declare_parameter('cluster_gap', 0.08)
        self.declare_parameter('min_cluster_points', 5)
        self.declare_parameter('max_line_rmse', 0.035)
        self.declare_parameter('max_face_yaw_deg', 25.0)
        self.declare_parameter('max_stale_scan_s', 0.5)
        self.declare_parameter('publish_rate_hz', 15.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.dock_pose_topic = str(self.get_parameter('dock_pose_topic').value)
        self.panel_width = float(self.get_parameter('panel_width').value)
        self.panel_width_tolerance = float(self.get_parameter('panel_width_tolerance').value)
        self.sector_half = math.radians(float(self.get_parameter('sector_half_deg').value))
        self.min_range = float(self.get_parameter('min_detect_range').value)
        self.max_range = float(self.get_parameter('max_detect_range').value)
        self.max_lateral = float(self.get_parameter('max_lateral_search').value)
        self.cluster_gap = float(self.get_parameter('cluster_gap').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.max_line_rmse = float(self.get_parameter('max_line_rmse').value)
        self.max_face_yaw = math.radians(float(self.get_parameter('max_face_yaw_deg').value))
        self.max_stale_scan_s = float(self.get_parameter('max_stale_scan_s').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._last_scan_time = None
        self._last_detection: Optional[Tuple[float, float, float]] = None

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.dock_pose_topic, 10)
        self.create_timer(1.0 / max(1.0, publish_rate_hz), self._publish_detection)

        self.get_logger().info(
            'lidar_dock_detector pronto: painel %.2f m, scan=%s, pose=%s'
            % (self.panel_width, self.scan_topic, self.dock_pose_topic))

    def _scan_cb(self, msg: LaserScan):
        points = self._scan_to_points(msg)
        self._last_scan_time = self.get_clock().now()
        self._last_detection = self._detect_panel(points)

    def _scan_to_points(self, msg: LaserScan) -> np.ndarray:
        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and self.min_range <= r <= self.max_range and abs(angle) <= self.sector_half:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if x > 0.0 and abs(y) <= self.max_lateral:
                    pts.append((x, y))
            angle += msg.angle_increment
        if not pts:
            return np.empty((0, 2), dtype=np.float64)
        return np.array(pts, dtype=np.float64)

    def _clusters(self, points: np.ndarray) -> List[np.ndarray]:
        if len(points) == 0:
            return []
        clusters = []
        current = [points[0]]
        for point in points[1:]:
            if np.linalg.norm(point - current[-1]) <= self.cluster_gap:
                current.append(point)
            else:
                if len(current) >= self.min_cluster_points:
                    clusters.append(np.array(current))
                current = [point]
        if len(current) >= self.min_cluster_points:
            clusters.append(np.array(current))
        return clusters

    def _detect_panel(self, points: np.ndarray) -> Optional[Tuple[float, float, float]]:
        best = None
        best_score = float('inf')

        for cluster in self._clusters(points):
            y_min = float(np.min(cluster[:, 1]))
            y_max = float(np.max(cluster[:, 1]))
            width = y_max - y_min
            if abs(width - self.panel_width) > self.panel_width_tolerance:
                continue

            ys = cluster[:, 1]
            xs = cluster[:, 0]
            if len(np.unique(np.round(ys, 4))) < 2:
                continue

            slope, intercept = np.polyfit(ys, xs, 1)
            fitted = slope * ys + intercept
            rmse = float(np.sqrt(np.mean(np.square(xs - fitted))))
            face_yaw = math.atan(slope)

            if rmse > self.max_line_rmse or abs(face_yaw) > self.max_face_yaw:
                continue

            center_y = (y_min + y_max) / 2.0
            face_x = float(intercept)
            distance_score = face_x
            center_score = abs(center_y) * 2.0
            width_score = abs(width - self.panel_width) * 3.0
            score = distance_score + center_score + width_score + rmse * 5.0

            if score < best_score:
                best_score = score
                best = (face_x, center_y, face_yaw)

        return best

    def _publish_detection(self):
        if self._last_detection is None or self._last_scan_time is None:
            return

        age = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9
        if age > self.max_stale_scan_s:
            return

        face_x, center_y, face_yaw = self._last_detection
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = float(face_x)
        msg.pose.position.y = float(center_y)
        msg.pose.orientation.z = math.sin(face_yaw / 2.0)
        msg.pose.orientation.w = math.cos(face_yaw / 2.0)
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = LidarDockDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
