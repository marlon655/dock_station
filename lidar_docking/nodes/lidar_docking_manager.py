#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Empty, Int32, String


IDLE = 'IDLE'
NAVIGATING_TO_DOCK_STAGING = 'NAVIGATING_TO_DOCK_STAGING'
LIDAR_DOCKING = 'LIDAR_DOCKING'
DOCKED = 'DOCKED'
FAILED = 'FAILED'


class LidarDockingManager(Node):
    def __init__(self):
        super().__init__('lidar_docking_manager')

        self.declare_parameter('destination_topic', 'destination')
        self.declare_parameter('reached_topic', 'has_reached')
        self.declare_parameter('dock_pose_topic', 'detected_dock_pose')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('dock_destination_sequence', 16)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('nav_timeout_s', 120.0)
        self.declare_parameter('detection_timeout_s', 2.0)
        self.declare_parameter('docking_timeout_s', 45.0)
        self.declare_parameter('robot_length', 0.50)
        self.declare_parameter('contact_clearance', 0.03)
        self.declare_parameter('final_distance_tolerance', 0.015)
        self.declare_parameter('lateral_tolerance', 0.025)
        self.declare_parameter('yaw_tolerance_deg', 4.0)
        self.declare_parameter('drive_lateral_tolerance', 0.06)
        self.declare_parameter('drive_yaw_tolerance_deg', 10.0)
        self.declare_parameter('near_panel_distance', 0.45)
        self.declare_parameter('linear_gain', 0.7)
        self.declare_parameter('lateral_gain', 1.8)
        self.declare_parameter('yaw_gain', 1.2)
        self.declare_parameter('max_linear_speed', 0.05)
        self.declare_parameter('min_linear_speed', 0.010)
        self.declare_parameter('max_angular_speed', 0.45)

        self.destination_topic = str(self.get_parameter('destination_topic').value)
        self.reached_topic = str(self.get_parameter('reached_topic').value)
        self.dock_pose_topic = str(self.get_parameter('dock_pose_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.dock_sequence = int(self.get_parameter('dock_destination_sequence').value)
        self.nav_timeout_s = float(self.get_parameter('nav_timeout_s').value)
        self.detection_timeout_s = float(self.get_parameter('detection_timeout_s').value)
        self.docking_timeout_s = float(self.get_parameter('docking_timeout_s').value)
        robot_length = float(self.get_parameter('robot_length').value)
        clearance = float(self.get_parameter('contact_clearance').value)
        self.final_face_distance = robot_length / 2.0 + clearance
        self.final_distance_tolerance = float(self.get_parameter('final_distance_tolerance').value)
        self.lateral_tolerance = float(self.get_parameter('lateral_tolerance').value)
        self.yaw_tolerance = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
        self.drive_lateral_tolerance = float(self.get_parameter('drive_lateral_tolerance').value)
        self.drive_yaw_tolerance = math.radians(float(self.get_parameter('drive_yaw_tolerance_deg').value))
        self.near_panel_distance = float(self.get_parameter('near_panel_distance').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.lateral_gain = float(self.get_parameter('lateral_gain').value)
        self.yaw_gain = float(self.get_parameter('yaw_gain').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.destination_pub = self.create_publisher(Int32, self.destination_topic, latched_qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, 'lidar_docking/state', 10)

        self.create_subscription(Bool, self.reached_topic, self._reached_cb, latched_qos)
        self.create_subscription(PoseStamped, self.dock_pose_topic, self._dock_pose_cb, 10)
        self.create_subscription(Empty, 'go_lidar_dock', self._go_cb, 10)

        self._state = IDLE
        self._state_started = self.get_clock().now()
        self._last_pose: Optional[PoseStamped] = None
        self._last_pose_time = None

        self.create_timer(0.1, self._control_tick)

        self._auto_start_timer = None
        if bool(self.get_parameter('auto_start').value):
            self._auto_start_timer = self.create_timer(1.0, self._auto_start_once)

        self.get_logger().info(
            'lidar_docking_manager pronto. Publique std_msgs/Empty em /go_lidar_dock para iniciar.')

    def _auto_start_once(self):
        if self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None
        if self._state == IDLE:
            self.start_docking()

    def _go_cb(self, _msg: Empty):
        self.start_docking()

    def start_docking(self):
        if self._state in (NAVIGATING_TO_DOCK_STAGING, LIDAR_DOCKING):
            self.get_logger().warn('Docking ja esta em andamento.')
            return

        msg = Int32()
        msg.data = self.dock_sequence
        self.destination_pub.publish(msg)
        self._set_state(NAVIGATING_TO_DOCK_STAGING)
        self.get_logger().info(
            'Destino de staging publicado no grafo: sequence=%d' % self.dock_sequence)

    def _reached_cb(self, msg: Bool):
        if self._state == NAVIGATING_TO_DOCK_STAGING and msg.data:
            self._set_state(LIDAR_DOCKING)
            self.get_logger().info('Staging alcancado. Iniciando centralizacao por LiDAR.')

    def _dock_pose_cb(self, msg: PoseStamped):
        self._last_pose = msg
        self._last_pose_time = self.get_clock().now()

    def _control_tick(self):
        self._publish_state()

        if self._state == NAVIGATING_TO_DOCK_STAGING:
            if self._state_age_s() > self.nav_timeout_s:
                self._fail('Timeout esperando has_reached do route graph.')
            return

        if self._state != LIDAR_DOCKING:
            return

        if self._state_age_s() > self.docking_timeout_s:
            self._fail('Timeout na aproximacao final por LiDAR.')
            return

        if self._last_pose is None or self._last_pose_time is None:
            if self._state_age_s() > self.detection_timeout_s:
                self._fail('Dock nao detectado pelo LiDAR.')
            return

        pose_age = (self.get_clock().now() - self._last_pose_time).nanoseconds / 1e9
        if pose_age > self.detection_timeout_s:
            self._fail('Deteccao do dock ficou stale.')
            return

        face_distance = float(self._last_pose.pose.position.x)
        lateral_error = float(self._last_pose.pose.position.y)
        yaw_error = self._yaw_from_pose(self._last_pose)
        distance_error = face_distance - self.final_face_distance

        if (distance_error <= self.final_distance_tolerance and
                abs(lateral_error) <= self.lateral_tolerance and
                abs(yaw_error) <= self.yaw_tolerance):
            self._stop()
            self._set_state(DOCKED)
            self.get_logger().info('Dock concluido por LiDAR.')
            return

        cmd = Twist()
        cmd.angular.z = self._clip(
            self.lateral_gain * lateral_error + self.yaw_gain * yaw_error,
            -self.max_angular_speed,
            self.max_angular_speed)

        aligned_enough_to_drive = (
            abs(lateral_error) <= self.drive_lateral_tolerance and
            abs(yaw_error) <= self.drive_yaw_tolerance)

        if distance_error > self.final_distance_tolerance and aligned_enough_to_drive:
            speed = self._clip(
                self.linear_gain * distance_error,
                self.min_linear_speed,
                self.max_linear_speed)
            if face_distance < self.near_panel_distance:
                speed *= max(0.25, distance_error / max(0.01, self.near_panel_distance))
            cmd.linear.x = speed

        self.cmd_pub.publish(cmd)

    def _set_state(self, state: str):
        self._state = state
        self._state_started = self.get_clock().now()
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self._state
        self.state_pub.publish(msg)

    def _state_age_s(self) -> float:
        return (self.get_clock().now() - self._state_started).nanoseconds / 1e9

    def _fail(self, reason: str):
        self._stop()
        self._set_state(FAILED)
        self.get_logger().error(reason)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _yaw_from_pose(msg: PoseStamped) -> float:
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        return 2.0 * math.atan2(z, w)

    @staticmethod
    def _clip(value: float, low: float, high: float) -> float:
        if value < low:
            return low
        if value > high:
            return high
        return value


def main():
    rclpy.init()
    node = LidarDockingManager()
    try:
        rclpy.spin(node)
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
