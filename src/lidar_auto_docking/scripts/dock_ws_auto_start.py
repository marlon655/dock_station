#!/usr/bin/env python3

import json

from action_msgs.msg import GoalInfo, GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from lidar_auto_docking_messages.action import Dock
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32


class DockWsAutoStart(Node):
    def __init__(self):
        super().__init__('dock_ws_auto_start')

        self.declare_parameter('dock_destination', 99)
        self.declare_parameter('load_file_path', '')
        self.declare_parameter('zero_cmd_count', 10)

        self._dock_destination = self.get_parameter(
            'dock_destination').get_parameter_value().integer_value
        self._load_file_path = self.get_parameter(
            'load_file_path').get_parameter_value().string_value
        self._zero_cmd_count = self.get_parameter(
            'zero_cmd_count').get_parameter_value().integer_value

        self._last_destination = None
        self._has_reached = False
        self._dock_sent = False

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cancel_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
        )
        self._dock_client = ActionClient(self, Dock, 'Dock')

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            Int32,
            '/destination',
            self._destination_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/has_reached',
            self._has_reached_callback,
            latched_qos,
        )

        self.get_logger().info(
            f'Aguardando destination={self._dock_destination} e has_reached=true.')

    def _destination_callback(self, msg):
        self._last_destination = msg.data
        self.get_logger().info(f'Recebido /destination={msg.data}')
        if msg.data != self._dock_destination:
            self._dock_sent = False
        self._maybe_start_docking()

    def _has_reached_callback(self, msg):
        self._has_reached = msg.data
        self.get_logger().info(
            f'Recebido /has_reached={msg.data} '
            f'last_destination={self._last_destination}')
        self._maybe_start_docking()

    def _maybe_start_docking(self):
        if not self._has_reached:
            return
        if self._last_destination != self._dock_destination:
            return
        if self._dock_sent:
            return

        self._dock_sent = True
        self.get_logger().info(
            'Destino de dock atingido. Cancelando Nav2 e iniciando autodocking.')
        self._cancel_nav2_goal()
        self._publish_zero_cmd()
        self._send_dock_goal()

    def _cancel_nav2_goal(self):
        if not self._cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'Servico cancel_goal do Nav2 indisponivel; seguindo com docking.')
            return

        request = CancelGoal.Request()
        request.goal_info = GoalInfo()
        self._cancel_client.call_async(request)

    def _publish_zero_cmd(self):
        zero = Twist()
        for _ in range(max(1, self._zero_cmd_count)):
            self._cmd_vel_pub.publish(zero)

    def _send_dock_goal(self):
        if not self._load_file_path:
            self.get_logger().error('Parametro load_file_path esta vazio.')
            return

        with open(self._load_file_path, encoding='utf-8') as dock_file:
            dock_pose = json.load(dock_file)

        if not self._dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server Dock indisponivel.')
            return

        goal_msg = Dock.Goal()
        goal_msg.dock_pose.header.frame_id = 'map'
        goal_msg.dock_pose.pose.position.x = dock_pose['x']
        goal_msg.dock_pose.pose.position.y = dock_pose['y']
        goal_msg.dock_pose.pose.orientation.z = dock_pose['z']
        goal_msg.dock_pose.pose.orientation.w = dock_pose['w']

        future = self._dock_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Dock goal rejeitado.')
            self._dock_sent = False
            return

        self.get_logger().info('Dock goal aceito.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED and result.docked:
            self.get_logger().info('Dock concluido com sucesso.')
            return

        self.get_logger().warn('Dock terminou sem sucesso.')
        self._dock_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = DockWsAutoStart()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
