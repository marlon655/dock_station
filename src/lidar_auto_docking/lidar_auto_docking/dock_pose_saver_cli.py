"""Interactive terminal client for the dock pose saver backend."""

import math
import queue
import sys
import threading

from lidar_auto_docking.dock_pose_saver_core import is_affirmative
from lidar_auto_docking.dock_pose_saver_core import TransformUnavailable
from lidar_auto_docking.dock_pose_saver_core import validate_detection
from lidar_auto_docking.dock_pose_saver_core import yaw_degrees
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


def _value_or_unavailable(value, pattern):
    if value is None:
        return 'indisponível'
    return pattern.format(value)


class DockPoseSaverCli(Node):
    """Display detections and process terminal commands without blocking ROS."""

    def __init__(self):
        super().__init__('dock_pose_saver_cli')
        self.declare_parameter('detection_timeout_sec', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('display_refresh_hz', 0.0)

        self._detection_timeout_sec = self.get_parameter(
            'detection_timeout_sec'
        ).get_parameter_value().double_value
        self._map_frame = self.get_parameter(
            'map_frame'
        ).get_parameter_value().string_value
        self._base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value
        refresh_hz = self.get_parameter(
            'display_refresh_hz'
        ).get_parameter_value().double_value

        self._lock = threading.Lock()
        self._latest_detection = None
        self._command_queue = queue.Queue()
        self._stop_requested = False
        self._awaiting_confirmation = False
        self._last_non_tty_screen = None
        self._operation_message = None
        self._interactive = sys.stdin.isatty() and sys.stdout.isatty()

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
        self._save_client = self.create_client(Trigger, '/save_dock_pose')
        # The terminal is refreshed manually by pressing R. This keeps command
        # results visible instead of immediately clearing them. A positive
        # value still enables the optional automatic refresh, capped at 2 Hz.
        self._display_timer = None
        if math.isfinite(refresh_hz) and refresh_hz > 0.0:
            self._display_timer = self.create_timer(
                1.0 / min(refresh_hz, 2.0),
                self.render,
            )

    @property
    def stop_requested(self):
        return self._stop_requested

    def _detection_callback(self, message):
        snapshot = snapshot_from_message(message)
        with self._lock:
            self._latest_detection = snapshot

    def _snapshot(self):
        with self._lock:
            return self._latest_detection

    def _screen_text(self):
        snapshot = self._snapshot()
        now_ns = self.get_clock().now().nanoseconds
        valid, reason, age_sec = validate_detection(
            snapshot,
            now_ns,
            self._detection_timeout_sec,
        )

        if snapshot is None or not snapshot.detected:
            detected_text = 'NÃO'
        elif reason == 'a deteccao expirou':
            detected_text = 'EXPIRADO'
        else:
            detected_text = 'SIM'

        lidar_distance = None
        width = None
        point_count = None
        fitness = None
        relative_pose = None
        relative_frame = None
        dock_map = None
        robot_map = None
        base_distance = None

        if snapshot is not None:
            if math.isfinite(snapshot.lidar_distance) and snapshot.lidar_distance >= 0.0:
                lidar_distance = snapshot.lidar_distance
            if math.isfinite(snapshot.width) and snapshot.width > 0.0:
                width = snapshot.width
            if snapshot.point_count > 0:
                point_count = snapshot.point_count
            if math.isfinite(snapshot.fitness) and snapshot.fitness >= 0.0:
                fitness = snapshot.fitness
            pose_values = (
                snapshot.pose.x,
                snapshot.pose.y,
                snapshot.pose.z,
                snapshot.pose.qx,
                snapshot.pose.qy,
                snapshot.pose.qz,
                snapshot.pose.qw,
            )
            if all(math.isfinite(value) for value in pose_values):
                relative_pose = snapshot.pose
            relative_frame = snapshot.frame_id or None

        tf_reason = None
        if valid and snapshot is not None:
            try:
                dock_map, robot_map = lookup_map_poses(
                    self._tf_buffer,
                    snapshot,
                    self._map_frame,
                    self._base_frame,
                    timeout_sec=0.05,
                )
                base_distance = math.hypot(
                    dock_map.x - robot_map.x,
                    dock_map.y - robot_map.y,
                )
            except TransformUnavailable as error:
                tf_reason = str(error)

        lines = [
            '=' * 50,
            'LIDAR DOCK STATION SAVER',
            '=' * 50,
            '',
            f'Dock detectado: {detected_text}',
            'Idade da detecção: '
            + _value_or_unavailable(age_sec, '{:.2f} s'),
            '',
            'Distância LiDAR → trapézio: '
            + _value_or_unavailable(lidar_distance, '{:.3f} m'),
            'Distância base_link → dock: '
            + _value_or_unavailable(base_distance, '{:.3f} m'),
            '',
            'Largura detectada: '
            + _value_or_unavailable(width, '{:.3f} m'),
            'Quantidade de pontos: '
            + ('indisponível' if point_count is None else str(point_count)),
            'Fitness do ICP: '
            + _value_or_unavailable(fitness, '{:.6f}'),
            '',
            f'Pose do dock no frame {self._map_frame}:',
        ]

        if dock_map is None:
            lines.append('indisponível')
        else:
            lines.extend([
                f'x:   {dock_map.x: .3f} m',
                f'y:   {dock_map.y: .3f} m',
                f'yaw: {yaw_degrees(dock_map): .2f}°',
            ])

        lines.extend([
            '',
            'Pose relativa no frame do LiDAR'
            + (f' ({relative_frame})' if relative_frame else '')
            + ':',
        ])
        if relative_pose is None or not relative_frame:
            lines.append('indisponível')
        else:
            lines.extend([
                f'x:   {relative_pose.x: .3f} m',
                f'y:   {relative_pose.y: .3f} m',
                f'yaw: {yaw_degrees(relative_pose): .2f}°',
            ])

        if not valid:
            lines.extend(['', f'Estado: {reason}'])
        elif tf_reason:
            lines.extend(['', f'TF: {tf_reason}'])

        if self._operation_message:
            lines.extend(['', self._operation_message])

        lines.extend([
            '',
            '---',
            '',
            '[S] Salvar localização da dock_station',
            '[R] Atualizar dados',
            '[C] Cancelar e sair',
            '-------------------',
        ])
        if not self._awaiting_confirmation:
            lines.append('Escolha:')
        return '\n'.join(lines)

    def render(self, force=False):
        screen = self._screen_text()
        if self._interactive:
            if not self._awaiting_confirmation:
                print('\033[2J\033[H' + screen, flush=True)
            return
        if force or screen != self._last_non_tty_screen:
            print(screen, flush=True)
            self._last_non_tty_screen = screen

    def start_keyboard_reader(self):
        thread = threading.Thread(target=self._read_keyboard, daemon=True)
        thread.start()
        return thread

    def _read_keyboard(self):
        while not self._stop_requested:
            try:
                line = sys.stdin.readline()
            except (OSError, ValueError) as error:
                self._command_queue.put(('error', str(error)))
                return
            if line == '':
                self._command_queue.put(('eof', ''))
                return
            self._command_queue.put(('line', line.strip()))

    def process_commands(self):
        while True:
            try:
                command_type, value = self._command_queue.get_nowait()
            except queue.Empty:
                return

            if command_type == 'eof':
                print('\n[CANCELADO] Entrada do terminal encerrada.', flush=True)
                self._stop_requested = True
                continue
            if command_type == 'error':
                print(f'\n[ERRO] Falha ao ler o terminal: {value}', flush=True)
                self._stop_requested = True
                continue

            if self._awaiting_confirmation:
                self._awaiting_confirmation = False
                if is_affirmative(value):
                    self._request_save()
                else:
                    self._operation_message = (
                        '[CANCELADO] Nenhum arquivo foi alterado.'
                    )
                    print('\n' + self._operation_message, flush=True)
                continue

            command = value.lower()
            if command == 's':
                self._awaiting_confirmation = True
                self._operation_message = None
                print(
                    '\nConfirma o salvamento da dock station? [s/N]: ',
                    end='',
                    flush=True,
                )
            elif command == 'r':
                self.render(force=True)
            elif command in ('c', 'q'):
                print('\n[CANCELADO] Encerrando sem novas alterações.', flush=True)
                self._stop_requested = True
            elif command:
                print(f'\nOpção desconhecida: {value}', flush=True)

    def _request_save(self):
        if not self._save_client.service_is_ready():
            self._operation_message = (
                '[ERRO] Serviço /save_dock_pose indisponível.'
            )
            print('\n' + self._operation_message, flush=True)
            return
        print('\nSalvando...', flush=True)
        future = self._save_client.call_async(Trigger.Request())
        future.add_done_callback(self._save_finished)

    def _save_finished(self, future):
        try:
            response = future.result()
            self._operation_message = response.message
            print('\n' + self._operation_message, flush=True)
        except Exception as error:
            self._operation_message = (
                f'[ERRO] Falha ao chamar serviço: {error}'
            )
            print('\n' + self._operation_message, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseSaverCli()
    node.start_keyboard_reader()
    node.render(force=True)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.process_commands()
    except KeyboardInterrupt:
        print('\n[CANCELADO] Interrompido por Ctrl+C.', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
