"""Validation and atomic persistence helpers for the dock pose saver."""

from dataclasses import dataclass
import json
import math
import os
import tempfile
from typing import Callable, Optional, Tuple


NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class PoseData:
    """ROS-independent representation of a 3D pose."""

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class DetectionSnapshot:
    """Immutable copy of one DockDetection message."""

    stamp_ns: int
    frame_id: str
    detected: bool
    width: float
    point_count: int
    fitness: float
    pose: PoseData
    lidar_distance: float


@dataclass(frozen=True)
class SaveResult:
    """Result shared by the backend service and unit tests."""

    success: bool
    message: str
    output_path: Optional[str] = None


class TransformUnavailable(RuntimeError):
    """Raised when a required TF cannot be obtained."""


def yaw_degrees(pose: PoseData) -> float:
    """Return the full quaternion yaw in degrees."""
    siny_cosp = 2.0 * (pose.qw * pose.qz + pose.qx * pose.qy)
    cosy_cosp = 1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def detection_age_sec(snapshot: DetectionSnapshot, now_ns: int) -> float:
    """Return a non-negative age based on the LaserScan timestamp."""
    return max(0.0, (now_ns - snapshot.stamp_ns) / NANOSECONDS_PER_SECOND)


def _pose_is_finite(pose: PoseData) -> bool:
    return all(math.isfinite(value) for value in (
        pose.x,
        pose.y,
        pose.z,
        pose.qx,
        pose.qy,
        pose.qz,
        pose.qw,
    ))


def validate_detection(
    snapshot: Optional[DetectionSnapshot],
    now_ns: int,
    timeout_sec: float,
) -> Tuple[bool, str, Optional[float]]:
    """Validate whether a detection can safely be displayed and saved."""
    if snapshot is None:
        return False, 'nenhuma mensagem de deteccao foi recebida', None
    age_sec = detection_age_sec(snapshot, now_ns)
    if not snapshot.detected:
        return False, 'a mensagem informa detected=false', age_sec
    if age_sec > timeout_sec:
        return False, 'a deteccao expirou', age_sec
    if not snapshot.frame_id.strip():
        return False, 'header.frame_id esta vazio', age_sec
    if not _pose_is_finite(snapshot.pose):
        return False, 'a pose possui NaN ou infinito', age_sec
    if not math.isfinite(snapshot.width) or snapshot.width <= 0.0:
        return False, 'a largura detectada e invalida', age_sec
    if snapshot.point_count <= 0:
        return False, 'a deteccao nao possui pontos', age_sec
    if not math.isfinite(snapshot.fitness) or snapshot.fitness < 0.0:
        return False, 'o fitness do ICP e invalido', age_sec
    if (
        not math.isfinite(snapshot.lidar_distance)
        or snapshot.lidar_distance < 0.0
    ):
        return False, 'a distancia do LiDAR e invalida', age_sec
    return True, 'deteccao valida', age_sec


def is_affirmative(answer: str) -> bool:
    """Accept only an explicit Portuguese yes response."""
    return answer.strip().lower() == 's'


def build_legacy_record(dock_pose: PoseData, robot_pose: PoseData) -> dict:
    """Build the exact JSON key set consumed by the existing docking nodes."""
    return {
        'x': dock_pose.x,
        'y': dock_pose.y,
        'z': dock_pose.qz,
        'w': dock_pose.qw,
        'bx': robot_pose.x,
        'by': robot_pose.y,
        'bz': robot_pose.qz,
        'bw': robot_pose.qw,
    }


def _validate_output_path(output_path: str) -> str:
    if not output_path or output_path == 'null':
        raise ValueError('o parametro load_file_path esta vazio')
    # Under --symlink-install the package-share JSON points back into src.
    # Resolve it before os.replace() so the target is atomically updated
    # without replacing the installation symlink itself.
    normalized = os.path.realpath(os.path.abspath(os.path.expanduser(output_path)))
    parent = os.path.dirname(normalized)
    if not os.path.isdir(parent):
        raise ValueError(f'o diretorio de saida nao existe: {parent}')
    if os.path.isdir(normalized):
        raise ValueError(f'o caminho de saida e um diretorio: {normalized}')
    return normalized


def atomic_write_json(output_path: str, data: dict) -> str:
    """Atomically replace a JSON file using a temporary sibling file."""
    normalized = _validate_output_path(output_path)
    parent = os.path.dirname(normalized)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=parent,
            prefix=f'.{os.path.basename(normalized)}.',
            suffix='.tmp',
            delete=False,
        ) as temporary_file:
            temporary_path = temporary_file.name
            json.dump(data, temporary_file)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
        os.chmod(temporary_path, 0o644)
        os.replace(temporary_path, normalized)
        temporary_path = None
        directory_fd = os.open(parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary_path and os.path.exists(temporary_path):
            os.unlink(temporary_path)
    return normalized


def save_snapshot(
    snapshot: Optional[DetectionSnapshot],
    now_ns: int,
    timeout_sec: float,
    output_path: str,
    pose_provider: Callable[[DetectionSnapshot], Tuple[PoseData, PoseData]],
) -> SaveResult:
    """Validate, resolve TF poses and atomically persist one snapshot."""
    valid, reason, _ = validate_detection(snapshot, now_ns, timeout_sec)
    if not valid or snapshot is None:
        return SaveResult(False, f'[ERRO] Pose nao salva.\nMotivo: {reason}')

    try:
        normalized = _validate_output_path(output_path)
        dock_pose, robot_pose = pose_provider(snapshot)
        if not _pose_is_finite(dock_pose) or not _pose_is_finite(robot_pose):
            raise ValueError('a pose transformada possui NaN ou infinito')
        record = build_legacy_record(dock_pose, robot_pose)
        atomic_write_json(normalized, record)
    except (OSError, TransformUnavailable, ValueError) as error:
        return SaveResult(
            False,
            f'[ERRO] Pose nao salva.\nMotivo: {error}',
        )

    message = (
        '[OK] Pose da dock salva.\n'
        '[OK] Pose do robo salva.\n'
        '[OK] Arquivo gravado em:\n'
        f'{normalized}'
    )
    return SaveResult(True, message, normalized)
