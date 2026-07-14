import json
import math

from lidar_auto_docking.dock_pose_saver_core import atomic_write_json
from lidar_auto_docking.dock_pose_saver_core import build_legacy_record
from lidar_auto_docking.dock_pose_saver_core import DetectionSnapshot
from lidar_auto_docking.dock_pose_saver_core import is_affirmative
from lidar_auto_docking.dock_pose_saver_core import PoseData
from lidar_auto_docking.dock_pose_saver_core import save_snapshot
from lidar_auto_docking.dock_pose_saver_core import TransformUnavailable
from lidar_auto_docking.dock_pose_saver_core import validate_detection
from lidar_auto_docking.dock_pose_saver_core import yaw_degrees
import pytest


NOW_NS = 10_000_000_000


def make_pose(**overrides):
    values = {
        'x': 1.5,
        'y': -0.2,
        'z': 0.0,
        'qx': 0.0,
        'qy': 0.0,
        'qz': 0.0,
        'qw': 1.0,
    }
    values.update(overrides)
    return PoseData(**values)


def make_detection(**overrides):
    values = {
        'stamp_ns': NOW_NS - 100_000_000,
        'frame_id': 'laser',
        'detected': True,
        'width': 0.469,
        'point_count': 198,
        'fitness': 0.0042,
        'pose': make_pose(),
        'lidar_distance': math.hypot(1.5, -0.2),
    }
    values.update(overrides)
    return DetectionSnapshot(**values)


def valid_pose_provider(_snapshot):
    dock_map = make_pose(x=2.4, y=-1.2, qz=math.sqrt(0.5), qw=math.sqrt(0.5))
    robot_map = make_pose(x=1.0, y=-1.0, qz=0.1, qw=0.9949874371)
    return dock_map, robot_map


def test_yaw_degrees_uses_full_quaternion():
    pose = make_pose(qz=math.sqrt(0.5), qw=math.sqrt(0.5))
    assert yaw_degrees(pose) == pytest.approx(90.0)


def test_expired_detection_is_rejected():
    detection = make_detection(stamp_ns=NOW_NS - 1_100_000_000)
    valid, reason, age_sec = validate_detection(detection, NOW_NS, 1.0)
    assert not valid
    assert reason == 'a deteccao expirou'
    assert age_sec == pytest.approx(1.1)


@pytest.mark.parametrize('bad_value', [math.nan, math.inf, -math.inf])
def test_non_finite_pose_is_rejected(bad_value):
    detection = make_detection(pose=make_pose(x=bad_value))
    valid, reason, _ = validate_detection(detection, NOW_NS, 1.0)
    assert not valid
    assert 'NaN ou infinito' in reason


def test_save_without_detection_fails_and_does_not_write(tmp_path):
    output_path = tmp_path / 'dock_ws_dock.json'
    result = save_snapshot(
        None,
        NOW_NS,
        1.0,
        str(output_path),
        valid_pose_provider,
    )
    assert not result.success
    assert 'nenhuma mensagem' in result.message
    assert not output_path.exists()


def test_save_without_required_tf_fails(tmp_path):
    def missing_tf(_snapshot):
        raise TransformUnavailable('TF map <- laser indisponivel')

    output_path = tmp_path / 'dock_ws_dock.json'
    result = save_snapshot(
        make_detection(),
        NOW_NS,
        1.0,
        str(output_path),
        missing_tf,
    )
    assert not result.success
    assert 'TF map <- laser indisponivel' in result.message
    assert not output_path.exists()


def test_negative_confirmation_never_calls_saver(tmp_path):
    output_path = tmp_path / 'dock_ws_dock.json'
    calls = []

    def request_save():
        calls.append(True)
        atomic_write_json(str(output_path), {'unexpected': True})

    answer = 'n'
    if is_affirmative(answer):
        request_save()
    assert calls == []
    assert not output_path.exists()


def test_atomic_write_replaces_complete_json_and_leaves_no_temp(tmp_path):
    output_path = tmp_path / 'dock_ws_dock.json'
    output_path.write_text('{"old": true}', encoding='utf-8')
    record = build_legacy_record(*valid_pose_provider(make_detection()))

    normalized = atomic_write_json(str(output_path), record)

    assert normalized == str(output_path)
    assert json.loads(output_path.read_text(encoding='utf-8')) == record
    assert list(tmp_path.glob('.*.tmp')) == []


def test_atomic_write_preserves_symlink_install_layout(tmp_path):
    source_directory = tmp_path / 'src'
    install_directory = tmp_path / 'install'
    source_directory.mkdir()
    install_directory.mkdir()
    source_path = source_directory / 'dock_ws_dock.json'
    source_path.write_text('{"old": true}', encoding='utf-8')
    installed_path = install_directory / 'dock_ws_dock.json'
    installed_path.symlink_to(source_path)

    atomic_write_json(str(installed_path), {'x': 2.5})

    assert installed_path.is_symlink()
    assert json.loads(source_path.read_text(encoding='utf-8')) == {'x': 2.5}


def test_invalid_detection_produces_service_failure_semantics(tmp_path):
    detection = make_detection(width=math.nan)
    result = save_snapshot(
        detection,
        NOW_NS,
        1.0,
        str(tmp_path / 'dock_ws_dock.json'),
        valid_pose_provider,
    )
    assert not result.success
    assert result.message.startswith('[ERRO] Pose nao salva.')


def test_valid_detection_produces_service_success_and_file(tmp_path):
    output_path = tmp_path / 'dock_ws_dock.json'
    result = save_snapshot(
        make_detection(),
        NOW_NS,
        1.0,
        str(output_path),
        valid_pose_provider,
    )
    assert result.success
    assert result.output_path == str(output_path)
    assert '[OK] Pose da dock salva.' in result.message
    assert output_path.exists()


def test_saved_format_preserves_all_legacy_keys(tmp_path):
    output_path = tmp_path / 'dock_ws_dock.json'
    result = save_snapshot(
        make_detection(),
        NOW_NS,
        1.0,
        str(output_path),
        valid_pose_provider,
    )
    assert result.success
    saved = json.loads(output_path.read_text(encoding='utf-8'))
    assert list(saved) == ['x', 'y', 'z', 'w', 'bx', 'by', 'bz', 'bw']
