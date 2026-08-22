from __future__ import annotations

from http.server import ThreadingHTTPServer
import io
import json
from pathlib import Path
import tarfile
import threading
from urllib.request import urlopen

from camera_calibration.nodes.art_calibration_ui import (
    build_action_command,
    CalibrationApp,
    EXPECTED_HEIGHT,
    EXPECTED_WIDTH,
    make_handler,
    mono_archive_summary,
    RELAY_CAMERA_BY_TASK,
    stream_gate,
    STREAM_TOPICS,
    task_for,
    task_paths,
    TASKS,
    validate_session_name,
)
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def healthy_streams() -> dict:
    streams = {
        key: {
            'topic': topic,
            'online': True,
            'rate_hz': 10.0,
            'width': EXPECTED_WIDTH,
            'height': EXPECTED_HEIGHT,
            'board_detected': True,
            'sharpness': 120.0,
        }
        for key, topic in STREAM_TOPICS.items()
    }
    streams['stereo_sync_delta_ms'] = 0.4
    return streams


def test_task_contract_has_one_stereo_pair_and_four_fisheye_cameras():
    stereo = [task for task in TASKS if task.kind == 'stereo']
    mono = [task for task in TASKS if task.kind == 'mono']

    assert len(stereo) == 1
    assert stereo[0].camera_names == (
        'vimba_front_left_center', 'vimba_front_right_center')
    assert stereo[0].model == 'pinhole'
    assert {task.camera_names[0] for task in mono} == {
        'vimba_front', 'vimba_left', 'vimba_right', 'vimba_rear'}
    assert all(task.model == 'fisheye' for task in mono)
    assert set(RELAY_CAMERA_BY_TASK) == {task.task_id for task in TASKS}
    assert RELAY_CAMERA_BY_TASK['vimba_rear'] == 'rear'


@pytest.mark.parametrize('session', ['20260822_120000', 'track-day.1', 'A_b-c'])
def test_session_name_accepts_safe_local_names(session):
    assert validate_session_name(session) == session


@pytest.mark.parametrize('session', ['', '../escape', '/tmp/result', 'has space'])
def test_session_name_rejects_path_traversal_and_unsafe_names(session):
    with pytest.raises(ValueError):
        validate_session_name(session)


def test_task_paths_stay_under_selected_data_root(tmp_path):
    paths = task_paths(tmp_path, 'safe-session', task_for('stereo_center'))

    assert paths['base'].is_relative_to(tmp_path)
    assert paths['capture'] == tmp_path / 'safe-session' / 'stereo_center' / 'capture'


def test_stereo_commands_use_guarded_native_resolution_pipeline(tmp_path):
    task = task_for('stereo_center')
    paths = task_paths(tmp_path, 'session', task)

    capture = build_action_command('capture', task, paths)
    solve = build_action_command('calibrate', task, paths, 0.3)

    assert capture[:4] == [
        'ros2', 'run', 'camera_calibration', 'art_stereo_capture']
    assert capture[capture.index('--max-pairs') + 1] == '60'
    assert capture[capture.index('--max-delta-ms') + 1] == '2.0'
    assert solve[:4] == [
        'ros2', 'run', 'camera_calibration', 'art_stereo_calibrate']
    assert solve[solve.index('--expected-width') + 1] == '2064'
    assert solve[solve.index('--expected-height') + 1] == '1544'
    assert solve[solve.index('--expected-baseline-m') + 1] == '0.3'


def test_mono_command_preserves_camera_model_wrapper_and_topic(tmp_path):
    task = task_for('vimba_rear')
    command = build_action_command(
        'calibrate', task, task_paths(tmp_path, 'session', task))

    assert command == [
        'ros2', 'run', 'camera_calibration', 'art_camera_calibrator',
        'vimba_rear', '--image-topic', '/vimba_rear/image']


def test_stream_gate_checks_size_rate_and_stereo_sync():
    streams = healthy_streams()
    passed, failures = stream_gate(task_for('stereo_center'), streams)
    assert passed
    assert failures == []

    streams['stereo_right']['width'] = 516
    streams['stereo_sync_delta_ms'] = 4.2
    passed, failures = stream_gate(task_for('stereo_center'), streams)
    assert not passed
    assert any('516x1544' in failure for failure in failures)
    assert any('above 2 ms' in failure for failure in failures)


def test_mono_archive_summary_requires_2k_equidistant(tmp_path):
    archive_path = tmp_path / 'calibrationdata.tar.gz'
    calibration = {
        'image_width': 2064,
        'image_height': 1544,
        'camera_name': 'vimba_front',
        'distortion_model': 'equidistant',
    }
    payload = json.dumps(calibration).encode('utf-8')
    info = tarfile.TarInfo('ost.yaml')
    info.size = len(payload)
    with tarfile.open(archive_path, 'w:gz') as archive:
        archive.addfile(info, io.BytesIO(payload))

    summary = mono_archive_summary(archive_path)

    assert summary is not None
    assert summary['status'] == 'PASS'
    assert summary['camera_name'] == 'vimba_front'


def test_demo_http_health_state_and_preview(tmp_path):
    assets = PACKAGE_ROOT / 'web'
    app = CalibrationApp(assets, tmp_path / 'data', demo=True)
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(app))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f'http://127.0.0.1:{server.server_address[1]}'
    try:
        with urlopen(f'{base_url}/api/health', timeout=3) as response:
            health = json.load(response)
        with urlopen(
            f'{base_url}/api/state?task=stereo_center&session=test-session',
            timeout=3,
        ) as response:
            state = json.load(response)
        with urlopen(f'{base_url}/api/frame/stereo_left.jpg', timeout=3) as response:
            image = response.read()

        assert health == {'status': 'ok', 'demo': True}
        assert state['stream_gate']['pass'] is True
        assert state['selected_task']['task_id'] == 'stereo_center'
        assert state['expected']['width'] == 2064
        assert image.startswith(b'\xff\xd8')
    finally:
        server.shutdown()
        server.server_close()
        app.close()
        thread.join(timeout=3)


def test_demo_streams_do_not_expire_while_operator_reviews_page(tmp_path):
    app = CalibrationApp(PACKAGE_ROOT / 'web', tmp_path / 'data', demo=True)
    try:
        for record in app.monitor._records.values():
            record['last_monotonic'] = 0.0

        state = app.state('stereo_center', 'long-demo')

        assert state['stream_gate']['pass'] is True
    finally:
        app.close()
