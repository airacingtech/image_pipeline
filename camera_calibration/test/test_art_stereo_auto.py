from __future__ import annotations

import csv
import json
from unittest import mock

from camera_calibration.nodes import art_stereo_auto


def _write_manifest(path, count):
    path.mkdir(parents=True, exist_ok=True)
    with (path / 'pairs.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['index'])
        for index in range(count):
            writer.writerow([index])


def test_auto_pipeline_collects_solves_and_writes_pass_progress(tmp_path):
    output = tmp_path / 'session'

    def run(command):
        if any('art_stereo_capture' in item for item in command):
            _write_manifest(output / 'capture', 15)
            return 0
        (output / 'result').mkdir(parents=True, exist_ok=True)
        (output / 'result' / 'report.json').write_text(
            json.dumps({'overall': 'PASS'}), encoding='utf-8')
        return 0

    with mock.patch.object(art_stereo_auto, 'run_stage', side_effect=run) as runner:
        return_code = art_stereo_auto.main([
            '--output', str(output), '--max-pairs', '15'])

    assert return_code == 0
    assert runner.call_count == 2
    progress = json.loads((output / 'progress.json').read_text())
    assert progress['status'] == 'saved'
    assert progress['samples'] == 15
    assert progress['overall'] == 'PASS'


def test_auto_pipeline_stops_before_solve_when_capture_fails(tmp_path):
    output = tmp_path / 'session'

    def run_capture(command):
        assert any('art_stereo_capture' in item for item in command)
        _write_manifest(output / 'capture', 4)
        return 3

    with mock.patch.object(
        art_stereo_auto, 'run_stage', side_effect=run_capture
    ) as runner:
        return_code = art_stereo_auto.main([
            '--output', str(output), '--max-pairs', '15'])

    assert return_code == 3
    runner.assert_called_once()
    progress = json.loads((output / 'progress.json').read_text())
    assert progress['status'] == 'capture_failed'
    assert progress['samples'] == 4


def test_auto_capture_command_enforces_2k_and_visible_vehicle_window(tmp_path):
    args = art_stereo_auto.parse_args([
        '--output', str(tmp_path / 'session'), '--max-pairs', '15'])

    command = art_stereo_auto.build_capture_command(args, tmp_path / 'capture')

    assert command[command.index('--expected-width') + 1] == '2064'
    assert command[command.index('--expected-height') + 1] == '1544'
    assert '--show-window' in command


def test_auto_pipeline_rejects_success_without_pass_report(tmp_path):
    output = tmp_path / 'session'

    def run(command):
        if any('art_stereo_capture' in item for item in command):
            _write_manifest(output / 'capture', 15)
        return 0

    with mock.patch.object(art_stereo_auto, 'run_stage', side_effect=run):
        return_code = art_stereo_auto.main([
            '--output', str(output), '--max-pairs', '15'])

    assert return_code == 4
    progress = json.loads((output / 'progress.json').read_text())
    assert progress['status'] == 'failed'
    assert progress['return_code'] == 4
