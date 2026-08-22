#!/usr/bin/env python3
"""Run the ART stereo capture and guarded offline solve as one workflow."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


def write_progress(path: Path, **values) -> None:
    """Atomically publish the current automatic stereo stage."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(
        json.dumps(values, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    with temporary.open('r+', encoding='utf-8') as stream:
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def capture_count(capture_dir: Path) -> int:
    manifest = capture_dir / 'pairs.csv'
    if not manifest.is_file():
        return 0
    with manifest.open('r', encoding='utf-8') as stream:
        return max(sum(1 for _ in stream) - 1, 0)


def build_capture_command(args: argparse.Namespace, capture_dir: Path) -> list[str]:
    command = [
        sys.executable, '-m',
        'camera_calibration.nodes.art_stereo_capture',
        '--output', str(capture_dir),
        '--left-topic', args.left_topic,
        '--right-topic', args.right_topic,
        '--mode', 'online-filter',
        '--max-pairs', str(args.max_pairs),
        '--max-delta-ms', f'{args.max_delta_ms:.9g}',
        '--min-interval-sec', f'{args.min_interval_sec:.9g}',
        '--min-blur', f'{args.min_blur:.9g}',
        '--min-novelty', f'{args.min_novelty:.9g}',
        '--expected-width', str(args.expected_width),
        '--expected-height', str(args.expected_height),
        '--show-window',
    ]
    if args.board:
        command.extend(['--board', args.board])
    return command


def build_calibration_command(
    args: argparse.Namespace,
    capture_dir: Path,
    result_dir: Path,
) -> list[str]:
    command = [
        sys.executable, '-m',
        'camera_calibration.nodes.art_stereo_calibrate',
        '--input', str(capture_dir),
        '--output', str(result_dir),
        '--expected-width', str(args.expected_width),
        '--expected-height', str(args.expected_height),
    ]
    if args.board:
        command.extend(['--board', args.board])
    if args.expected_baseline_m is not None:
        command.extend([
            '--expected-baseline-m', f'{args.expected_baseline_m:.9g}'])
    return command


def run_stage(command: list[str]) -> int:
    process = subprocess.Popen(command)
    try:
        return process.wait()
    except KeyboardInterrupt:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5.0)
        raise


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output', required=True,
        help='new vehicle-side session directory containing capture and result')
    parser.add_argument('--board', help='optional checkerboard YAML override')
    parser.add_argument('--left-topic', default='/vimba_calib_left/image')
    parser.add_argument('--right-topic', default='/vimba_calib_right/image')
    parser.add_argument('--max-pairs', type=int, default=60)
    parser.add_argument('--max-delta-ms', type=float, default=2.0)
    parser.add_argument('--min-interval-sec', type=float, default=1.0)
    parser.add_argument('--min-blur', type=float, default=30.0)
    parser.add_argument('--min-novelty', type=float, default=0.025)
    parser.add_argument('--expected-width', type=int, default=2064)
    parser.add_argument('--expected-height', type=int, default=1544)
    parser.add_argument('--expected-baseline-m', type=float)
    args = parser.parse_args(argv)

    if args.max_pairs < 15:
        parser.error('--max-pairs must be at least 15')
    if args.max_delta_ms <= 0:
        parser.error('--max-delta-ms must be positive')
    if args.min_interval_sec < 0 or args.min_blur < 0 or args.min_novelty < 0:
        parser.error('interval, blur, and novelty thresholds must be non-negative')
    if args.expected_width <= 0 or args.expected_height <= 0:
        parser.error('expected image dimensions must be positive')
    if args.expected_baseline_m is not None and args.expected_baseline_m <= 0:
        parser.error('--expected-baseline-m must be positive')
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    output = Path(args.output).expanduser().resolve()
    capture_dir = output / 'capture'
    result_dir = output / 'result'
    progress_path = output / 'progress.json'

    for directory in (capture_dir, result_dir):
        if directory.exists() and any(directory.iterdir()):
            raise SystemExit(
                f'{directory} is not empty; choose a new output directory')
    output.mkdir(parents=True, exist_ok=True)

    print('ART_STEREO_STAGE=collecting', flush=True)
    write_progress(
        progress_path,
        status='collecting',
        samples=0,
        capture=str(capture_dir),
        result=str(result_dir),
    )
    try:
        capture_return_code = run_stage(build_capture_command(args, capture_dir))
    except KeyboardInterrupt:
        write_progress(
            progress_path,
            status='stopped',
            samples=capture_count(capture_dir),
            capture=str(capture_dir),
            result=str(result_dir),
        )
        return 130

    samples = capture_count(capture_dir)
    if capture_return_code != 0:
        write_progress(
            progress_path,
            status='capture_failed',
            samples=samples,
            capture_return_code=capture_return_code,
            capture=str(capture_dir),
            result=str(result_dir),
        )
        return capture_return_code

    print('ART_STEREO_STAGE=solving', flush=True)
    write_progress(
        progress_path,
        status='solving',
        samples=samples,
        capture=str(capture_dir),
        result=str(result_dir),
    )
    try:
        calibration_return_code = run_stage(
            build_calibration_command(args, capture_dir, result_dir))
    except KeyboardInterrupt:
        write_progress(
            progress_path,
            status='stopped',
            samples=samples,
            capture=str(capture_dir),
            result=str(result_dir),
        )
        return 130

    report_path = result_dir / 'report.json'
    overall = None
    if report_path.is_file():
        try:
            overall = json.loads(report_path.read_text(encoding='utf-8')).get(
                'overall')
        except (OSError, json.JSONDecodeError):
            overall = 'INVALID'
    status = 'saved' if calibration_return_code == 0 and overall == 'PASS' else 'failed'
    final_return_code = calibration_return_code
    if status != 'saved' and final_return_code == 0:
        final_return_code = 4
    write_progress(
        progress_path,
        status=status,
        samples=samples,
        overall=overall,
        calibration_return_code=calibration_return_code,
        return_code=final_return_code,
        capture=str(capture_dir),
        result=str(result_dir),
    )
    print(f'ART_STEREO_STAGE={status}', flush=True)
    return final_return_code


if __name__ == '__main__':
    sys.exit(main())
