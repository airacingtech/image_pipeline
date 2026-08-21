#!/usr/bin/env python3
"""Calibrate the confirmed ART center stereo pair from captured PNG pairs."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import sys
from typing import Iterable

from ament_index_python.packages import get_package_share_directory
from camera_calibration.art_stereo import common_points, detect_board, Detection, load_board
import cv2
import numpy as np
import yaml


@dataclass
class View:
    index: int
    left_path: Path
    right_path: Path
    left: Detection
    right: Detection
    stereo_object: np.ndarray
    stereo_left: np.ndarray
    stereo_right: np.ndarray
    timestamp_delta_ms: float | None


@dataclass
class Model:
    mono_rms_left: float
    mono_rms_right: float
    mono_per_view_left: np.ndarray
    mono_per_view_right: np.ndarray
    stereo_rms: float
    k1: np.ndarray
    d1: np.ndarray
    k2: np.ndarray
    d2: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray
    essential: np.ndarray
    fundamental: np.ndarray
    rectify_left: np.ndarray
    rectify_right: np.ndarray
    projection_left: np.ndarray
    projection_right: np.ndarray
    disparity_to_depth: np.ndarray
    valid_roi_left: tuple[int, int, int, int]
    valid_roi_right: tuple[int, int, int, int]


def percentile(values: Iterable[float], q: float) -> float | None:
    values = np.asarray(list(values), dtype=np.float64)
    if values.size == 0:
        return None
    return float(np.percentile(values, q))


def matrix_list(matrix: np.ndarray) -> list:
    return np.asarray(matrix, dtype=np.float64).tolist()


def camera_info_matrix(matrix: np.ndarray) -> dict:
    matrix = np.asarray(matrix, dtype=np.float64)
    return {
        'rows': int(matrix.shape[0]),
        'cols': int(matrix.shape[1]),
        'data': matrix.reshape(-1).tolist(),
    }


def read_manifest(input_dir: Path) -> list[dict]:
    manifest = input_dir / 'pairs.csv'
    if not manifest.is_file():
        raise FileNotFoundError(f'missing capture manifest {manifest}')
    with manifest.open('r', newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'{manifest} contains no image pairs')
    return rows


def load_views(input_dir: Path, board) -> tuple[list[View], tuple[int, int], list[dict]]:
    views: list[View] = []
    rejected: list[dict] = []
    image_size: tuple[int, int] | None = None

    for sequence, row in enumerate(read_manifest(input_dir)):
        index = int(row.get('index', sequence))
        left_path = (input_dir / row['left_file']).resolve()
        right_path = (input_dir / row['right_file']).resolve()
        if (
            input_dir.resolve() not in left_path.parents
            or input_dir.resolve() not in right_path.parents
        ):
            raise ValueError(f'manifest pair {index} escapes input directory')
        left_image = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
        right_image = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)
        if left_image is None or right_image is None:
            rejected.append({'index': index, 'reason': 'image_read_failed'})
            continue
        if left_image.shape != right_image.shape:
            rejected.append({'index': index, 'reason': 'left_right_shape_mismatch'})
            continue
        current_size = (left_image.shape[1], left_image.shape[0])
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            rejected.append({'index': index, 'reason': 'dataset_image_size_mismatch'})
            continue

        left_detection = detect_board(left_image, board, fast=False)
        right_detection = detect_board(right_image, board, fast=False)
        if left_detection is None or right_detection is None:
            rejected.append({'index': index, 'reason': 'board_redetection_failed'})
            continue
        common = common_points(
            left_detection, right_detection, minimum_points=board.minimum_points)
        if common is None:
            rejected.append({'index': index, 'reason': 'insufficient_common_corners'})
            continue
        stereo_object, stereo_left, stereo_right, _ = common
        delta_text = row.get('delta_ms', '')
        delta_ms = float(delta_text) if delta_text not in {'', None} else None
        views.append(View(
            index=index,
            left_path=left_path,
            right_path=right_path,
            left=left_detection,
            right=right_detection,
            stereo_object=stereo_object,
            stereo_left=stereo_left,
            stereo_right=stereo_right,
            timestamp_delta_ms=delta_ms,
        ))

    if image_size is None:
        raise ValueError('no readable image pair found')
    return views, image_size, rejected


def calibrate_mono(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    # The target lenses use the normal plumb_bob model. Keep k3 fixed unless a
    # dedicated model-selection experiment proves that the extra radial term
    # improves held-out data; unconstrained k3 easily overfits a planar board.
    calibration_flags = cv2.CALIB_FIX_K3
    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-7,
    )
    if hasattr(cv2, 'calibrateCameraExtended'):
        result = cv2.calibrateCameraExtended(
            object_points,
            image_points,
            image_size,
            None,
            None,
            flags=calibration_flags,
            criteria=criteria,
        )
        rms, camera_matrix, distortion = result[:3]
        per_view = np.asarray(result[-1], dtype=np.float64).reshape(-1)
    else:  # pragma: no cover - current target OpenCV has the extended API
        rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            object_points, image_points, image_size, None, None,
            flags=calibration_flags, criteria=criteria)
        errors = []
        for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
            projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
            residual = projected.reshape(-1, 2) - img.reshape(-1, 2)
            errors.append(float(np.sqrt(np.mean(np.sum(residual * residual, axis=1)))))
        per_view = np.asarray(errors)
    return float(rms), camera_matrix, distortion, per_view


def fit_model(views: list[View], image_size: tuple[int, int]) -> Model:
    left_objects = [view.left.object_points.astype(np.float32) for view in views]
    left_images = [view.left.image_points.astype(np.float32) for view in views]
    right_objects = [view.right.object_points.astype(np.float32) for view in views]
    right_images = [view.right.image_points.astype(np.float32) for view in views]
    stereo_objects = [view.stereo_object.astype(np.float32) for view in views]
    stereo_left = [view.stereo_left.astype(np.float32) for view in views]
    stereo_right = [view.stereo_right.astype(np.float32) for view in views]

    left_rms, k1, d1, left_per_view = calibrate_mono(
        left_objects, left_images, image_size)
    right_rms, k2, d2, right_per_view = calibrate_mono(
        right_objects, right_images, image_size)

    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-7,
    )
    stereo_result = cv2.stereoCalibrate(
        stereo_objects,
        stereo_left,
        stereo_right,
        k1,
        d1,
        k2,
        d2,
        image_size,
        criteria=criteria,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    stereo_rms, k1, d1, k2, d2, rotation, translation, essential, fundamental = stereo_result

    rectify_result = cv2.stereoRectify(
        k1,
        d1,
        k2,
        d2,
        image_size,
        rotation,
        translation,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0.0,
    )
    r1, r2, p1, p2, q, roi1, roi2 = rectify_result
    return Model(
        mono_rms_left=left_rms,
        mono_rms_right=right_rms,
        mono_per_view_left=left_per_view,
        mono_per_view_right=right_per_view,
        stereo_rms=float(stereo_rms),
        k1=k1,
        d1=d1,
        k2=k2,
        d2=d2,
        rotation=rotation,
        translation=translation,
        essential=essential,
        fundamental=fundamental,
        rectify_left=r1,
        rectify_right=r2,
        projection_left=p1,
        projection_right=p2,
        disparity_to_depth=q,
        valid_roi_left=tuple(int(value) for value in roi1),
        valid_roi_right=tuple(int(value) for value in roi2),
    )


def robust_limit(values: np.ndarray, hard_limit: float) -> float:
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    statistical_limit = median + max(3.0 * 1.4826 * mad, 0.15)
    return min(hard_limit, statistical_limit)


def reject_mono_outliers(
    views: list[View],
    preliminary: Model,
    hard_limit: float,
    minimum_views: int,
) -> tuple[list[View], list[dict], dict]:
    left_limit = robust_limit(preliminary.mono_per_view_left, hard_limit)
    right_limit = robust_limit(preliminary.mono_per_view_right, hard_limit)
    keep: list[View] = []
    reject: list[dict] = []
    for view, left_error, right_error in zip(
        views, preliminary.mono_per_view_left, preliminary.mono_per_view_right
    ):
        if left_error <= left_limit and right_error <= right_limit:
            keep.append(view)
        else:
            reject.append({
                'index': view.index,
                'reason': 'mono_reprojection_outlier',
                'left_rms_px': float(left_error),
                'right_rms_px': float(right_error),
            })
    if len(keep) < minimum_views:
        raise RuntimeError(
            f'outlier rejection leaves only {len(keep)} views; need {minimum_views}. '
            'Collect a cleaner and more diverse dataset instead of relaxing the gate.')
    return keep, reject, {
        'left_limit_px': left_limit,
        'right_limit_px': right_limit,
        'hard_limit_px': hard_limit,
    }


def epipolar_vertical_errors(model: Model, views: list[View]) -> np.ndarray:
    errors = []
    for view in views:
        left_rectified = cv2.undistortPoints(
            view.stereo_left.reshape(-1, 1, 2),
            model.k1,
            model.d1,
            R=model.rectify_left,
            P=model.projection_left,
        ).reshape(-1, 2)
        right_rectified = cv2.undistortPoints(
            view.stereo_right.reshape(-1, 1, 2),
            model.k2,
            model.d2,
            R=model.rectify_right,
            P=model.projection_right,
        ).reshape(-1, 2)
        errors.extend(np.abs(left_rectified[:, 1] - right_rectified[:, 1]).tolist())
    return np.asarray(errors, dtype=np.float64)


def split_train_holdout(views: list[View]) -> tuple[list[View], list[View]]:
    # Deterministic spread over capture order. With 15 views this gives 12 train + 3 holdout.
    holdout = [view for position, view in enumerate(views) if position % 5 == 0]
    train = [view for position, view in enumerate(views) if position % 5 != 0]
    if len(train) < 12 or len(holdout) < 3:
        raise RuntimeError('need at least 15 accepted views for a 12/3 train/holdout split')
    return train, holdout


def ros_camera_info(
    name: str,
    image_size: tuple[int, int],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    rectification: np.ndarray,
    projection: np.ndarray,
) -> dict:
    return {
        'image_width': int(image_size[0]),
        'image_height': int(image_size[1]),
        'camera_name': name,
        'camera_matrix': camera_info_matrix(camera_matrix),
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': camera_info_matrix(
            np.asarray(distortion, dtype=np.float64).reshape(1, -1)),
        'rectification_matrix': camera_info_matrix(rectification),
        'projection_matrix': camera_info_matrix(projection),
    }


def write_yaml(path: Path, data: dict) -> None:
    path.write_text(
        yaml.safe_dump(data, sort_keys=False, default_flow_style=False),
        encoding='utf-8',
    )


def write_previews(
    output_dir: Path,
    views: list[View],
    model: Model,
    image_size: tuple[int, int],
    count: int = 6,
) -> None:
    preview_dir = output_dir / 'rectified_previews'
    preview_dir.mkdir(parents=True, exist_ok=True)
    map1x, map1y = cv2.initUndistortRectifyMap(
        model.k1, model.d1, model.rectify_left, model.projection_left,
        image_size, cv2.CV_16SC2)
    map2x, map2y = cv2.initUndistortRectifyMap(
        model.k2, model.d2, model.rectify_right, model.projection_right,
        image_size, cv2.CV_16SC2)
    positions = np.linspace(0, len(views) - 1, min(count, len(views)), dtype=int)
    for position in sorted(set(positions.tolist())):
        view = views[position]
        left = cv2.imread(str(view.left_path), cv2.IMREAD_COLOR)
        right = cv2.imread(str(view.right_path), cv2.IMREAD_COLOR)
        left_rect = cv2.remap(left, map1x, map1y, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right, map2x, map2y, cv2.INTER_LINEAR)
        canvas = np.hstack([left_rect, right_rect])
        for y in range(40, canvas.shape[0], 80):
            cv2.line(canvas, (0, y), (canvas.shape[1] - 1, y), (0, 255, 0), 1)
        cv2.imwrite(str(preview_dir / f'pair_{view.index:04d}.png'), canvas)


def gate(value: float | None, limit: float, mode: str = 'max') -> dict:
    if value is None:
        return {'status': 'NOT_CHECKED', 'value': None, 'limit': limit}
    if mode == 'max':
        passed = value <= limit
    elif mode == 'min':
        passed = value >= limit
    elif mode == 'equal':
        passed = value == limit
    else:
        raise ValueError(f'unknown gate mode {mode!r}')
    return {'status': 'PASS' if passed else 'FAIL', 'value': value, 'limit': limit}


def report_markdown(report: dict) -> str:
    metrics = report['metrics']
    lines = [
        '# Stereo calibration report',
        '',
        f"Overall: **{report['overall']}**",
        '',
        f"- Image size: {report['image_size']['width']} x {report['image_size']['height']}",
        f"- Accepted views: {report['views']['accepted']}",
        f"- Rejected views: {report['views']['rejected']}",
        f"- Baseline: {metrics['baseline_m']:.6f} m",
        f"- Left mono RMS: {metrics['mono_rms_left_px']:.4f} px",
        f"- Right mono RMS: {metrics['mono_rms_right_px']:.4f} px",
        f"- Stereo RMS: {metrics['stereo_rms_px']:.4f} px",
        f"- Final vertical epipolar p95: {metrics['epipolar_final_p95_px']:.4f} px",
        f"- Holdout vertical epipolar p95: {metrics['epipolar_holdout_p95_px']:.4f} px",
        '',
        '## Gates',
        '',
        '| Gate | Status | Value | Limit |',
        '|---|---:|---:|---:|',
    ]
    for name, item in report['gates'].items():
        value = item.get('value')
        limit = item.get('limit')
        value_text = 'n/a' if value is None else f'{value:.6g}'
        limit_text = 'n/a' if limit is None else str(limit)
        lines.append(f"| {name} | {item['status']} | {value_text} | {limit_text} |")
    lines.extend([
        '',
        '## Deployment rule',
        '',
        'Only deploy `left_camera_info.yaml` and `right_camera_info.yaml` when Overall is PASS ',
        'and the rectified preview images show corresponding points on the same horizontal lines.',
        '',
    ])
    return '\n'.join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--board',
        default=str(
            Path(get_package_share_directory('camera_calibration'))
            / 'config' / 'art_stereo_board.yaml'),
        help='board YAML (defaults to the installed ART 10x7, 70 mm checkerboard)')
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--expected-width', type=int, default=2064)
    parser.add_argument('--expected-height', type=int, default=1544)
    parser.add_argument('--expected-baseline-m', type=float)
    parser.add_argument('--baseline-tolerance-fraction', type=float, default=0.05)
    parser.add_argument('--max-view-rms-px', type=float, default=1.5)
    parser.add_argument('--max-stereo-rms-px', type=float, default=1.0)
    parser.add_argument('--max-final-epipolar-p95-px', type=float, default=1.0)
    parser.add_argument('--max-holdout-epipolar-p95-px', type=float, default=1.5)
    parser.add_argument('--max-capture-delta-p95-ms', type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.expected_baseline_m is not None and args.expected_baseline_m <= 0:
        raise SystemExit('--expected-baseline-m must be > 0')
    if args.expected_width <= 0 or args.expected_height <= 0:
        raise SystemExit('--expected-width and --expected-height must be > 0')
    if args.baseline_tolerance_fraction <= 0:
        raise SystemExit('--baseline-tolerance-fraction must be > 0')

    board = load_board(args.board)
    input_dir = Path(args.input).resolve()
    output_dir = Path(args.output).resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(f'output directory is not empty: {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)

    views, image_size, detection_rejects = load_views(input_dir, board)
    if len(views) < 15:
        raise SystemExit(
            f'only {len(views)} pairs passed corner detection; need at least 15')

    preliminary = fit_model(views, image_size)
    filtered, outlier_rejects, outlier_limits = reject_mono_outliers(
        views, preliminary, args.max_view_rms_px, minimum_views=15)
    rejected = detection_rejects + outlier_rejects

    train, holdout = split_train_holdout(filtered)
    train_model = fit_model(train, image_size)
    holdout_epipolar = epipolar_vertical_errors(train_model, holdout)

    final_model = fit_model(filtered, image_size)
    final_epipolar = epipolar_vertical_errors(final_model, filtered)
    baseline = float(np.linalg.norm(final_model.translation.reshape(-1)))
    timestamp_deltas = [
        view.timestamp_delta_ms for view in filtered if view.timestamp_delta_ms is not None]

    final_p95 = percentile(final_epipolar, 95.0)
    holdout_p95 = percentile(holdout_epipolar, 95.0)
    capture_p95 = percentile(timestamp_deltas, 95.0)

    gates = {
        'accepted_views': gate(float(len(filtered)), 15.0, mode='min'),
        'expected_image_width_px': gate(
            float(image_size[0]), float(args.expected_width), mode='equal'),
        'expected_image_height_px': gate(
            float(image_size[1]), float(args.expected_height), mode='equal'),
        'stereo_rms_px': gate(final_model.stereo_rms, args.max_stereo_rms_px),
        'final_epipolar_p95_px': gate(
            final_p95, args.max_final_epipolar_p95_px),
        'holdout_epipolar_p95_px': gate(
            holdout_p95, args.max_holdout_epipolar_p95_px),
        'capture_timestamp_p95_ms': gate(
            capture_p95, args.max_capture_delta_p95_ms),
        'baseline_nonzero_m': gate(baseline, 0.001, mode='min'),
        'right_projection_tx_abs_px': gate(
            abs(float(final_model.projection_right[0, 3])), 1e-6, mode='min'),
    }
    if args.expected_baseline_m is None:
        gates['mechanical_baseline_relative_error'] = {
            'status': 'NOT_CHECKED', 'value': None,
            'limit': args.baseline_tolerance_fraction,
        }
        expected_error = None
    else:
        expected_error = abs(baseline - args.expected_baseline_m) / args.expected_baseline_m
        gates['mechanical_baseline_relative_error'] = gate(
            expected_error, args.baseline_tolerance_fraction)

    required_gate_names = [
        'accepted_views',
        'expected_image_width_px',
        'expected_image_height_px',
        'stereo_rms_px',
        'final_epipolar_p95_px',
        'holdout_epipolar_p95_px',
        'capture_timestamp_p95_ms',
        'baseline_nonzero_m',
        'right_projection_tx_abs_px',
    ]
    if args.expected_baseline_m is not None:
        required_gate_names.append('mechanical_baseline_relative_error')
    overall = (
        'PASS'
        if all(gates[name]['status'] == 'PASS' for name in required_gate_names)
        else 'FAIL')

    left_info = ros_camera_info(
        'vimba_front_left_center', image_size,
        final_model.k1, final_model.d1,
        final_model.rectify_left, final_model.projection_left)
    right_info = ros_camera_info(
        'vimba_front_right_center', image_size,
        final_model.k2, final_model.d2,
        final_model.rectify_right, final_model.projection_right)
    write_yaml(output_dir / 'left_camera_info.yaml', left_info)
    write_yaml(output_dir / 'right_camera_info.yaml', right_info)

    calibration = {
        'schema_version': 1,
        'transform_convention': 'X_right = R_left_to_right * X_left + T_left_to_right',
        'board': board.as_dict(),
        'image_size': {'width': image_size[0], 'height': image_size[1]},
        'left': {
            'camera_name': 'vimba_front_left_center',
            'camera_matrix': matrix_list(final_model.k1),
            'distortion_coefficients': final_model.d1.reshape(-1).tolist(),
            'rectification_matrix': matrix_list(final_model.rectify_left),
            'projection_matrix': matrix_list(final_model.projection_left),
        },
        'right': {
            'camera_name': 'vimba_front_right_center',
            'camera_matrix': matrix_list(final_model.k2),
            'distortion_coefficients': final_model.d2.reshape(-1).tolist(),
            'rectification_matrix': matrix_list(final_model.rectify_right),
            'projection_matrix': matrix_list(final_model.projection_right),
        },
        'R_left_to_right': matrix_list(final_model.rotation),
        'T_left_to_right_m': final_model.translation.reshape(-1).tolist(),
        'essential_matrix': matrix_list(final_model.essential),
        'fundamental_matrix': matrix_list(final_model.fundamental),
        'Q_disparity_to_depth': matrix_list(final_model.disparity_to_depth),
        'valid_roi_left': list(final_model.valid_roi_left),
        'valid_roi_right': list(final_model.valid_roi_right),
    }
    write_yaml(output_dir / 'stereo_calibration.yaml', calibration)

    report = {
        'overall': overall,
        'board': board.as_dict(),
        'image_size': {'width': image_size[0], 'height': image_size[1]},
        'views': {
            'manifest': len(read_manifest(input_dir)),
            'accepted': len(filtered),
            'train': len(train),
            'holdout': len(holdout),
            'rejected': len(rejected),
            'accepted_indices': [view.index for view in filtered],
            'rejections': rejected,
            'outlier_limits': outlier_limits,
        },
        'metrics': {
            'mono_rms_left_px': final_model.mono_rms_left,
            'mono_rms_right_px': final_model.mono_rms_right,
            'stereo_rms_px': final_model.stereo_rms,
            'epipolar_final_median_px': percentile(final_epipolar, 50.0),
            'epipolar_final_p95_px': final_p95,
            'epipolar_final_max_px': percentile(final_epipolar, 100.0),
            'epipolar_holdout_median_px': percentile(holdout_epipolar, 50.0),
            'epipolar_holdout_p95_px': holdout_p95,
            'epipolar_holdout_max_px': percentile(holdout_epipolar, 100.0),
            'capture_timestamp_delta_p95_ms': capture_p95,
            'capture_timestamp_delta_max_ms': percentile(timestamp_deltas, 100.0),
            'baseline_m': baseline,
            'expected_baseline_m': args.expected_baseline_m,
            'baseline_relative_error': expected_error,
        },
        'gates': gates,
        'opencv_version': cv2.__version__,
    }
    (output_dir / 'report.json').write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    (output_dir / 'report.md').write_text(report_markdown(report), encoding='utf-8')
    write_previews(output_dir, filtered, final_model, image_size)

    print(report_markdown(report))
    print(f'Artifacts: {output_dir}')
    return 0 if overall == 'PASS' else 2


if __name__ == '__main__':
    sys.exit(main())
