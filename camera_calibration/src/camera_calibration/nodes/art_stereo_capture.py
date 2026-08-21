#!/usr/bin/env python3
"""Save synchronized stereo PNG pairs for online or offline filtering workflows."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from camera_calibration.art_stereo import BoardSpec, detect_board, Detection, load_board
import cv2
import numpy as np

try:
    import message_filters
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
except ImportError as exc:  # pragma: no cover
    raise SystemExit(f'ROS 2 Python dependency missing; source the workspace first: {exc}')


BAYER_CODES = {
    'bayer_rggb8': cv2.COLOR_BayerRG2BGR,
    'bayer_bggr8': cv2.COLOR_BayerBG2BGR,
    'bayer_gbrg8': cv2.COLOR_BayerGB2BGR,
    'bayer_grbg8': cv2.COLOR_BayerGR2BGR,
}

BAYER_GRAY_CODES = {
    'bayer_rggb8': cv2.COLOR_BayerRG2GRAY,
    'bayer_bggr8': cv2.COLOR_BayerBG2GRAY,
    'bayer_gbrg8': cv2.COLOR_BayerGB2GRAY,
    'bayer_grbg8': cv2.COLOR_BayerGR2GRAY,
}


def stamp_ns(msg: Image) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def image_to_bgr(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    height = int(msg.height)
    width = int(msg.width)
    if height <= 0 or width <= 0 or msg.step <= 0:
        raise ValueError('invalid ROS Image dimensions')

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    required = height * int(msg.step)
    if raw.size < required:
        raise ValueError(f'ROS Image buffer has {raw.size} bytes, expected at least {required}')
    rows = raw[:required].reshape(height, int(msg.step))

    if encoding == 'mono8':
        gray = rows[:, :width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if encoding in BAYER_CODES:
        mosaic = rows[:, :width]
        return cv2.cvtColor(mosaic, BAYER_CODES[encoding])
    if encoding in {'bgr8', 'rgb8'}:
        pixels = rows[:, :width * 3].reshape(height, width, 3)
        return pixels.copy() if encoding == 'bgr8' else cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    if encoding in {'bgra8', 'rgba8'}:
        pixels = rows[:, :width * 4].reshape(height, width, 4)
        code = cv2.COLOR_BGRA2BGR if encoding == 'bgra8' else cv2.COLOR_RGBA2BGR
        return cv2.cvtColor(pixels, code)
    raise ValueError(
        f'unsupported ROS Image encoding {msg.encoding!r}; expected mono8, Bayer8, rgb8 or bgr8')


def image_to_gray(msg: Image) -> np.ndarray:
    """Decode a ROS Image directly to uint8 grayscale without a 3-channel copy."""
    encoding = msg.encoding.lower()
    height = int(msg.height)
    width = int(msg.width)
    if height <= 0 or width <= 0 or msg.step <= 0:
        raise ValueError('invalid ROS Image dimensions')

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    required = height * int(msg.step)
    if raw.size < required:
        raise ValueError(f'ROS Image buffer has {raw.size} bytes, expected at least {required}')
    rows = raw[:required].reshape(height, int(msg.step))

    if encoding == 'mono8':
        return rows[:, :width].copy()
    if encoding in BAYER_GRAY_CODES:
        return cv2.cvtColor(rows[:, :width], BAYER_GRAY_CODES[encoding])
    if encoding in {'bgr8', 'rgb8'}:
        pixels = rows[:, :width * 3].reshape(height, width, 3)
        code = cv2.COLOR_BGR2GRAY if encoding == 'bgr8' else cv2.COLOR_RGB2GRAY
        return cv2.cvtColor(pixels, code)
    if encoding in {'bgra8', 'rgba8'}:
        pixels = rows[:, :width * 4].reshape(height, width, 4)
        code = cv2.COLOR_BGRA2GRAY if encoding == 'bgra8' else cv2.COLOR_RGBA2GRAY
        return cv2.cvtColor(pixels, code)
    raise ValueError(
        f'unsupported ROS Image encoding {msg.encoding!r}; expected mono8, Bayer8, rgb8 or bgr8')


def sharpness(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def pose_signature(detection: Detection, width: int, height: int) -> np.ndarray:
    points = detection.image_points.astype(np.float64)
    centered = points - points.mean(axis=0, keepdims=True)
    covariance = centered.T @ centered / max(len(points), 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major = eigenvectors[:, int(np.argmax(eigenvalues))]
    angle = math.atan2(float(major[1]), float(major[0]))
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    span = maximum - minimum
    center = points.mean(axis=0)
    return np.array([
        center[0] / width,
        center[1] / height,
        span[0] / width,
        span[1] / height,
        0.15 * math.cos(2.0 * angle),
        0.15 * math.sin(2.0 * angle),
    ])


class StereoCapture(Node):
    FIELDNAMES = [
        'index', 'left_file', 'right_file', 'left_stamp_ns', 'right_stamp_ns',
        'delta_ms', 'left_blur', 'right_blur', 'board_center_x', 'board_center_y',
        'board_width_fraction', 'board_height_fraction',
    ]

    def __init__(self, args: argparse.Namespace, board: BoardSpec):
        super().__init__('art_stereo_pair_capture')
        self.args = args
        self.board = board
        self.output = Path(args.output)
        if self.output.exists() and any(self.output.iterdir()):
            raise RuntimeError(
                f'{self.output} is not empty; choose a new output directory')
        self.left_dir = self.output / 'left'
        self.right_dir = self.output / 'right'
        self.left_dir.mkdir(parents=True, exist_ok=True)
        self.right_dir.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.output / 'pairs.csv'

        self.manifest_stream = self.manifest_path.open('w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.manifest_stream, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()
        self.manifest_stream.flush()

        self.saved = 0
        self.last_saved_stamp_ns: int | None = None
        self.signatures: list[np.ndarray] = []
        self.counters: Counter[str] = Counter()
        self.rows: list[dict] = []
        self.done = False

        self.left_sub = message_filters.Subscriber(
            self, Image, args.left_topic, qos_profile=qos_profile_sensor_data)
        self.right_sub = message_filters.Subscriber(
            self, Image, args.right_topic, qos_profile=qos_profile_sensor_data)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=20,
            slop=args.max_delta_ms / 1000.0,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self._callback)

        self.get_logger().info(
            f'capturing {args.left_topic} + {args.right_topic}; '
            f'mode={args.mode}, interval={args.min_interval_sec:.3f} s; '
            f'board={board.columns}x{board.rows} inner corners, '
            f'square={board.square_size_m:.6f} m')

    def close(self) -> None:
        if not self.manifest_stream.closed:
            self.manifest_stream.flush()
            self.manifest_stream.close()
        summary = {
            'board': self.board.as_dict(),
            'topics': {'left': self.args.left_topic, 'right': self.args.right_topic},
            'capture_mode': self.args.mode,
            'saved_pairs': self.saved,
            'reject_counts': dict(sorted(self.counters.items())),
            'max_delta_ms': self.args.max_delta_ms,
            'min_blur': self.args.min_blur,
        }
        if self.signatures:
            summary['capture_coverage'] = {
                'center_x_min': min(row['board_center_x'] for row in self.rows),
                'center_x_max': max(row['board_center_x'] for row in self.rows),
                'center_y_min': min(row['board_center_y'] for row in self.rows),
                'center_y_max': max(row['board_center_y'] for row in self.rows),
                'width_fraction_min': min(row['board_width_fraction'] for row in self.rows),
                'width_fraction_max': max(row['board_width_fraction'] for row in self.rows),
            }
        (self.output / 'capture_summary.json').write_text(
            json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')

    def _callback(self, left_msg: Image, right_msg: Image) -> None:
        if self.done:
            return
        self.counters['synchronized_callbacks'] += 1

        left_ns = stamp_ns(left_msg)
        right_ns = stamp_ns(right_msg)
        delta_ms = abs(left_ns - right_ns) / 1e6
        if delta_ms > self.args.max_delta_ms:
            self.counters['timestamp_delta'] += 1
            return

        reference_ns = min(left_ns, right_ns)
        if self.last_saved_stamp_ns is not None:
            elapsed = (reference_ns - self.last_saved_stamp_ns) / 1e9
            if elapsed < self.args.min_interval_sec:
                self.counters['minimum_interval'] += 1
                return

        try:
            left_gray = image_to_gray(left_msg)
            right_gray = image_to_gray(right_msg)
        except Exception as exc:
            self.counters['decode_error'] += 1
            self.get_logger().error(str(exc))
            return

        if left_gray.shape != right_gray.shape:
            self.counters['shape_mismatch'] += 1
            return

        signature = None
        left_blur: float | str = ''
        right_blur: float | str = ''
        if self.args.mode == 'online-filter':
            left_blur = sharpness(left_gray)
            right_blur = sharpness(right_gray)
            if min(left_blur, right_blur) < self.args.min_blur:
                self.counters['blur'] += 1
                return

            left_detection = detect_board(left_gray, self.board, fast=True)
            right_detection = detect_board(right_gray, self.board, fast=True)
            if left_detection is None or right_detection is None:
                self.counters['board_not_in_both'] += 1
                return

            height, width = left_gray.shape
            signature = pose_signature(left_detection, width, height)
            if self.args.min_novelty > 0 and self.signatures:
                novelty = min(float(np.linalg.norm(signature - old)) for old in self.signatures)
                if novelty < self.args.min_novelty:
                    self.counters['duplicate_pose'] += 1
                    return

        index = self.saved
        left_name = f'{index:04d}.png'
        right_name = f'{index:04d}.png'
        if not cv2.imwrite(str(self.left_dir / left_name), left_gray):
            raise RuntimeError(f'failed to write {self.left_dir / left_name}')
        if not cv2.imwrite(str(self.right_dir / right_name), right_gray):
            raise RuntimeError(f'failed to write {self.right_dir / right_name}')

        row = {
            'index': index,
            'left_file': f'left/{left_name}',
            'right_file': f'right/{right_name}',
            'left_stamp_ns': left_ns,
            'right_stamp_ns': right_ns,
            'delta_ms': round(delta_ms, 6),
            'left_blur': round(left_blur, 3) if left_blur != '' else '',
            'right_blur': round(right_blur, 3) if right_blur != '' else '',
            'board_center_x': round(float(signature[0]), 6) if signature is not None else '',
            'board_center_y': round(float(signature[1]), 6) if signature is not None else '',
            'board_width_fraction': round(float(signature[2]), 6) if signature is not None else '',
            'board_height_fraction': (
                round(float(signature[3]), 6) if signature is not None else ''),
        }
        self.writer.writerow(row)
        self.manifest_stream.flush()
        self.rows.append(row)
        if signature is not None:
            self.signatures.append(signature)
        self.last_saved_stamp_ns = reference_ns
        self.saved += 1
        if signature is None:
            self.get_logger().info(
                f'saved raw pair {index:04d}: dt={delta_ms:.3f} ms')
        else:
            self.get_logger().info(
                f'saved filtered pair {index:04d}: dt={delta_ms:.3f} ms, '
                f'blur=({left_blur:.0f},{right_blur:.0f}), '
                f'center=({signature[0]:.2f},{signature[1]:.2f})')
        if self.saved >= self.args.max_pairs:
            self.done = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--board',
        default=str(
            Path(get_package_share_directory('camera_calibration'))
            / 'config' / 'art_stereo_board.yaml'),
        help='board YAML (defaults to the installed ART 10x7, 70 mm checkerboard)')
    parser.add_argument('--output', required=True)
    parser.add_argument('--left-topic', default='/vimba_calib_left/image')
    parser.add_argument('--right-topic', default='/vimba_calib_right/image')
    parser.add_argument(
        '--mode', choices=('interval', 'online-filter'), default='interval',
        help='interval saves without board detection; online-filter runs the legacy live detector')
    parser.add_argument('--max-pairs', type=int, default=60)
    parser.add_argument('--max-delta-ms', type=float, default=2.0)
    parser.add_argument('--min-interval-sec', type=float, default=1.0)
    parser.add_argument('--min-blur', type=float, default=30.0)
    parser.add_argument(
        '--min-novelty', type=float, default=0.025,
        help='minimum normalized pose-signature distance; set 0 to disable duplicate rejection')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.max_pairs < 1:
        raise SystemExit('--max-pairs must be at least 1')
    if args.max_delta_ms <= 0 or args.min_interval_sec < 0 or args.min_blur < 0:
        raise SystemExit('delta/interval/blur arguments must be non-negative (delta > 0)')
    board = load_board(args.board)

    rclpy.init()
    node: StereoCapture | None = None
    try:
        node = StereoCapture(args, board)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            saved = node.saved
            node.destroy_node()
        else:
            saved = 0
        rclpy.shutdown()
    print(f'capture complete: {saved} pairs')
    return 0 if saved >= 15 else 2


if __name__ == '__main__':
    sys.exit(main())
