#!/usr/bin/env python3
"""Measure stereo CameraInfo rate, timestamp pairing and calibration baseline."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
except ImportError as exc:  # pragma: no cover - only available in a sourced ROS environment
    raise SystemExit(f'ROS 2 Python dependency missing; source the workspace first: {exc}')


def stamp_seconds(msg: CameraInfo) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def percentile(values: list[float], percentile_value: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile_value / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def pair_deltas(left: list[float], right: list[float]) -> list[float]:
    """Greedily form one-to-one chronological pairs and return absolute deltas."""
    left = sorted(left)
    right = sorted(right)
    deltas: list[float] = []
    i = 0
    j = 0
    while i < len(left) and j < len(right):
        current = abs(left[i] - right[j])
        left_next = abs(left[i + 1] - right[j]) if i + 1 < len(left) else math.inf
        right_next = abs(left[i] - right[j + 1]) if j + 1 < len(right) else math.inf
        if left_next < current and left_next <= right_next:
            i += 1
        elif right_next < current:
            j += 1
        else:
            deltas.append(current)
            i += 1
            j += 1
    return deltas


def message_rate(stamps: list[float]) -> float | None:
    if len(stamps) < 2:
        return None
    span = max(stamps) - min(stamps)
    return (len(stamps) - 1) / span if span > 0 else None


def matrix_is_identity(values: list[float], tolerance: float = 1e-9) -> bool:
    identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    return len(values) == 9 and all(abs(a - b) <= tolerance for a, b in zip(values, identity))


def projection_baseline(msg: CameraInfo | None) -> float | None:
    if msg is None or len(msg.p) != 12:
        return None
    candidates = []
    if abs(msg.p[0]) > 1e-12:
        candidates.append(abs(msg.p[3] / msg.p[0]))
    if abs(msg.p[5]) > 1e-12:
        candidates.append(abs(msg.p[7] / msg.p[5]))
    return max(candidates) if candidates else None


class StatusNode(Node):
    def __init__(self, left_topic: str, right_topic: str):
        super().__init__('art_stereo_status')
        self.left_stamps: list[float] = []
        self.right_stamps: list[float] = []
        self.left_last: CameraInfo | None = None
        self.right_last: CameraInfo | None = None
        self.create_subscription(
            CameraInfo, left_topic, self._left_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, right_topic, self._right_callback, qos_profile_sensor_data)

    def _left_callback(self, msg: CameraInfo) -> None:
        self.left_stamps.append(stamp_seconds(msg))
        self.left_last = msg

    def _right_callback(self, msg: CameraInfo) -> None:
        self.right_stamps.append(stamp_seconds(msg))
        self.right_last = msg


def message_summary(msg: CameraInfo | None) -> dict:
    if msg is None:
        return {'received': False}
    return {
        'received': True,
        'frame_id': msg.header.frame_id,
        'width': int(msg.width),
        'height': int(msg.height),
        'distortion_model': msg.distortion_model,
        'k': list(msg.k),
        'd': list(msg.d),
        'r': list(msg.r),
        'p': list(msg.p),
        'rectification_is_identity': matrix_is_identity(list(msg.r)),
        'projection_baseline_m': projection_baseline(msg),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--left-topic', default='/vimba_calib_left/camera_info')
    parser.add_argument('--right-topic', default='/vimba_calib_right/camera_info')
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--expected-width', type=int, default=2064)
    parser.add_argument('--expected-height', type=int, default=1544)
    parser.add_argument('--expected-rate-hz', type=float, default=10.0)
    parser.add_argument('--rate-tolerance-hz', type=float, default=2.0)
    parser.add_argument('--max-p95-delta-ms', type=float, default=2.0)
    parser.add_argument('--json-out')
    parser.add_argument('--require-baseline', action='store_true')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0:
        raise SystemExit('--duration must be > 0')
    if args.expected_width <= 0 or args.expected_height <= 0:
        raise SystemExit('--expected-width and --expected-height must be > 0')
    if args.expected_rate_hz <= 0 or args.rate_tolerance_hz < 0:
        raise SystemExit('--expected-rate-hz must be > 0 and tolerance must be >= 0')

    rclpy.init()
    node = StatusNode(args.left_topic, args.right_topic)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    deltas_ms = [value * 1000.0 for value in pair_deltas(
        node.left_stamps, node.right_stamps)]
    p95_delta = percentile(deltas_ms, 95.0)
    baseline = projection_baseline(node.right_last)
    left_rate = message_rate(node.left_stamps)
    right_rate = message_rate(node.right_stamps)
    expected_size = (args.expected_width, args.expected_height)
    left_size = (
        (int(node.left_last.width), int(node.left_last.height))
        if node.left_last is not None else None)
    right_size = (
        (int(node.right_last.width), int(node.right_last.height))
        if node.right_last is not None else None)

    def rate_is_valid(rate: float | None) -> bool:
        return bool(
            rate is not None
            and abs(rate - args.expected_rate_hz) <= args.rate_tolerance_hz)

    gates = {
        'both_streaming': bool(node.left_stamps and node.right_stamps),
        'matching_image_size': bool(left_size is not None and left_size == right_size),
        'expected_image_size': bool(
            left_size == expected_size and right_size == expected_size),
        'rates_within_tolerance': bool(
            rate_is_valid(left_rate) and rate_is_valid(right_rate)),
        'timestamp_p95_within_limit': bool(
            p95_delta is not None and p95_delta <= args.max_p95_delta_ms),
        'baseline_nonzero': bool(baseline is not None and baseline > 1e-6),
    }
    required = [
        gates['both_streaming'],
        gates['matching_image_size'],
        gates['expected_image_size'],
        gates['rates_within_tolerance'],
        gates['timestamp_p95_within_limit'],
    ]
    if args.require_baseline:
        required.append(gates['baseline_nonzero'])

    report = {
        'duration_sec': args.duration,
        'topics': {'left': args.left_topic, 'right': args.right_topic},
        'left': {
            **message_summary(node.left_last),
            'messages': len(node.left_stamps),
            'rate_hz': left_rate,
        },
        'right': {
            **message_summary(node.right_last),
            'messages': len(node.right_stamps),
            'rate_hz': right_rate,
        },
        'expected': {
            'width': args.expected_width,
            'height': args.expected_height,
            'rate_hz': args.expected_rate_hz,
            'rate_tolerance_hz': args.rate_tolerance_hz,
        },
        'timestamp_pairing': {
            'pairs': len(deltas_ms),
            'delta_ms_median': statistics.median(deltas_ms) if deltas_ms else None,
            'delta_ms_p95': p95_delta,
            'delta_ms_max': max(deltas_ms) if deltas_ms else None,
            'limit_ms': args.max_p95_delta_ms,
        },
        'stereo_baseline_m': baseline,
        'gates': gates,
        'overall': 'PASS' if all(required) else 'FAIL',
    }

    output = json.dumps(report, indent=2, sort_keys=True)
    print(output)
    if args.json_out:
        path = Path(args.json_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(output + '\n', encoding='utf-8')
    return 0 if all(required) else 2


if __name__ == '__main__':
    sys.exit(main())
