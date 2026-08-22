#!/usr/bin/env python3
"""Local web control room for the ART six-camera calibration SOP."""

from __future__ import annotations

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import threading
import time
from typing import Callable
from urllib.parse import parse_qs, urlparse
import webbrowser

from ament_index_python.packages import get_package_share_directory
from camera_calibration.art_stereo import BoardSpec, detect_board
import cv2
import numpy as np
import yaml


EXPECTED_WIDTH = 2064
EXPECTED_HEIGHT = 1544
EXPECTED_RATE_HZ = 10.0
MIN_MONO_PREVIEW_RATE_HZ = 0.5
DETECTION_INTERVAL_SEC = 0.80
STREAM_ONLINE_TIMEOUT_SEC = 4.0
SESSION_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')


@dataclass(frozen=True)
class TaskSpec:
    task_id: str
    label: str
    short_label: str
    kind: str
    model: str
    camera_names: tuple[str, ...]
    stream_keys: tuple[str, ...]
    target_captures: int
    description: str


TASKS = (
    TaskSpec(
        'stereo_center', 'Center stereo pair', 'Stereo', 'stereo', 'pinhole',
        ('vimba_front_left_center', 'vimba_front_right_center'),
        ('stereo_left', 'stereo_right'), 60,
        'Joint pinhole intrinsics, stereo transform, and rectification.'),
    TaskSpec(
        'vimba_front', 'Front fisheye', 'Front', 'mono', 'fisheye',
        ('vimba_front',), ('vimba_front',), 30,
        'Monocular equidistant calibration for the front surround camera.'),
    TaskSpec(
        'vimba_left', 'Left fisheye', 'Left', 'mono', 'fisheye',
        ('vimba_left',), ('vimba_left',), 30,
        'Monocular equidistant calibration for the left surround camera.'),
    TaskSpec(
        'vimba_right', 'Right fisheye', 'Right', 'mono', 'fisheye',
        ('vimba_right',), ('vimba_right',), 30,
        'Monocular equidistant calibration for the right surround camera.'),
    TaskSpec(
        'vimba_rear', 'Rear fisheye', 'Rear', 'mono', 'fisheye',
        ('vimba_rear',), ('vimba_rear',), 30,
        'Monocular equidistant calibration for the rear surround camera.'),
)

TASK_BY_ID = {task.task_id: task for task in TASKS}

STREAM_TOPICS = {
    'stereo_left': '/vimba_calib_left/image',
    'stereo_right': '/vimba_calib_right/image',
    'vimba_front': '/vimba_front/image',
    'vimba_left': '/vimba_left/image',
    'vimba_right': '/vimba_right/image',
    'vimba_rear': '/vimba_rear/image',
}

PREVIEW_TOPICS = {
    key: f'{topic.rsplit("/", 1)[0]}/calibration_preview/compressed'
    for key, topic in STREAM_TOPICS.items()
}

RELAY_CAMERA_BY_TASK = {
    'stereo_center': 'stereo',
    'vimba_front': 'front',
    'vimba_left': 'left',
    'vimba_right': 'right',
    'vimba_rear': 'rear',
}

POSES = (
    {'id': 'center-medium', 'title': '中心 · 中距离', 'instruction':
     '保持整块板完整可见，并让板面大致平行于成像平面。',
     'x': 50, 'y': 50, 'scale': 0.72, 'rotate': 0},
    {'id': 'center-near', 'title': '中心 · 近距离', 'instruction':
     '靠近相机，但所有外沿方格仍须完整出现在所需画面中。',
     'x': 50, 'y': 50, 'scale': 1.02, 'rotate': 0},
    {'id': 'center-far', 'title': '中心 · 远距离', 'instruction':
     '远离相机，同时确保角点仍然清晰、可识别。',
     'x': 50, 'y': 50, 'scale': 0.48, 'rotate': 0},
    {'id': 'top-left', 'title': '覆盖左上角', 'instruction':
     '将整块板移到图像左上区域，不要切掉板边。',
     'x': 24, 'y': 25, 'scale': 0.62, 'rotate': -4},
    {'id': 'top-right', 'title': '覆盖右上角', 'instruction':
     '将整块板移到图像右上区域，不要切掉板边。',
     'x': 76, 'y': 25, 'scale': 0.62, 'rotate': 4},
    {'id': 'bottom-left', 'title': '覆盖左下角', 'instruction':
     '将整块板移到图像左下区域，不要切掉板边。',
     'x': 24, 'y': 75, 'scale': 0.62, 'rotate': 4},
    {'id': 'bottom-right', 'title': '覆盖右下角', 'instruction':
     '将整块板移到图像右下区域，不要切掉板边。',
     'x': 76, 'y': 75, 'scale': 0.62, 'rotate': -4},
    {'id': 'top-edge', 'title': '覆盖上边缘', 'instruction':
     '让标定板填充图像上方中央区域，保持整板可见。',
     'x': 50, 'y': 22, 'scale': 0.66, 'rotate': 0},
    {'id': 'bottom-edge', 'title': '覆盖下边缘', 'instruction':
     '让标定板填充图像下方中央区域，保持整板可见。',
     'x': 50, 'y': 78, 'scale': 0.66, 'rotate': 0},
    {'id': 'left-edge', 'title': '覆盖左边缘', 'instruction':
     '让标定板填充图像左侧中央区域，保持整板可见。',
     'x': 20, 'y': 50, 'scale': 0.62, 'rotate': -2},
    {'id': 'right-edge', 'title': '覆盖右边缘', 'instruction':
     '让标定板填充图像右侧中央区域，保持整板可见。',
     'x': 80, 'y': 50, 'scale': 0.62, 'rotate': 2},
    {'id': 'yaw-left', 'title': '偏航 · 向左转', 'instruction':
     '让板绕竖直轴向左转，保持所有角点可见。',
     'x': 43, 'y': 50, 'scale': 0.72, 'rotate': -7},
    {'id': 'yaw-right', 'title': '偏航 · 向右转', 'instruction':
     '让板绕竖直轴向右转，保持所有角点可见。',
     'x': 57, 'y': 50, 'scale': 0.72, 'rotate': 7},
    {'id': 'pitch-up', 'title': '俯仰 · 上沿后倾', 'instruction':
     '让标定板上沿远离相机，形成明显但不过大的俯仰角。',
     'x': 50, 'y': 44, 'scale': 0.68, 'rotate': 0},
    {'id': 'pitch-down', 'title': '俯仰 · 下沿后倾', 'instruction':
     '让标定板下沿远离相机，形成相反方向的俯仰角。',
     'x': 50, 'y': 56, 'scale': 0.68, 'rotate': 0},
    {'id': 'roll-left', 'title': '滚转 · 逆时针', 'instruction':
     '逆时针旋转标定板，同时保持完整边界可见。',
     'x': 50, 'y': 50, 'scale': 0.68, 'rotate': -22},
    {'id': 'roll-right', 'title': '滚转 · 顺时针', 'instruction':
     '顺时针旋转标定板，同时保持完整边界可见。',
     'x': 50, 'y': 50, 'scale': 0.68, 'rotate': 22},
    {'id': 'combo-left-near', 'title': '复合姿态 · 左侧近距', 'instruction':
     '同时加入左侧偏移、近距离、偏航和轻微滚转。',
     'x': 31, 'y': 46, 'scale': 0.88, 'rotate': -13},
    {'id': 'combo-right-near', 'title': '复合姿态 · 右侧近距', 'instruction':
     '在右侧做与上一步相反方向的复合姿态。',
     'x': 69, 'y': 54, 'scale': 0.88, 'rotate': 13},
    {'id': 'combo-high-far', 'title': '复合姿态 · 上方远距', 'instruction':
     '将较小的板放在画面上方，并加入偏航和俯仰。',
     'x': 54, 'y': 29, 'scale': 0.52, 'rotate': 9},
    {'id': 'combo-low-far', 'title': '复合姿态 · 下方远距', 'instruction':
     '将较小的板放在画面下方，并使用相反的偏航和俯仰。',
     'x': 46, 'y': 71, 'scale': 0.52, 'rotate': -9},
    {'id': 'free-1', 'title': '补充姿态 1', 'instruction':
     '选择一个与前面明显不同且足够清晰的姿态。',
     'x': 36, 'y': 34, 'scale': 0.61, 'rotate': 16},
    {'id': 'free-2', 'title': '补充姿态 2', 'instruction':
     '同时改变距离、画面区域和标定板朝向。',
     'x': 64, 'y': 66, 'scale': 0.74, 'rotate': -16},
    {'id': 'final-center', 'title': '最终中心检查', 'instruction':
     '回到清晰的中心姿态，确认对焦后再开始求解。',
     'x': 50, 'y': 50, 'scale': 0.7, 'rotate': 0},
)


def validate_session_name(value: str) -> str:
    value = value.strip()
    if not SESSION_PATTERN.fullmatch(value):
        raise ValueError(
            'session must be 1-64 characters using letters, digits, dot, dash, or underscore')
    return value


def task_for(task_id: str) -> TaskSpec:
    try:
        return TASK_BY_ID[task_id]
    except KeyError as exc:
        raise ValueError(f'unknown calibration task: {task_id}') from exc


def task_paths(data_root: Path, session: str, task: TaskSpec) -> dict[str, Path]:
    base = data_root / validate_session_name(session) / task.task_id
    return {
        'base': base,
        'capture': base / 'capture',
        'result': base / 'result',
        'preflight': base / 'preflight.json',
        'mono_archive': base / 'calibrationdata.tar.gz',
    }


def build_action_command(
    action: str,
    task: TaskSpec,
    paths: dict[str, Path],
    expected_baseline_m: float | None = None,
) -> list[str]:
    if action == 'preflight' and task.kind == 'stereo':
        return [
            'ros2', 'run', 'camera_calibration', 'art_stereo_status',
            '--duration', '10', '--json-out', str(paths['preflight']),
        ]
    if action == 'capture' and task.kind == 'stereo':
        return [
            'ros2', 'run', 'camera_calibration', 'art_stereo_capture',
            '--output', str(paths['capture']), '--mode', 'online-filter',
            '--max-pairs', str(task.target_captures), '--max-delta-ms', '2.0',
            '--min-interval-sec', '1.0', '--min-novelty', '0.025',
        ]
    if action == 'calibrate' and task.kind == 'stereo':
        command = [
            'ros2', 'run', 'camera_calibration', 'art_stereo_calibrate',
            '--input', str(paths['capture']), '--output', str(paths['result']),
            '--expected-width', str(EXPECTED_WIDTH),
            '--expected-height', str(EXPECTED_HEIGHT),
        ]
        if expected_baseline_m is not None:
            command.extend(['--expected-baseline-m', f'{expected_baseline_m:.9g}'])
        return command
    if action == 'calibrate' and task.kind == 'mono':
        camera_name = task.camera_names[0]
        return [
            'ros2', 'run', 'camera_calibration', 'art_camera_calibrator',
            camera_name, '--image-topic', STREAM_TOPICS[task.stream_keys[0]],
        ]
    raise ValueError(f'action {action!r} is not valid for {task.task_id}')


def command_text(command: list[str]) -> str:
    return shlex.join(command)


class ProcessManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._label: str | None = None
        self._command: list[str] = []
        self._logs: deque[str] = deque(maxlen=240)
        self._return_code: int | None = None

    def start(
        self,
        label: str,
        command: list[str],
        on_exit: Callable[[int, float], None] | None = None,
    ) -> None:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                raise RuntimeError(f'{self._label} is already running')
            started_at = time.time()
            self._label = label
            self._command = list(command)
            self._logs.clear()
            self._logs.append(f'$ {command_text(command)}')
            self._return_code = None
            self._process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            process = self._process

        def consume() -> None:
            assert process.stdout is not None
            for line in process.stdout:
                with self._lock:
                    self._logs.append(line.rstrip())
            return_code = process.wait()
            with self._lock:
                self._return_code = return_code
                self._logs.append(f'process exited with code {return_code}')
            if on_exit is not None:
                try:
                    on_exit(return_code, started_at)
                except Exception as exc:  # pragma: no cover - defensive logging
                    with self._lock:
                        self._logs.append(f'post-process error: {exc}')

        threading.Thread(target=consume, name='calibration-process-log', daemon=True).start()

    def stop(self) -> bool:
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                return False
            os.killpg(process.pid, signal.SIGINT)
            self._logs.append('sent SIGINT; waiting for clean shutdown')
            return True

    def snapshot(self) -> dict:
        with self._lock:
            running = self._process is not None and self._process.poll() is None
            return {
                'running': running,
                'label': self._label,
                'command': command_text(self._command) if self._command else None,
                'return_code': self._return_code,
                'logs': list(self._logs),
            }

    def close(self) -> None:
        self.stop()


class RelayManager:
    """Keep exactly one vehicle-to-local camera relay active for the UI task."""

    def __init__(self, foxglove_url: str | None):
        self.url = foxglove_url
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._camera: str | None = None
        self._publish_raw = False
        self._command: list[str] = []
        self._logs: deque[str] = deque(maxlen=80)
        self._return_code: int | None = None

    def _stop_locked(self) -> None:
        process = self._process
        if process is None or process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        self._logs.append(f'stopping {self._camera} relay')
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2.0)

    def activate(self, task: TaskSpec, publish_raw: bool = False) -> None:
        if self.url is None:
            return
        camera = RELAY_CAMERA_BY_TASK[task.task_id]
        with self._lock:
            if (
                self._camera == camera
                and self._publish_raw == publish_raw
                and self._process is not None
                and self._process.poll() is None
            ):
                return
            self._stop_locked()
            self._camera = camera
            self._publish_raw = publish_raw
            self._return_code = None
            self._command = [
                'ros2', 'run', 'camera_calibration', 'art_foxglove_relay',
                '--url', self.url, '--camera', camera,
            ]
            if publish_raw:
                self._command.append('--publish-raw')
            self._logs.append(f'$ {command_text(self._command)}')
            try:
                self._process = subprocess.Popen(
                    self._command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
            except OSError as exc:
                self._process = None
                self._return_code = 127
                self._logs.append(f'failed to start relay: {exc}')
                return
            process = self._process

        def consume() -> None:
            assert process.stdout is not None
            for line in process.stdout:
                with self._lock:
                    self._logs.append(line.rstrip())
            return_code = process.wait()
            with self._lock:
                if self._process is process:
                    self._return_code = return_code
                self._logs.append(f'{camera} relay exited with code {return_code}')

        threading.Thread(
            target=consume, name=f'calibration-relay-{camera}', daemon=True).start()

    def snapshot(self) -> dict:
        with self._lock:
            running = self._process is not None and self._process.poll() is None
            return {
                'enabled': self.url is not None,
                'running': running,
                'camera': self._camera,
                'publish_raw': self._publish_raw,
                'url': self.url,
                'return_code': self._return_code,
                'logs': list(self._logs),
            }

    def close(self) -> None:
        with self._lock:
            self._stop_locked()


class StreamMonitor:
    def __init__(self, demo: bool = False):
        self.demo = demo
        self._lock = threading.Lock()
        self._records = {
            key: {
                'topic': topic,
                'stamps': deque(maxlen=120),
                'last_monotonic': None,
                'last_detection_monotonic': 0.0,
                'width': None,
                'height': None,
                'encoding': None,
                'frame_id': None,
                'board_detected': False,
                'sharpness': None,
                'jpeg': None,
            }
            for key, topic in STREAM_TOPICS.items()
        }
        self._thread: threading.Thread | None = None
        self._node = None
        self._rclpy = None
        self._compressed_type = None
        self._sensor_qos = None
        self._subscriptions = {}
        self._subscription_lock = threading.Lock()
        self._detection_pending: set[str] = set()
        self._detection_executor = ThreadPoolExecutor(
            max_workers=2, thread_name_prefix='calibration-board')
        self._board = BoardSpec('checkerboard', 10, 7, 0.0700)
        if demo:
            self._seed_demo()
        else:
            self._start_ros()

    def _seed_demo(self) -> None:
        stamp = time.time()
        for index, key in enumerate(self._records):
            record = self._records[key]
            record['stamps'].extend(stamp - value * 0.1 for value in range(20, -1, -1))
            record['last_monotonic'] = time.monotonic()
            record['width'] = EXPECTED_WIDTH
            record['height'] = EXPECTED_HEIGHT
            record['encoding'] = 'bayer_rggb8'
            record['frame_id'] = key
            record['board_detected'] = True
            record['sharpness'] = 142.0 + index * 7.0
            record['jpeg'] = self._demo_jpeg(key, index)

    @staticmethod
    def _demo_jpeg(label: str, index: int) -> bytes:
        image = np.full((620, 900, 3), (26, 31, 37), dtype=np.uint8)
        for row in range(8):
            for column in range(11):
                color = 225 if (row + column) % 2 == 0 else 35
                x0 = 180 + column * 48
                y0 = 115 + row * 48
                cv2.rectangle(image, (x0, y0), (x0 + 48, y0 + 48),
                              (color, color, color), -1)
        cv2.putText(image, label.replace('_', ' ').upper(), (28, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (111, 227, 185), 2)
        cv2.putText(image, f'DEMO STREAM {index + 1}', (28, 588),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (163, 174, 184), 1)
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 82])
        return encoded.tobytes() if ok else b''

    def _start_ros(self) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        rclpy.init(args=None)
        self._rclpy = rclpy
        node = Node('art_calibration_ui_monitor')
        self._node = node
        self._compressed_type = CompressedImage
        self._sensor_qos = qos_profile_sensor_data

        def spin() -> None:
            from rclpy.executors import ExternalShutdownException

            try:
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.1)
            except ExternalShutdownException:
                pass
            except Exception:
                if rclpy.ok():
                    raise

        self._thread = threading.Thread(target=spin, name='calibration-ros', daemon=True)
        self._thread.start()

    def activate(self, keys: tuple[str, ...]) -> None:
        """Subscribe only to the selected job to avoid six 2K DDS streams."""
        if self.demo:
            return
        requested = set(keys)
        with self._subscription_lock:
            for key in set(self._subscriptions) - requested:
                self._node.destroy_subscription(self._subscriptions.pop(key))
            for key in requested - set(self._subscriptions):
                with self._lock:
                    record = self._records[key]
                    record['stamps'].clear()
                    record['last_monotonic'] = None
                    record['last_detection_monotonic'] = 0.0
                    record['board_detected'] = False
                self._subscriptions[key] = self._node.create_subscription(
                    self._compressed_type,
                    PREVIEW_TOPICS[key],
                    self._callback_for(key),
                    self._sensor_qos,
                )

    def _callback_for(self, key: str):
        def callback(msg) -> None:
            now = time.monotonic()
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            with self._lock:
                record = self._records[key]
                record['stamps'].append(stamp)
                record['last_monotonic'] = now
                record['frame_id'] = msg.header.frame_id
                dimensions = re.search(r'(\d+)x(\d+)', msg.format)
                if dimensions is not None:
                    record['width'] = int(dimensions.group(1))
                    record['height'] = int(dimensions.group(2))
                record['encoding'] = msg.format
                record['jpeg'] = bytes(msg.data)
                detection_due = (
                    now - record['last_detection_monotonic']
                    >= DETECTION_INTERVAL_SEC
                    and key not in self._detection_pending)
                if detection_due:
                    record['last_detection_monotonic'] = now
                    self._detection_pending.add(key)

            if detection_due:
                self._detection_executor.submit(self._update_detection, key, msg)

        return callback

    @staticmethod
    def _preview_gray(msg):
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        gray = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise ValueError('compressed preview is not a readable JPEG')
        preview_width = min(960, gray.shape[1])
        scale = preview_width / gray.shape[1]
        return cv2.resize(
            gray,
            (preview_width, int(gray.shape[0] * scale)),
            interpolation=cv2.INTER_AREA,
        )

    def _update_detection(self, key: str, msg) -> None:
        try:
            from camera_calibration.nodes.art_stereo_capture import sharpness

            preview = self._preview_gray(msg)
            detection = detect_board(preview, self._board, fast=True)
            with self._lock:
                self._records[key]['board_detected'] = detection is not None
                self._records[key]['sharpness'] = sharpness(preview)
        finally:
            with self._lock:
                self._detection_pending.discard(key)

    @staticmethod
    def _rate(stamps: deque) -> float | None:
        if len(stamps) < 2:
            return None
        span = max(stamps) - min(stamps)
        return (len(stamps) - 1) / span if span > 0 else None

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            output = {}
            for key, record in self._records.items():
                age = (
                    0.0 if self.demo else now - record['last_monotonic']
                    if record['last_monotonic'] is not None else None)
                output[key] = {
                    'topic': record['topic'],
                    'online': bool(
                        age is not None and age < STREAM_ONLINE_TIMEOUT_SEC),
                    'age_sec': age,
                    'rate_hz': self._rate(record['stamps']),
                    'width': record['width'],
                    'height': record['height'],
                    'encoding': record['encoding'],
                    'frame_id': record['frame_id'],
                    'board_detected': record['board_detected'],
                    'sharpness': record['sharpness'],
                }

            left = self._records['stereo_left']['stamps']
            right = self._records['stereo_right']['stamps']
            delta_ms = (
                min(
                    min(abs(left[-1] - stamp) for stamp in list(right)[-3:]),
                    min(abs(right[-1] - stamp) for stamp in list(left)[-3:]),
                ) * 1000.0
                if left and right else None)
            output['stereo_sync_delta_ms'] = delta_ms
            return output

    def frame(self, key: str) -> bytes | None:
        with self._lock:
            record = self._records.get(key)
            return record['jpeg'] if record is not None else None

    def close(self) -> None:
        if self.demo or self._rclpy is None:
            return
        if self._node is not None:
            self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
        self._detection_executor.shutdown(wait=False, cancel_futures=True)


def stream_gate(task: TaskSpec, streams: dict) -> tuple[bool, list[str]]:
    failures = []
    for key in task.stream_keys:
        stream = streams[key]
        if not stream['online']:
            failures.append(f'{stream["topic"]} is offline')
            continue
        if (stream['width'], stream['height']) != (EXPECTED_WIDTH, EXPECTED_HEIGHT):
            failures.append(
                f'{stream["topic"]} is {stream["width"]}x{stream["height"]}, '
                f'expected {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}')
        rate = stream['rate_hz']
        if task.kind == 'stereo':
            if rate is None or abs(rate - EXPECTED_RATE_HZ) > 2.0:
                failures.append(f'{stream["topic"]} rate is outside 10 +/- 2 Hz')
        elif rate is None or rate < MIN_MONO_PREVIEW_RATE_HZ:
            failures.append(
                f'{stream["topic"]} preview rate is below '
                f'{MIN_MONO_PREVIEW_RATE_HZ:.1f} Hz')
    if task.kind == 'stereo':
        delta = streams.get('stereo_sync_delta_ms')
        if delta is None or delta > 2.0:
            failures.append('stereo timestamp delta is above 2 ms or unavailable')
    return not failures, failures


def mono_archive_summary(path: Path) -> dict | None:
    if not path.is_file():
        return None
    try:
        with tarfile.open(path, 'r:gz') as archive:
            member = archive.getmember('ost.yaml')
            stream = archive.extractfile(member)
            if stream is None:
                return {'status': 'INVALID', 'reason': 'ost.yaml is unreadable'}
            data = yaml.safe_load(stream.read().decode('utf-8')) or {}
        model = data.get('distortion_model')
        size_ok = (
            data.get('image_width') == EXPECTED_WIDTH
            and data.get('image_height') == EXPECTED_HEIGHT)
        return {
            'status': 'PASS' if model == 'equidistant' and size_ok else 'FAIL',
            'distortion_model': model,
            'image_width': data.get('image_width'),
            'image_height': data.get('image_height'),
            'camera_name': data.get('camera_name'),
            'path': str(path),
        }
    except Exception as exc:
        return {'status': 'INVALID', 'reason': str(exc), 'path': str(path)}


class CalibrationApp:
    def __init__(
        self,
        assets_dir: Path,
        data_root: Path,
        demo: bool,
        foxglove_url: str | None = None,
    ):
        self.assets_dir = assets_dir
        self.data_root = data_root.expanduser().resolve()
        self.demo = demo
        self.monitor = StreamMonitor(demo=demo)
        self.process = ProcessManager()
        self.relay = RelayManager(None if demo else foxglove_url)

    def _relay_needs_raw(self, task: TaskSpec, process: dict | None = None) -> bool:
        process = process or self.process.snapshot()
        if not process['running'] or not process['label']:
            return False
        task_id, _, action = process['label'].partition(':')
        if task_id != task.task_id:
            return False
        return (
            task.kind == 'mono' and action == 'calibrate'
            or task.kind == 'stereo' and action in {'preflight', 'capture'}
        )

    def session_status(self, task: TaskSpec, session: str) -> dict:
        paths = task_paths(self.data_root, session, task)
        paths['base'].mkdir(parents=True, exist_ok=True)
        status = {
            'base': str(paths['base']),
            'capture_count': 0,
            'preflight': None,
            'result': None,
            'mono_archive': None,
        }
        manifest = paths['capture'] / 'pairs.csv'
        if manifest.is_file():
            with manifest.open('r', encoding='utf-8') as stream:
                status['capture_count'] = max(sum(1 for _ in stream) - 1, 0)
        if paths['preflight'].is_file():
            try:
                status['preflight'] = json.loads(paths['preflight'].read_text())
            except (OSError, json.JSONDecodeError):
                status['preflight'] = {'overall': 'INVALID'}
        report = paths['result'] / 'report.json'
        if report.is_file():
            try:
                status['result'] = json.loads(report.read_text())
            except (OSError, json.JSONDecodeError):
                status['result'] = {'overall': 'INVALID'}
        if task.kind == 'mono':
            status['mono_archive'] = mono_archive_summary(paths['mono_archive'])
        return status

    def state(self, task_id: str, session: str) -> dict:
        task = task_for(task_id)
        process = self.process.snapshot()
        self.relay.activate(task, publish_raw=self._relay_needs_raw(task, process))
        self.monitor.activate(task.stream_keys)
        streams = self.monitor.snapshot()
        gate_ok, gate_failures = stream_gate(task, streams)
        return {
            'demo': self.demo,
            'expected': {
                'width': EXPECTED_WIDTH,
                'height': EXPECTED_HEIGHT,
                'rate_hz': EXPECTED_RATE_HZ,
                'board': '11 x 8 squares / 10 x 7 inner corners / 70 mm',
            },
            'tasks': [asdict(item) for item in TASKS],
            'poses': list(POSES),
            'selected_task': asdict(task),
            'streams': streams,
            'stream_gate': {'pass': gate_ok, 'failures': gate_failures},
            'session': self.session_status(task, session),
            'process': process,
            'transport': self.relay.snapshot(),
            'data_root': str(self.data_root),
        }

    def start_action(self, payload: dict) -> dict:
        action = str(payload.get('action', ''))
        task = task_for(str(payload.get('task_id', '')))
        session = validate_session_name(str(payload.get('session', '')))
        paths = task_paths(self.data_root, session, task)
        paths['base'].mkdir(parents=True, exist_ok=True)

        publish_raw = (
            task.kind == 'mono' and action == 'calibrate'
            or task.kind == 'stereo' and action in {'preflight', 'capture'}
        )
        self.relay.activate(task, publish_raw=publish_raw)
        self.monitor.activate(task.stream_keys)
        streams = self.monitor.snapshot()
        gate_ok, failures = stream_gate(task, streams)
        if action in {'capture', 'calibrate'} and task.kind == 'mono' and not gate_ok:
            raise RuntimeError('; '.join(failures))
        if action == 'capture' and task.kind == 'stereo' and not gate_ok:
            raise RuntimeError('; '.join(failures))

        if action == 'preflight' and task.kind == 'mono':
            return {'started': False, 'preflight_pass': gate_ok, 'failures': failures}
        if action == 'stop':
            return {'stopped': self.process.stop()}

        baseline = payload.get('expected_baseline_m')
        if baseline in {'', None}:
            baseline_value = None
        else:
            baseline_value = float(baseline)
            if baseline_value <= 0:
                raise ValueError('expected baseline must be positive')

        if action == 'capture' and paths['capture'].exists() and any(paths['capture'].iterdir()):
            raise RuntimeError('capture directory is not empty; choose a new session')
        if action == 'calibrate' and task.kind == 'stereo':
            if not (paths['capture'] / 'pairs.csv').is_file():
                raise RuntimeError('no stereo capture manifest exists for this session')
            if paths['result'].exists() and any(paths['result'].iterdir()):
                raise RuntimeError('result directory is not empty; choose a new session')

        command = build_action_command(action, task, paths, baseline_value)
        on_exit = None
        if action == 'calibrate' and task.kind == 'mono':
            source_archive = Path('/tmp/calibrationdata.tar.gz')

            def save_mono_archive(return_code: int, started_at: float) -> None:
                if (
                    return_code == 0
                    and source_archive.is_file()
                    and source_archive.stat().st_mtime >= started_at - 1.0
                ):
                    paths['base'].mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source_archive, paths['mono_archive'])

            on_exit = save_mono_archive

        self.process.start(f'{task.task_id}:{action}', command, on_exit=on_exit)
        return {'started': True, 'command': command_text(command)}

    def close(self) -> None:
        self.process.close()
        self.relay.close()
        self.monitor.close()


def make_handler(app: CalibrationApp):
    from http.server import BaseHTTPRequestHandler

    class Handler(BaseHTTPRequestHandler):
        server_version = 'ARTCalibrationUI/1.0'

        def _json(self, status: int, data: dict) -> None:
            body = json.dumps(data, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _asset(self, filename: str, content_type: str) -> None:
            path = app.assets_dir / filename
            if not path.is_file():
                self.send_error(404)
                return
            body = path.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path in {'/', '/index.html'}:
                self._asset('index.html', 'text/html; charset=utf-8')
                return
            if parsed.path == '/styles.css':
                self._asset('styles.css', 'text/css; charset=utf-8')
                return
            if parsed.path == '/app.js':
                self._asset('app.js', 'text/javascript; charset=utf-8')
                return
            if parsed.path == '/api/health':
                self._json(200, {'status': 'ok', 'demo': app.demo})
                return
            if parsed.path == '/api/state':
                query = parse_qs(parsed.query)
                task_id = query.get('task', ['stereo_center'])[0]
                session = query.get('session', [time.strftime('%Y%m%d_%H%M%S')])[0]
                try:
                    self._json(200, app.state(task_id, validate_session_name(session)))
                except (ValueError, OSError) as exc:
                    self._json(400, {'error': str(exc)})
                return
            if parsed.path.startswith('/api/frame/') and parsed.path.endswith('.jpg'):
                key = parsed.path[len('/api/frame/'):-len('.jpg')]
                frame = app.monitor.frame(key)
                if frame is None:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(frame)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(frame)
                return
            self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802
            if self.path != '/api/action':
                self.send_error(404)
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if length <= 0 or length > 64 * 1024:
                    raise ValueError('invalid request body size')
                payload = json.loads(self.rfile.read(length).decode('utf-8'))
                self._json(200, app.start_action(payload))
            except (ValueError, RuntimeError, OSError, json.JSONDecodeError) as exc:
                self._json(400, {'error': str(exc)})

        def log_message(self, format_string: str, *args) -> None:
            return

    return Handler


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='ART local camera calibration web UI')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument(
        '--data-root', default='~/camera_calibration_data',
        help='local roar directory for capture sessions and results')
    parser.add_argument('--demo', action='store_true', help='show synthetic healthy streams')
    parser.add_argument(
        '--foxglove-url', default='ws://10.42.27.200:8766/',
        help='vehicle Foxglove Bridge used to relay only the selected camera')
    parser.add_argument(
        '--direct-ros', action='store_true',
        help='disable the managed Foxglove relay and subscribe to local ROS topics directly')
    parser.add_argument('--open-browser', action='store_true')
    parser.add_argument('--assets-dir', help=argparse.SUPPRESS)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    from http.server import ThreadingHTTPServer

    args = parse_args(argv)
    if not (1 <= args.port <= 65535):
        raise SystemExit('--port must be between 1 and 65535')
    assets_dir = (
        Path(args.assets_dir).resolve()
        if args.assets_dir else
        Path(get_package_share_directory('camera_calibration')) / 'web')
    if not (assets_dir / 'index.html').is_file():
        raise SystemExit(f'web assets are missing from {assets_dir}')

    app = CalibrationApp(
        assets_dir,
        Path(args.data_root),
        demo=args.demo,
        foxglove_url=None if args.direct_ros else args.foxglove_url,
    )
    server = ThreadingHTTPServer((args.host, args.port), make_handler(app))
    url = f'http://{args.host}:{server.server_address[1]}'
    print(f'ART calibration UI: {url}')
    print(f'Local data root: {app.data_root}')
    if args.open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        app.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
