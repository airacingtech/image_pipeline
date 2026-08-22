from __future__ import annotations

import struct

import cv2
import numpy as np
from camera_calibration.nodes.art_foxglove_relay import (
    CAMERA_TOPICS,
    FoxgloveRelay,
    parse_message_frame,
)
from sensor_msgs.msg import Image


def test_all_calibration_tasks_have_expected_vehicle_raw_topics():
    assert CAMERA_TOPICS['stereo'][0][0] == '/vimba_calib_left/image'
    assert CAMERA_TOPICS['stereo'][1][0] == '/vimba_calib_right/image'
    assert CAMERA_TOPICS['front'][0][0] == '/vimba_front/image'
    assert CAMERA_TOPICS['left'][0][0] == '/vimba_left/image'
    assert CAMERA_TOPICS['right'][0][0] == '/vimba_right/image'
    assert CAMERA_TOPICS['rear'][0][0] == '/vimba_rear/image'


def test_parse_message_frame_extracts_subscription_timestamp_and_cdr():
    payload = b'cdr-payload'
    frame = b'\x01' + struct.pack('<IQ', 7, 123456789) + payload

    assert parse_message_frame(frame) == (7, 123456789, payload)


def test_preview_compression_preserves_full_source_dimensions():
    image = Image()
    image.width = 32
    image.height = 24
    image.step = 32
    image.encoding = 'bayer_rggb8'
    image.data = np.arange(32 * 24, dtype=np.uint8).tobytes()

    preview = FoxgloveRelay._compress_preview(image)
    decoded = cv2.imdecode(
        np.frombuffer(preview.data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)

    assert preview.format == 'jpeg; mono8; source=32x24; preview=32x24'
    assert decoded.shape == (24, 32)
