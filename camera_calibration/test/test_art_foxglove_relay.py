from __future__ import annotations

import struct

import cv2
import numpy as np
from camera_calibration.nodes.art_foxglove_relay import (
    CAMERA_TOPICS,
    FoxgloveRelay,
    parse_message_frame,
)
from camera_calibration.nodes.art_stereo_capture import image_to_gray
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


def test_preview_compression_demosaics_bayer_before_jpeg():
    image = Image()
    image.width = 128
    image.height = 96
    image.step = 128
    image.encoding = 'bayer_rggb8'

    # A deliberately strong Bayer color imbalance exposes the 2x2 mosaic if
    # the relay accidentally treats raw Bayer bytes as a mono image.
    mosaic = np.empty((image.height, image.width), dtype=np.uint8)
    mosaic[0::2, 0::2] = 220
    mosaic[0::2, 1::2] = 130
    mosaic[1::2, 0::2] = 130
    mosaic[1::2, 1::2] = 35
    image.data = mosaic.tobytes()

    preview = FoxgloveRelay._compress_preview(image)
    decoded = cv2.imdecode(
        np.frombuffer(preview.data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    expected = image_to_gray(image)

    assert decoded.shape == expected.shape
    assert np.mean(np.abs(decoded.astype(np.int16) - expected.astype(np.int16))) < 3.0
    assert np.std(decoded.astype(np.float32)) < 3.0
