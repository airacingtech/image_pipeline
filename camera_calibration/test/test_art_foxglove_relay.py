from __future__ import annotations

import struct

from camera_calibration.nodes.art_foxglove_relay import (
    CAMERA_TOPICS,
    parse_message_frame,
)


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
