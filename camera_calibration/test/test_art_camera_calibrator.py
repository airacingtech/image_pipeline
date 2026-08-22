#!/usr/bin/env python3

from camera_calibration.nodes.art_camera_calibrator import (
    build_cameracalibrator_args,
    camera_model,
    FISHEYE_CAMERAS,
    main,
    PINHOLE_STEREO_CAMERAS,
)
import pytest


@pytest.mark.parametrize('camera_name', sorted(FISHEYE_CAMERAS))
def test_four_surround_cameras_use_fisheye(camera_name):
    arguments = build_cameracalibrator_args(camera_name)

    assert camera_model(camera_name) == 'fisheye'
    assert arguments[:2] == ['--camera-model', 'fisheye']
    assert '--fisheye-recompute-extrinsics' in arguments
    assert '--fisheye-check-conditions' in arguments
    assert '--fisheye-fix-skew' in arguments
    assert f'image:=/{camera_name}/image' in arguments
    assert arguments[arguments.index('--size') + 1] == '7x10'
    assert arguments[arguments.index('--square') + 1] == '0.0700'


@pytest.mark.parametrize('camera_name', sorted(PINHOLE_STEREO_CAMERAS))
def test_center_stereo_cameras_use_pinhole(camera_name):
    arguments = build_cameracalibrator_args(camera_name)

    assert camera_model(camera_name) == 'pinhole'
    assert arguments[:2] == ['--camera-model', 'pinhole']
    assert not any(value.startswith('--fisheye-') for value in arguments)


def test_custom_image_topic_is_preserved():
    arguments = build_cameracalibrator_args(
        'vimba_front', '/vimba_calib_front/image')

    assert 'image:=/vimba_calib_front/image' in arguments


def test_relative_image_topic_is_rejected():
    with pytest.raises(ValueError, match='absolute ROS topic'):
        build_cameracalibrator_args('vimba_front', 'vimba_front/image')


def test_unknown_camera_is_rejected():
    with pytest.raises(ValueError, match='Unsupported ART camera'):
        camera_model('vimba_unknown')


def test_print_command_exposes_resolved_contract(capsys):
    assert main(['vimba_rear', '--print-command']) == 0

    output = capsys.readouterr().out
    assert 'cameracalibrator --camera-model fisheye' in output
    assert '--size 7x10 --square 0.0700' in output
    assert 'image:=/vimba_rear/image' in output
