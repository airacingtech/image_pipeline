#!/usr/bin/env python3

import json
from unittest import mock

from camera_calibration.calibrator import (
    _calibrate_fisheye,
    _get_corners,
    CalibrationException,
    CAMERA_MODEL,
    ChessboardInfo,
    MonoCalibrator,
    Patterns,
)
from camera_calibration.camera_calibrator import OpenCVCalibrationNode
import cv2
import numpy
import pytest


def _points(count):
    object_points = [numpy.full((6, 1, 3), index, dtype=numpy.float64)
                     for index in range(count)]
    image_points = [numpy.full((6, 1, 2), index, dtype=numpy.float64)
                    for index in range(count)]
    return object_points, image_points


def _success_result():
    return (
        0.5,
        numpy.eye(3, dtype=numpy.float64),
        numpy.zeros((4, 1), dtype=numpy.float64),
        [],
        [],
    )


def test_checkerboard_detection_uses_sb_fallback_after_classic_miss():
    board = ChessboardInfo('chessboard', 10, 7, 0.07)
    image = numpy.zeros((480, 640), dtype=numpy.uint8)
    corners = numpy.mgrid[0:10, 0:7].T.reshape(-1, 1, 2).astype(numpy.float32)
    corners *= 20.0
    corners += 20.0

    with (
        mock.patch(
            'camera_calibration.calibrator.cv2.findChessboardCorners',
            return_value=(False, None),
        ),
        mock.patch(
            'camera_calibration.calibrator.cv2.findChessboardCornersSB',
            return_value=(True, corners.copy()),
        ) as find_sb,
        mock.patch('camera_calibration.calibrator.cv2.cornerSubPix'),
    ):
        found, detected = _get_corners(
            image, board, checkerboard_flags=0)

    assert found is True
    assert detected.shape == (70, 1, 2)
    find_sb.assert_called_once()


def test_camera_model_is_available_from_installed_module():
    assert CAMERA_MODEL.FISHEYE.value == 1


def test_full_coverage_mode_does_not_treat_40_samples_as_complete():
    board = ChessboardInfo('chessboard', 10, 7, 0.07)
    calibrator = MonoCalibrator(
        [board],
        pattern=Patterns.Chessboard,
        require_full_coverage=True,
    )
    calibrator.db = [([0.5, 0.5, 0.1, 0.0], None)] * 40

    calibrator.compute_goodenough()

    assert calibrator.goodenough is False


def test_default_mode_preserves_upstream_40_sample_shortcut():
    board = ChessboardInfo('chessboard', 10, 7, 0.07)
    calibrator = MonoCalibrator([board], pattern=Patterns.Chessboard)
    calibrator.db = [([0.5, 0.5, 0.1, 0.0], None)] * 40

    calibrator.compute_goodenough()

    assert calibrator.goodenough is True


def test_full_coverage_mode_completes_when_every_dimension_is_covered():
    board = ChessboardInfo('chessboard', 10, 7, 0.07)
    calibrator = MonoCalibrator(
        [board],
        pattern=Patterns.Chessboard,
        require_full_coverage=True,
    )
    calibrator.db = [
        ([0.0, 0.0, 0.1, 0.0], None),
        ([0.7, 0.7, 0.4, 0.5], None),
    ]

    calibrator.compute_goodenough()

    assert calibrator.goodenough is True


def test_fisheye_calibration_preserves_inputs_and_flags():
    object_points, image_points = _points(10)
    flags = cv2.fisheye.CALIB_CHECK_COND

    with mock.patch(
        'camera_calibration.calibrator.cv2.fisheye.calibrate',
        return_value=_success_result(),
    ) as calibrate:
        result = _calibrate_fisheye(
            object_points, image_points, (1032, 772), numpy.eye(3),
            flags)

    assert result[0] == 0.5
    calibrate.assert_called_once()
    args = calibrate.call_args.args
    assert args[0].dtype == numpy.float64
    assert args[1].dtype == numpy.float64
    assert args[2] == (1032, 772)
    numpy.testing.assert_array_equal(args[3], numpy.eye(3))
    assert args[4] is None
    assert calibrate.call_args.kwargs == {'flags': flags}


def test_conditioning_failure_is_not_retried_or_hidden():
    object_points, image_points = _points(20)

    with mock.patch(
        'camera_calibration.calibrator.cv2.fisheye.calibrate',
        side_effect=cv2.error(
            'CALIB_CHECK_COND - Ill-conditioned matrix for input array 0'
        ),
    ) as calibrate:
        with pytest.raises(CalibrationException, match='Capture more varied'):
            _calibrate_fisheye(
                object_points, image_points, (1032, 772), numpy.eye(3),
                cv2.fisheye.CALIB_CHECK_COND)

    calibrate.assert_called_once()


def test_non_conditioning_error_is_actionable():
    object_points, image_points = _points(10)

    with mock.patch(
        'camera_calibration.calibrator.cv2.fisheye.calibrate',
        side_effect=cv2.error('unexpected OpenCV failure'),
    ):
        with pytest.raises(CalibrationException, match='unexpected OpenCV failure'):
            _calibrate_fisheye(
                object_points, image_points, (1032, 772), numpy.eye(3), 0)


@pytest.mark.parametrize(
    'result',
    [
        (numpy.nan, numpy.eye(3), numpy.zeros((4, 1)), [], []),
        (0.5, numpy.full((3, 3), numpy.nan), numpy.zeros((4, 1)), [], []),
        (0.5, numpy.diag([-1.0, 1.0, 1.0]), numpy.zeros((4, 1)), [], []),
        (0.5, numpy.eye(3), numpy.zeros((5, 1)), [], []),
    ],
)
def test_invalid_fisheye_solutions_are_rejected(result):
    object_points, image_points = _points(10)
    with mock.patch(
        'camera_calibration.calibrator.cv2.fisheye.calibrate',
        return_value=result,
    ):
        with pytest.raises(CalibrationException, match='invalid solution'):
            _calibrate_fisheye(
                object_points, image_points, (1032, 772), numpy.eye(3), 0)


def test_synthetic_fisheye_solution_exports_equidistant_camera_info():
    board = ChessboardInfo('chessboard', 9, 6, 0.04)
    calibrator = MonoCalibrator([board], pattern=Patterns.Chessboard)
    calibrator.size = (1032, 772)
    calibrator.set_cammodel(CAMERA_MODEL.FISHEYE)
    calibrator.fisheye_calib_flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        | cv2.fisheye.CALIB_CHECK_COND
        | cv2.fisheye.CALIB_FIX_SKEW
    )

    object_points = calibrator.mk_object_points(calibrator._boards)[0].astype(
        numpy.float64
    )
    true_intrinsics = numpy.array(
        [[500.0, 0.0, 516.0], [0.0, 505.0, 386.0], [0.0, 0.0, 1.0]]
    )
    true_distortion = numpy.array([[-0.12], [0.02], [-0.003], [0.0005]])
    samples = []
    for index in range(18):
        rotation = numpy.array(
            [
                -0.35 + 0.7 * ((index % 3) / 2.0),
                -0.45 + 0.9 * (((index // 3) % 3) / 2.0),
                -0.2 + 0.4 * ((index % 5) / 4.0),
            ]
        )
        translation = numpy.array(
            [
                -0.18 + 0.36 * ((index % 4) / 3.0),
                -0.12 + 0.24 * (((index // 4) % 4) / 3.0),
                0.65 + 0.06 * (index % 3),
            ]
        )
        image_points, _ = cv2.fisheye.projectPoints(
            object_points * board.dim,
            rotation,
            translation,
            true_intrinsics,
            true_distortion,
        )
        samples.append((image_points, None, calibrator._boards[0]))

    calibrator.cal_fromcorners(samples)
    message = calibrator.as_message()

    assert calibrator.reprojection_error < 1e-6
    assert message.distortion_model == 'equidistant'
    assert len(message.d) == 4
    numpy.testing.assert_allclose(calibrator.intrinsics, true_intrinsics, atol=1e-6)
    numpy.testing.assert_allclose(calibrator.distortion, true_distortion, atol=1e-6)


def test_calibration_worker_reports_failure_and_resets_running_state():
    node = object.__new__(OpenCVCalibrationNode)
    node.c = mock.Mock()
    node.c.do_calibration.side_effect = CalibrationException('bad views')
    node.c.good_corners = [object()]
    node._calibration_running = True
    logger = mock.Mock()
    node.get_logger = mock.Mock(return_value=logger)

    node._run_calibration()

    assert node._calibration_running is False
    assert len(node.c.good_corners) == 1
    logger.error.assert_called_once()


def test_calibration_worker_automatically_saves_progress(tmp_path):
    node = object.__new__(OpenCVCalibrationNode)
    node.c = mock.Mock()
    node.c.db = [object(), object()]
    node.c.goodenough = True
    node.c.calibrated = True
    node._calibration_running = True
    node._auto_save_path = str(tmp_path / 'calibrationdata.tar.gz')
    node._auto_progress_path = str(tmp_path / 'progress.json')
    node._auto_exit = False
    node._auto_phase = 'calibrating'
    node._last_auto_progress = None
    logger = mock.Mock()
    node.get_logger = mock.Mock(return_value=logger)

    node._run_calibration()

    node.c.do_save.assert_called_once_with(node._auto_save_path)
    progress = json.loads((tmp_path / 'progress.json').read_text())
    assert progress['status'] == 'saved'
    assert progress['samples'] == 2
    assert progress['goodenough'] is True
    assert node._calibration_running is False
    logger.info.assert_called_once()
