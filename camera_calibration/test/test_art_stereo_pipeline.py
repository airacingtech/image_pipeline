from __future__ import annotations

from pathlib import Path
import unittest
from unittest import mock

from camera_calibration.art_stereo import detect_board, Detection, load_board
from camera_calibration.nodes import art_stereo_calibrate as calibration
import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BOARD_CONFIG = PACKAGE_ROOT / 'config' / 'art_stereo_board.yaml'


class BoardTest(unittest.TestCase):
    def test_confirmed_board_geometry_detects_10_by_7_inner_corners(self):
        board = load_board(BOARD_CONFIG)
        self.assertEqual((board.columns, board.rows), (10, 7))
        self.assertAlmostEqual(board.square_size_m, 0.070)

        square_px = 70
        squares_x = board.columns + 1
        squares_y = board.rows + 1
        margin = 100
        image = np.full(
            (squares_y * square_px + 2 * margin,
             squares_x * square_px + 2 * margin),
            255,
            dtype=np.uint8,
        )
        for row in range(squares_y):
            for column in range(squares_x):
                if (row + column) % 2 == 0:
                    x0 = margin + column * square_px
                    y0 = margin + row * square_px
                    image[y0:y0 + square_px, x0:x0 + square_px] = 0
        detection = detect_board(image, board, fast=False)
        self.assertIsNotNone(detection)
        self.assertEqual(detection.image_points.shape, (70, 2))

    def test_fast_detector_uses_small_sector_based_fallback(self):
        board = load_board(BOARD_CONFIG)
        image = np.zeros((1544, 2064), dtype=np.uint8)
        corners = (
            np.mgrid[0:10, 0:7].T.reshape(-1, 1, 2).astype(np.float32)
            * 20.0
            + 20.0
        )
        with (
            mock.patch(
                'camera_calibration.art_stereo.cv2.findChessboardCornersSB',
                return_value=(True, corners),
            ) as find_sb,
            mock.patch(
                'camera_calibration.art_stereo.cv2.findChessboardCorners',
            ) as find_classic,
            mock.patch('camera_calibration.art_stereo.cv2.cornerSubPix'),
        ):
            detection = detect_board(image, board, fast=True)

        self.assertIsNotNone(detection)
        self.assertEqual(detection.image_points.shape, (70, 2))
        find_sb.assert_called_once()
        find_classic.assert_not_called()


class SyntheticStereoTest(unittest.TestCase):
    def test_equal_gate_rejects_wrong_image_size(self):
        self.assertEqual(calibration.gate(2064.0, 2064.0, mode='equal')['status'], 'PASS')
        self.assertEqual(calibration.gate(516.0, 2064.0, mode='equal')['status'], 'FAIL')

    def test_recovers_nonzero_baseline_and_rectification(self):
        rng = np.random.default_rng(8)
        board = load_board(BOARD_CONFIG)
        object_points = np.zeros((board.columns * board.rows, 3), dtype=np.float32)
        object_points[:, :2] = (
            np.mgrid[0:board.columns, 0:board.rows].T.reshape(-1, 2)
            * board.square_size_m
        )
        ids = np.arange(len(object_points), dtype=np.int32)

        image_size = (1280, 720)
        k1 = np.array([[900.0, 0.0, 640.0], [0.0, 895.0, 360.0], [0.0, 0.0, 1.0]])
        k2 = np.array([[905.0, 0.0, 638.0], [0.0, 900.0, 358.0], [0.0, 0.0, 1.0]])
        d1 = np.array([-0.08, 0.015, 0.0005, -0.0003, 0.0])
        d2 = np.array([-0.07, 0.012, -0.0004, 0.0002, 0.0])
        rotation_lr = cv2.Rodrigues(np.array([0.002, -0.004, 0.001]))[0]
        translation_lr = np.array([[-0.300], [0.001], [0.002]])

        views = []
        attempts = 0
        while len(views) < 24 and attempts < 500:
            attempts += 1
            rvec_left = rng.uniform([-0.30, -0.35, -0.15], [0.30, 0.35, 0.15])
            rotation_board_left = cv2.Rodrigues(rvec_left)[0]
            translation_board_left = np.array([
                [rng.uniform(-0.30, 0.15)],
                [rng.uniform(-0.25, 0.05)],
                [rng.uniform(2.2, 4.5)],
            ])
            rotation_board_right = rotation_lr @ rotation_board_left
            translation_board_right = (
                rotation_lr @ translation_board_left + translation_lr)
            rvec_right = cv2.Rodrigues(rotation_board_right)[0]

            left_points, _ = cv2.projectPoints(
                object_points, rvec_left, translation_board_left, k1, d1)
            right_points, _ = cv2.projectPoints(
                object_points, rvec_right, translation_board_right, k2, d2)
            left_points = left_points.reshape(-1, 2)
            right_points = right_points.reshape(-1, 2)
            if not (
                np.all((left_points[:, 0] > 5) & (left_points[:, 0] < image_size[0] - 5))
                and np.all((left_points[:, 1] > 5) & (left_points[:, 1] < image_size[1] - 5))
                and np.all((right_points[:, 0] > 5) & (right_points[:, 0] < image_size[0] - 5))
                and np.all((right_points[:, 1] > 5) & (right_points[:, 1] < image_size[1] - 5))
            ):
                continue
            left_points += rng.normal(0.0, 0.08, left_points.shape)
            right_points += rng.normal(0.0, 0.08, right_points.shape)
            left_detection = Detection(
                left_points.astype(np.float32), object_points.copy(), ids.copy())
            right_detection = Detection(
                right_points.astype(np.float32), object_points.copy(), ids.copy())
            views.append(calibration.View(
                index=len(views),
                left_path=Path('/unused/left.png'),
                right_path=Path('/unused/right.png'),
                left=left_detection,
                right=right_detection,
                stereo_object=object_points.copy(),
                stereo_left=left_points.astype(np.float32),
                stereo_right=right_points.astype(np.float32),
                timestamp_delta_ms=0.2,
            ))

        self.assertEqual(len(views), 24)
        model = calibration.fit_model(views, image_size)
        baseline = float(np.linalg.norm(model.translation))
        errors = calibration.epipolar_vertical_errors(model, views)
        self.assertAlmostEqual(baseline, float(np.linalg.norm(translation_lr)), delta=0.01)
        self.assertLess(model.stereo_rms, 0.3)
        self.assertLess(float(np.percentile(errors, 95)), 0.5)
        self.assertGreater(abs(float(model.projection_right[0, 3])), 1.0)


if __name__ == '__main__':
    unittest.main()
