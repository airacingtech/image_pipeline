from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class BoardSpec:
    pattern: str
    columns: int
    rows: int
    square_size_m: float
    marker_size_m: Optional[float] = None
    aruco_dictionary: str = 'DICT_5X5_100'

    @property
    def minimum_points(self) -> int:
        return 12 if self.pattern == 'charuco' else self.columns * self.rows

    def as_dict(self) -> dict:
        return {
            'pattern': self.pattern,
            'columns': self.columns,
            'rows': self.rows,
            'square_size_m': self.square_size_m,
            'marker_size_m': self.marker_size_m,
            'aruco_dictionary': self.aruco_dictionary,
        }


@dataclass(frozen=True)
class Detection:
    image_points: np.ndarray  # Nx2 float32
    object_points: np.ndarray  # Nx3 float32, meters
    ids: np.ndarray  # N int32, stable board corner IDs


def _positive_number(data: dict, key: str) -> float:
    value = data.get(key)
    if value is None:
        raise ValueError(f'board config field {key!r} must be filled with a measured value')
    value = float(value)
    if value <= 0:
        raise ValueError(f'board config field {key!r} must be > 0')
    return value


def load_board(path: str | Path) -> BoardSpec:
    with Path(path).open('r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    pattern = str(data.get('pattern', '')).strip().lower()
    if pattern not in {'checkerboard', 'charuco'}:
        raise ValueError("board pattern must be 'checkerboard' or 'charuco'")

    columns = int(_positive_number(data, 'columns'))
    rows = int(_positive_number(data, 'rows'))
    if columns < 3 or rows < 3:
        raise ValueError('board columns and rows must both be at least 3')

    square_size_m = _positive_number(data, 'square_size_m')
    marker_size_m = None
    dictionary = str(data.get('aruco_dictionary', 'DICT_5X5_100'))
    if pattern == 'charuco':
        marker_size_m = _positive_number(data, 'marker_size_m')
        if marker_size_m >= square_size_m:
            raise ValueError('marker_size_m must be smaller than square_size_m')
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('this OpenCV build has no cv2.aruco; install opencv-contrib')
        if not hasattr(cv2.aruco, dictionary):
            raise ValueError(f'unknown OpenCV ArUco dictionary {dictionary!r}')

    return BoardSpec(
        pattern=pattern,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
        marker_size_m=marker_size_m,
        aruco_dictionary=dictionary,
    )


def checkerboard_object_points(spec: BoardSpec) -> np.ndarray:
    grid = np.zeros((spec.rows * spec.columns, 3), dtype=np.float32)
    grid[:, :2] = np.mgrid[0:spec.columns, 0:spec.rows].T.reshape(-1, 2)
    grid[:, :2] *= spec.square_size_m
    return grid


def _create_charuco_board(spec: BoardSpec):
    dictionary_id = getattr(cv2.aruco, spec.aruco_dictionary)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(cv2.aruco, 'CharucoBoard_create'):
        board = cv2.aruco.CharucoBoard_create(
            spec.columns,
            spec.rows,
            spec.square_size_m,
            spec.marker_size_m,
            dictionary,
        )
    else:
        board = cv2.aruco.CharucoBoard(
            (spec.columns, spec.rows),
            spec.square_size_m,
            spec.marker_size_m,
            dictionary,
        )
    return dictionary, board


def _charuco_object_corners(board) -> np.ndarray:
    if hasattr(board, 'getChessboardCorners'):
        return np.asarray(board.getChessboardCorners(), dtype=np.float32).reshape(-1, 3)
    return np.asarray(board.chessboardCorners, dtype=np.float32).reshape(-1, 3)


def _detect_checkerboard(gray: np.ndarray, spec: BoardSpec, fast: bool) -> Optional[Detection]:
    board_size = (spec.columns, spec.rows)
    corners = None
    found = False

    # During online capture, detect at half resolution and refine on the original image.
    if fast and min(gray.shape[:2]) >= 700:
        small = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        found, small_corners = cv2.findChessboardCorners(
            small,
            board_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK,
        )
        if found:
            corners = small_corners * 2.0

    if not found and not fast and hasattr(cv2, 'findChessboardCornersSB'):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        flags |= getattr(cv2, 'CALIB_CB_EXHAUSTIVE', 0)
        flags |= getattr(cv2, 'CALIB_CB_ACCURACY', 0)
        # This physical board has rounded black cells at the outer boundary and
        # is used in a dark garage. Raw grayscale occasionally fails even when
        # all internal corners are visible, so offline processing tries two
        # contrast-normalized copies before falling back to the classic API.
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        candidates = (gray, cv2.equalizeHist(gray), clahe.apply(gray))
        for candidate in candidates:
            found, corners = cv2.findChessboardCornersSB(
                candidate, board_size, flags=flags)
            if found:
                break

    if not found:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, board_size, flags)

    if not found or corners is None:
        return None

    corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
    cv2.cornerSubPix(
        gray,
        corners,
        (7, 7),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3),
    )
    image_points = corners.reshape(-1, 2)
    ids = np.arange(image_points.shape[0], dtype=np.int32)
    return Detection(image_points, checkerboard_object_points(spec), ids)


def _detect_charuco(gray: np.ndarray, spec: BoardSpec) -> Optional[Detection]:
    dictionary, board = _create_charuco_board(spec)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if marker_ids is None or len(marker_ids) < 2:
        return None

    count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
    )
    if charuco_ids is None or charuco_corners is None or int(count) < spec.minimum_points:
        return None

    ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    image_points = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
    all_object_points = _charuco_object_corners(board)
    return Detection(image_points, all_object_points[ids], ids)


def detect_board(gray: np.ndarray, spec: BoardSpec, *, fast: bool = False) -> Optional[Detection]:
    if gray.ndim != 2 or gray.dtype != np.uint8:
        raise ValueError('detect_board expects one uint8 grayscale image')
    if spec.pattern == 'checkerboard':
        return _detect_checkerboard(gray, spec, fast)
    return _detect_charuco(gray, spec)


def common_points(
    left: Detection,
    right: Detection,
    minimum_points: int,
) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
    common_ids, left_indices, right_indices = np.intersect1d(
        left.ids,
        right.ids,
        assume_unique=False,
        return_indices=True,
    )
    if common_ids.size < minimum_points:
        return None
    object_points = left.object_points[left_indices]
    return (
        object_points.astype(np.float32),
        left.image_points[left_indices].astype(np.float32),
        right.image_points[right_indices].astype(np.float32),
        common_ids.astype(np.int32),
    )
