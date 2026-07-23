"""Tests for terminal geometry PnP pose estimation."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_perception.terminal_geometry import target_to_pose


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_camera_matrix(fx: float = 800.0, fy: float = 800.0, cx: float = 320.0, cy: float = 240.0) -> np.ndarray:
    """Build a 3×3 pinhole intrinsic matrix."""
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


def _make_object_points(w: float = 0.2, h: float = 0.15) -> np.ndarray:
    """Return 4 target corner points in the XY world plane (Z=0).

    Order: top-left, top-right, bottom-right, bottom-left.
    """
    return np.array(
        [[0.0, 0.0, 0.0], [w, 0.0, 0.0], [w, -h, 0.0], [0.0, -h, 0.0]],
        dtype=np.float64,
    )


def _project_points(
    object_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
) -> np.ndarray:
    """Project 3D world points to 2D image points using OpenCV.

    Returns N×2 array of (u, v) pixel coordinates.
    """
    import cv2

    img_pts, _ = cv2.projectPoints(
        object_points.astype(np.float64),
        rvec,
        tvec,
        K.astype(np.float64),
        None,
    )
    return img_pts.reshape(-1, 2)


# ---------------------------------------------------------------------------
# test_terminal_geometry_known_target
# ---------------------------------------------------------------------------


def test_terminal_geometry_known_target() -> None:
    """Four-corner target at known pose → correct pose recovered via PnP."""
    K = _make_camera_matrix()
    object_points = _make_object_points(w=0.2, h=0.15)

    # Ground-truth extrinsics: camera at world (0.5, 0.2, 1.0) looking at origin.
    import cv2

    # Rotation: camera looking along -Z_world, Y_cam = -Y_world
    R_w2c = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float64)
    rvec_gt, _ = cv2.Rodrigues(R_w2c)
    # tvec = -R_w2c @ camera_center_world  (solvePnP convention)
    camera_center_world = np.array([[0.5], [0.2], [1.0]], dtype=np.float64)
    tvec_gt = -R_w2c @ camera_center_world

    image_points = _project_points(object_points, rvec_gt, tvec_gt, K)

    pose = target_to_pose(image_points, object_points, K)
    assert pose is not None

    # Camera position in world should match the ground-truth camera center.
    assert pose.pose.pose.position.x == pytest.approx(float(camera_center_world[0, 0]), abs=0.01)
    assert pose.pose.pose.position.y == pytest.approx(float(camera_center_world[1, 0]), abs=0.01)
    assert pose.pose.pose.position.z == pytest.approx(float(camera_center_world[2, 0]), abs=0.01)

    assert pose.header.frame_id == "camera_optical"


def test_terminal_geometry_pose_frame_id_override() -> None:
    """Frame ID can be customised."""
    K = _make_camera_matrix()
    object_points = _make_object_points()
    rvec = np.zeros((3, 1), dtype=np.float64)
    tvec = np.array([[0.0], [0.0], [1.0]], dtype=np.float64)
    image_points = _project_points(object_points, rvec, tvec, K)

    pose = target_to_pose(image_points, object_points, K, frame_id="terminal_frame")
    assert pose is not None
    assert pose.header.frame_id == "terminal_frame"


# ---------------------------------------------------------------------------
# test_terminal_rejects_missing_calibration
# ---------------------------------------------------------------------------


def test_terminal_rejects_missing_calibration() -> None:
    """None camera matrix → None pose."""
    object_points = _make_object_points()
    image_points = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float64)

    result = target_to_pose(image_points, object_points, None)  # type: ignore[arg-type]
    assert result is None


def test_terminal_rejects_bad_matrix_shape() -> None:
    """Malformed camera matrix → None pose."""
    object_points = _make_object_points()
    image_points = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float64)
    bad_K = np.eye(4, dtype=np.float64)

    result = target_to_pose(image_points, object_points, bad_K)
    assert result is None


def test_terminal_rejects_insufficient_keypoints() -> None:
    """Fewer than 4 points → None pose."""
    K = _make_camera_matrix()
    object_points = np.array([[0, 0, 0], [0.2, 0, 0], [0.2, -0.15, 0]], dtype=np.float64)
    image_points = np.array([[100, 100], [200, 100], [200, 200]], dtype=np.float64)

    result = target_to_pose(image_points, object_points, K)
    assert result is None


def test_terminal_rejects_none_inputs() -> None:
    """None image_points or object_points → None."""
    K = _make_camera_matrix()
    assert target_to_pose(None, np.zeros((4, 3)), K) is None  # type: ignore[arg-type]
    assert target_to_pose(np.zeros((4, 2)), None, K) is None  # type: ignore[arg-type]
