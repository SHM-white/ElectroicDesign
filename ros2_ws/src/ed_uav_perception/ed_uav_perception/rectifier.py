"""Pinhole and fisheye rectification with serial calibration.

Provides deterministic image undistortion for the 135-degree wide camera,
supporting both standard pinhole (radial/tangential) and OpenCV fisheye
(Kannala-Brandt) distortion models.  Calibration is resolution-bound and
immutable once constructed.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

try:
    import cv2  # type: ignore[import-untyped]

    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

# ---------------------------------------------------------------------------
# Calibration model
# ---------------------------------------------------------------------------

DistortionModel = Literal["pinhole", "fisheye"]


@dataclass(frozen=True, slots=True)
class CameraCalibration:
    """Immutable, resolution-bound camera calibration for rectification.

    Attributes:
        K: 3×3 intrinsic camera matrix ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``.
        D: Distortion coefficients.  For ``pinhole`` this is the 4/5/8/12/14-element
            OpenCV vector (k1, k2, p1, p2, k3, ...).  For ``fisheye`` it is the
            4-element vector (k1, k2, k3, k4).
        width: Image width in pixels.
        height: Image height in pixels.
        model: Distortion model identifier.
    """

    K: np.ndarray
    D: np.ndarray
    width: int
    height: int
    model: DistortionModel = "pinhole"

    def __post_init__(self) -> None:
        if self.K.shape != (3, 3):
            raise ValueError(f"K must be 3×3, got {self.K.shape}")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be positive")


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def rectify(image: np.ndarray, calib: CameraCalibration) -> np.ndarray:
    """Remove lens distortion from a single image.

    Args:
        image: Input image as ``(H, W, C)`` uint8 array (BGR or RGB).
        calib: Camera calibration returned by the calibration system.

    Returns:
        Undistorted image with the same dtype and channel count.

    Raises:
        RuntimeError: If OpenCV is not available.
        ValueError:  If *calib.model* is unrecognised.
    """
    if not _HAS_CV2:
        raise RuntimeError("OpenCV (cv2) is required for rectification")
    if calib.model == "pinhole":
        return _rectify_pinhole(image, calib)
    if calib.model == "fisheye":
        return _rectify_fisheye(image, calib)
    raise ValueError(f"Unknown distortion model: {calib.model}")


# ---------------------------------------------------------------------------
# Internals
# ---------------------------------------------------------------------------


def _rectify_pinhole(image: np.ndarray, calib: CameraCalibration) -> np.ndarray:
    new_K, _roi = cv2.getOptimalNewCameraMatrix(
        calib.K, calib.D, (calib.width, calib.height), alpha=1.0
    )
    return cv2.undistort(image, calib.K, calib.D, None, new_K)


def _rectify_fisheye(image: np.ndarray, calib: CameraCalibration) -> np.ndarray:
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        calib.K, calib.D, (calib.width, calib.height), np.eye(3), balance=1.0
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        calib.K,
        calib.D,
        np.eye(3),
        new_K,
        (calib.width, calib.height),
        cv2.CV_16SC2,
    )
    return cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)


# ---------------------------------------------------------------------------
# Builder helpers
# ---------------------------------------------------------------------------


def make_pinhole_calibration(
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    width: int,
    height: int,
    k1: float = 0.0,
    k2: float = 0.0,
    p1: float = 0.0,
    p2: float = 0.0,
    k3: float = 0.0,
) -> CameraCalibration:
    """Construct a pinhole calibration with explicit parameters (for tests)."""
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
    return CameraCalibration(K=K, D=D, width=width, height=height, model="pinhole")


def make_fisheye_calibration(
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    width: int,
    height: int,
    k1: float = 0.0,
    k2: float = 0.0,
    k3: float = 0.0,
    k4: float = 0.0,
) -> CameraCalibration:
    """Construct a fisheye calibration with explicit parameters (for tests)."""
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.array([k1, k2, k3, k4], dtype=np.float64)
    return CameraCalibration(K=K, D=D, width=width, height=height, model="fisheye")
