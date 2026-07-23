"""Optical-flow and homography-based visual odometry.

Estimates frame-to-frame camera motion from the wide camera using
sparse feature tracking (Shi-Tomasi → KLT) followed by homography
decomposition.  All parameters are fixed at call time — no randomness.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore[import-untyped]

    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class FrameMotion:
    """Camera motion between two consecutive frames.

    Translation ``t`` is unit-length (up-to-scale) unless a scale source
    is provided externally.
    """

    R: np.ndarray  #: 3×3 rotation matrix (camera-frame rotation).
    t: np.ndarray  #: 3×1 unit translation vector.
    inlier_ratio: float  #: Fraction of tracked points used in the estimate.
    homography: Optional[np.ndarray] = None  #: 3×3 planar homography (may be None).


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def estimate_motion(
    prev_frame: np.ndarray,
    curr_frame: np.ndarray,
    K: np.ndarray,
    *,
    feature_count: int = 300,
    quality_level: float = 0.01,
    min_distance: float = 10.0,
    min_inlier_ratio: float = 0.40,
    ransac_threshold: float = 3.0,
) -> Optional[FrameMotion]:
    """Estimate camera motion between two frames.

    Pipeline
    --------
    1. Detect Shi-Tomasi corners in *prev_frame*.
    2. Track them into *curr_frame* via KLT optical flow.
    3. Estimate a homography between the matched point sets with RANSAC.
    4. Decompose the homography into (R, t) candidates and select the
       solution with positive depth.

    Returns ``None`` when tracking produces fewer than 8 inliers or the
    inlier ratio falls below ``min_inlier_ratio``.

    Args:
        prev_frame: Previous BGR image ``(H, W, 3)`` uint8.
        curr_frame: Current BGR image ``(H, W, 3)`` uint8.
        K: 3×3 intrinsic matrix shared by both frames.
        feature_count: Maximum number of corners to detect.
        quality_level: Shi-Tomasi quality level (0–1).
        min_distance: Minimum Euclidean distance between detected corners.
        min_inlier_ratio: Reject estimate when inlier fraction < this.
        ransac_threshold: RANSAC reprojection error threshold in pixels.

    Returns:
        ``FrameMotion`` on success, ``None`` on failure.
    """
    if not _HAS_CV2:
        raise RuntimeError("OpenCV (cv2) is required for visual odometry")

    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

    # --- feature detection ---
    prev_pts = cv2.goodFeaturesToTrack(
        prev_gray,
        maxCorners=feature_count,
        qualityLevel=quality_level,
        minDistance=min_distance,
    )
    if prev_pts is None or len(prev_pts) < 8:
        return None

    # --- KLT tracking ---
    curr_pts, status, _err = cv2.calcOpticalFlowPyrLK(
        prev_gray,
        curr_gray,
        prev_pts,
        None,
    )
    if curr_pts is None:
        return None

    mask = status.ravel() == 1
    prev_valid = prev_pts[mask].reshape(-1, 2)
    curr_valid = curr_pts[mask].reshape(-1, 2)

    if len(prev_valid) < 8:
        return None

    # --- homography estimation ---
    H, inlier_mask = cv2.findHomography(
        prev_valid,
        curr_valid,
        cv2.RANSAC,
        ransacReprojThreshold=ransac_threshold,
    )
    if H is None:
        return None

    inlier_count: int = int(inlier_mask.sum()) if inlier_mask is not None else 0
    inlier_ratio = inlier_count / len(prev_valid)

    if inlier_ratio < min_inlier_ratio:
        return None

    # --- decompose homography → (R, t) ---
    _num, rotations, translations, normals = cv2.decomposeHomographyMat(H, K)

    # Select the solution whose plane normal has largest z component
    # (camera-forward direction).
    best_idx = 0
    best_nz = -1.0
    for idx, n_vec in enumerate(normals):
        nz = abs(float(n_vec[2, 0]))
        if nz > best_nz:
            best_nz = nz
            best_idx = idx

    R = rotations[best_idx]
    t = translations[best_idx]
    # Normalise translation to unit length.
    t_norm = float(np.linalg.norm(t))
    if t_norm > 1e-9:
        t = t / t_norm

    return FrameMotion(
        R=R.astype(np.float64),
        t=t.astype(np.float64),
        inlier_ratio=inlier_ratio,
        homography=H.astype(np.float64),
    )
