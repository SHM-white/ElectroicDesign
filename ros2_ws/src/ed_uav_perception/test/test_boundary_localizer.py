"""Tests for wide-camera boundary localizer: rectifier, extractor, localizer."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

try:
    import cv2  # type: ignore[import-untyped]

    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False


# ---------------------------------------------------------------------------
# test_rectifier_pinhole_model
# ---------------------------------------------------------------------------


def test_rectifier_pinhole_model() -> None:
    """Pinhole distortion correction preserves straight lines and image size."""
    from ed_uav_perception.rectifier import (
        CameraCalibration,
        make_pinhole_calibration,
        rectify,
    )

    w, h = 320, 240
    fx = fy = 400.0
    cx, cy = w / 2.0, h / 2.0

    # Create a synthetic checkerboard on a grey background.
    img = np.full((h, w, 3), 128, dtype=np.uint8)
    # Draw a vertical and horizontal line.
    cv2.line(img, (int(cx), 0), (int(cx), h), (255, 255, 255), 2)
    cv2.line(img, (0, int(cy)), (w, int(cy)), (255, 255, 255), 2)
    cv2.rectangle(img, (100, 80), (220, 160), (255, 255, 255), 2)

    # Distort with mild barrel (k1 < 0).
    calib = make_pinhole_calibration(
        fx=fx, fy=fy, cx=cx, cy=cy, width=w, height=h,
        k1=-0.3, k2=0.1, p1=0.0, p2=0.0, k3=0.0,
    )
    K = calib.K
    D = calib.D

    # Forward-distort the image (simulate a distorted capture).
    map_x, map_y = cv2.initUndistortRectifyMap(
        K, D, None, K, (w, h), cv2.CV_32FC1,
    )
    # Invert: map undistorted → distorted.
    distorted = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)

    # Rectify should recover straight lines.
    rectified = rectify(distorted, calib)
    assert rectified.shape == (h, w, 3)
    assert rectified.dtype == np.uint8

    # After rectification, lines should be approximately straight.
    # Check that the centre region still has the cross-hair.
    centre_pixel = rectified[int(cy), int(cx)]
    # The white cross-hair should be brighter than the grey background.
    assert np.mean(centre_pixel) > 180, "centre cross-hair not recovered"


def test_rectifier_rejects_bad_calibration() -> None:
    """Calibration with wrong K shape raises ValueError."""
    from ed_uav_perception.rectifier import CameraCalibration

    with pytest.raises(ValueError, match="3×3"):
        CameraCalibration(
            K=np.eye(2, dtype=np.float64),
            D=np.zeros(4, dtype=np.float64),
            width=320,
            height=240,
        )


def test_rectifier_fisheye_model() -> None:
    """Fisheye rectification produces output of correct dimensions."""
    from ed_uav_perception.rectifier import (
        make_fisheye_calibration,
        rectify,
    )

    w, h = 320, 240
    calib = make_fisheye_calibration(
        fx=300.0, fy=300.0, cx=160.0, cy=120.0,
        width=w, height=h,
        k1=0.1, k2=0.01, k3=0.001, k4=0.0001,
    )
    img = np.full((h, w, 3), 128, dtype=np.uint8)
    cv2.circle(img, (160, 120), 40, (255, 255, 255), -1)

    result = rectify(img, calib)
    assert result.shape == (h, w, 3)
    assert result.dtype == np.uint8


# ---------------------------------------------------------------------------
# test_boundary_detects_lines
# ---------------------------------------------------------------------------


def test_boundary_detects_lines() -> None:
    """Detect a dark line painted on a light background."""
    from ed_uav_perception.boundary_extractor import DARK_BOUNDARY, extract_lines

    w, h = 640, 480
    # Light-green "grass" background.
    img = np.full((h, w, 3), (60, 180, 60), dtype=np.uint8)
    # Dark boundary line (thick, diagonal).
    cv2.line(img, (100, 100), (500, 350), (20, 20, 20), 8)
    # Add some noise / texture.
    rng = np.random.RandomState(42)
    noise = rng.randint(-15, 15, img.shape, dtype=np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    lines = extract_lines(img, DARK_BOUNDARY, min_line_length=100.0)

    assert len(lines) >= 1, f"expected at least 1 line, got {len(lines)}"
    # The detected line should be roughly diagonal (slope ≈ (350-100)/(500-100)).
    main = lines[0]
    expected_slope = (350.0 - 100.0) / (500.0 - 100.0)  # ≈ 0.625
    actual_slope = abs((main.y2 - main.y1) / (main.x2 - main.x1 + 1e-9))
    # Allow ±30% tolerance for Hough discretisation.
    assert 0.3 < actual_slope < 1.2, f"unexpected slope: {actual_slope:.2f}"


def test_boundary_no_lines_on_uniform_image() -> None:
    """A uniform green image yields no lines."""
    from ed_uav_perception.boundary_extractor import DARK_BOUNDARY, extract_lines

    img = np.full((240, 320, 3), (60, 180, 60), dtype=np.uint8)
    lines = extract_lines(img, DARK_BOUNDARY)
    assert len(lines) == 0


def test_compute_intersection_detects_cross() -> None:
    """Two crossing lines produce a valid intersection point."""
    from ed_uav_perception.boundary_extractor import ImageLine, compute_intersection

    a = ImageLine(x1=100.0, y1=100.0, x2=300.0, y2=300.0)
    b = ImageLine(x1=100.0, y1=300.0, x2=300.0, y2=100.0)

    pt = compute_intersection(a, b)
    assert pt is not None
    assert pt == pytest.approx((200.0, 200.0), abs=1.0)


def test_compute_intersection_parallel_is_none() -> None:
    """Parallel lines return None."""
    from ed_uav_perception.boundary_extractor import ImageLine, compute_intersection

    a = ImageLine(x1=0.0, y1=0.0, x2=100.0, y2=0.0)
    b = ImageLine(x1=0.0, y1=50.0, x2=100.0, y2=50.0)

    assert compute_intersection(a, b) is None


# ---------------------------------------------------------------------------
# test_full_pose_two_lines
# ---------------------------------------------------------------------------


def test_full_pose_two_lines() -> None:
    """Two nonparallel matched lines produce full planar pose (DOF_X | DOF_Y | DOF_YAW)."""
    import rclpy

    rclpy.init()

    try:
        from ed_uav_perception.localizer import (
            DOF_X,
            DOF_Y,
            DOF_YAW,
            GroundLine,
            PlanarPose,
            compute_boundary_observation,
        )

        # Profile: two perpendicular boundary segments meeting at origin.
        # Segment "east":  horizontal at y=0, from x=0 to x=10 (normal points north).
        # Segment "north": vertical   at x=0, from y=0 to y=10 (normal points east).
        profile = [
            ("east", (0.0, 0.0), (10.0, 0.0)),
            ("north", (0.0, 0.0), (0.0, 10.0)),
        ]

        # True camera pose: (x=2.0, y=3.0).
        # IMU estimate overestimates x by 0.2 and underestimates y by 0.3.
        # No yaw error for simplicity (so angle offsets are zero).
        imu_pose = PlanarPose(x_m=2.2, y_m=2.7, yaw_rad=0.0)

        # Detected ground lines are the true segments shifted by the IMU error.
        # IMU error: Δx_imu = 2.2 - 2.0 = 0.2, Δy_imu = 2.7 - 3.0 = -0.3.
        # The detected line appears at segment + (Δx_imu, Δy_imu):
        lines = [
            GroundLine(x1_m=0.2, y1_m=-0.3, x2_m=10.2, y2_m=-0.3),
            GroundLine(x1_m=0.2, y1_m=-0.3, x2_m=0.2, y2_m=9.7),
        ]

        obs = compute_boundary_observation(
            ground_lines=lines,
            profile_boundary_segments=profile,
            imu_planar_pose=imu_pose,
            imu_age_sec=0.01,
            timestamp_sec=1000,
            timestamp_nanosec=0,
            frame_id="map",
            stale_imu_threshold_sec=0.5,
            full_pose_min_angle_deg=30.0,
            full_pose_min_inlier_ratio=0.60,
            expected_line_count=2,
        )
        assert obs is not None, "expected a full-pose observation"

        # Full mask: DOF_X | DOF_Y | DOF_YAW = 1 | 2 | 32 = 35.
        assert obs.observable_dof_mask == (DOF_X | DOF_Y | DOF_YAW), (
            f"expected full mask 35, got {obs.observable_dof_mask}"
        )

        # At least 2 constraints.
        assert obs.constraint_count >= 2

        # Confidence should be high.
        assert obs.confidence >= 0.60, f"confidence {obs.confidence} below threshold"

        # The corrected position should be closer to the true pose (2.0, 3.0)
        # than the raw IMU pose (2.2, 2.7).
        px = obs.pose.pose.position.x
        py = obs.pose.pose.position.y
        imu_err = np.hypot(2.2 - 2.0, 2.7 - 3.0)
        corrected_err = np.hypot(px - 2.0, py - 3.0)
        assert corrected_err < imu_err, (
            f"correction moved away from true: imu_err={imu_err:.3f}, "
            f"corrected_err={corrected_err:.3f}, pose=({px:.2f}, {py:.2f})"
        )

    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# test_partial_pose_single_line
# ---------------------------------------------------------------------------


def test_partial_pose_single_line() -> None:
    """A single matched line yields DOF_YAW only (partial observation)."""
    import rclpy

    rclpy.init()

    try:
        from ed_uav_perception.localizer import (
            DOF_YAW,
            GroundLine,
            PlanarPose,
            compute_boundary_observation,
        )

        # Profile: one segment.
        profile = [
            ("south", (0.0, 0.0), (10.0, 0.0)),
        ]

        imu_pose = PlanarPose(x_m=5.0, y_m=2.0, yaw_rad=0.0)

        # Single detected ground line — offset from the segment.
        lines = [
            GroundLine(x1_m=0.0, y1_m=0.5, x2_m=10.0, y2_m=0.5),
        ]

        obs = compute_boundary_observation(
            ground_lines=lines,
            profile_boundary_segments=profile,
            imu_planar_pose=imu_pose,
            imu_age_sec=0.01,
            timestamp_sec=1000,
            timestamp_nanosec=0,
            frame_id="map",
            stale_imu_threshold_sec=0.5,
            full_pose_min_angle_deg=30.0,
            full_pose_min_inlier_ratio=0.60,
            expected_line_count=1,
        )
        assert obs is not None, "expected a partial observation for single line"

        # Partial mask: DOF_YAW only (32).
        assert obs.observable_dof_mask == DOF_YAW, (
            f"expected DOF_YAW (32), got {obs.observable_dof_mask}"
        )

        # Position should be the uncorrected IMU position (we don't
        # correct X/Y with only one constraint).
        assert obs.pose.pose.position.x == pytest.approx(imu_pose.x_m)
        assert obs.pose.pose.position.y == pytest.approx(imu_pose.y_m)
        assert obs.constraint_count == 1

    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# test_rejects_stale_imu
# ---------------------------------------------------------------------------


def test_rejects_stale_imu() -> None:
    """An IMU older than the threshold produces no observation."""
    import rclpy

    rclpy.init()

    try:
        from ed_uav_perception.localizer import (
            GroundLine,
            PlanarPose,
            compute_boundary_observation,
        )

        profile = [
            ("east", (0.0, 0.0), (10.0, 0.0)),
            ("north", (0.0, 0.0), (0.0, 10.0)),
        ]
        imu_pose = PlanarPose(x_m=0.0, y_m=0.0, yaw_rad=0.0)
        lines = [
            GroundLine(x1_m=0.0, y1_m=0.0, x2_m=10.0, y2_m=0.0),
            GroundLine(x1_m=0.0, y1_m=0.0, x2_m=0.0, y2_m=10.0),
        ]

        obs = compute_boundary_observation(
            ground_lines=lines,
            profile_boundary_segments=profile,
            imu_planar_pose=imu_pose,
            imu_age_sec=1.5,  # well beyond the 0.5 s threshold
            timestamp_sec=1000,
            timestamp_nanosec=0,
            frame_id="map",
            stale_imu_threshold_sec=0.5,
        )
        assert obs is None, "stale IMU must produce None"

    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# test_rejects_missing_calibration
# ---------------------------------------------------------------------------


def test_rejects_missing_calibration() -> None:
    """Missing calibration flag produces no observation."""
    import rclpy

    rclpy.init()

    try:
        from ed_uav_perception.localizer import (
            GroundLine,
            PlanarPose,
            compute_boundary_observation,
        )

        profile = [("east", (0.0, 0.0), (10.0, 0.0))]
        imu_pose = PlanarPose(x_m=0.0, y_m=0.0, yaw_rad=0.0)
        lines = [GroundLine(x1_m=0.0, y1_m=0.0, x2_m=10.0, y2_m=0.0)]

        obs = compute_boundary_observation(
            ground_lines=lines,
            profile_boundary_segments=profile,
            imu_planar_pose=imu_pose,
            imu_age_sec=0.01,
            timestamp_sec=1000,
            timestamp_nanosec=0,
            frame_id="map",
            calibration_valid=False,
        )
        assert obs is None, "missing calibration must produce None"

    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# test_rejects_glare
# ---------------------------------------------------------------------------


def test_rejects_glare() -> None:
    """Glare (few lines when many expected) produces no observation."""
    import rclpy

    rclpy.init()

    try:
        from ed_uav_perception.localizer import (
            GroundLine,
            PlanarPose,
            compute_boundary_observation,
        )

        profile = [
            ("south", (0.0, 0.0), (10.0, 0.0)),
            ("east", (10.0, 0.0), (10.0, 10.0)),
            ("north", (10.0, 10.0), (0.0, 10.0)),
            ("west", (0.0, 10.0), (0.0, 0.0)),
        ]
        imu_pose = PlanarPose(x_m=5.0, y_m=5.0, yaw_rad=0.0)

        # Only 1 line detected but 4 are expected → glare.
        lines = [GroundLine(x1_m=0.0, y1_m=0.0, x2_m=10.0, y2_m=0.0)]

        obs = compute_boundary_observation(
            ground_lines=lines,
            profile_boundary_segments=profile,
            imu_planar_pose=imu_pose,
            imu_age_sec=0.01,
            timestamp_sec=1000,
            timestamp_nanosec=0,
            frame_id="map",
            expected_line_count=4,
            glare_line_threshold=2,
        )
        assert obs is None, "glare must produce None"

    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# test_visual_odometry_basic
# ---------------------------------------------------------------------------


def test_visual_odometry_basic() -> None:
    """Frame-to-frame motion estimation produces a valid result for small translation."""
    from ed_uav_perception.visual_odometry import estimate_motion

    w, h = 320, 240
    K = np.array([[400, 0, 160], [0, 400, 120], [0, 0, 1]], dtype=np.float64)

    # Create a textured scene (random dots).
    rng = np.random.RandomState(42)
    frame1 = rng.randint(0, 255, (h, w, 3), dtype=np.uint8)
    # Shift frame2 by 3 pixels right, 2 pixels down.
    M = np.float32([[1, 0, 3], [0, 1, 2]])
    frame2 = cv2.warpAffine(frame1, M, (w, h))

    result = estimate_motion(frame1, frame2, K)
    assert result is not None, "motion estimation failed"
    assert result.inlier_ratio >= 0.40


def test_visual_odometry_insufficient_features() -> None:
    """A uniform image with no features returns None."""
    from ed_uav_perception.visual_odometry import estimate_motion

    K = np.array([[400, 0, 160], [0, 400, 120], [0, 0, 1]], dtype=np.float64)
    frame = np.full((240, 320, 3), 128, dtype=np.uint8)

    result = estimate_motion(frame, frame, K)
    assert result is None, "uniform image should produce no result"


# ---------------------------------------------------------------------------
# test_ray_projection_ground
# ---------------------------------------------------------------------------


def test_ray_projection_to_ground() -> None:
    """Ray projection places a vertical image line onto the ground plane."""
    from ed_uav_perception.boundary_extractor import ImageLine
    from ed_uav_perception.localizer import project_ray_to_ground

    K = np.array([[400, 0, 160], [0, 400, 120], [0, 0, 1]], dtype=np.float64)

    # Camera at (0, 0, 10) looking straight down.
    # Downward-facing: camera X → world +X (east), camera Y → world +Y (north),
    # camera Z → world -Z (down).  This is a 180° rotation around X:
    #   [1  0   0]
    #   [0  1   0]   — actually this would make Z point UP.
    #   [0  0  -1]
    # For a drone: camera X → East, camera Y → South, camera Z → Down
    # R_cam_to_world = [[1,0,0],[0,-1,0],[0,0,-1]]
    R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float64)
    t = np.array([[0.0], [0.0], [10.0]], dtype=np.float64)

    # Image line: vertical line at x=200, from y=60 to y=180.
    img_line = ImageLine(x1=200.0, y1=60.0, x2=200.0, y2=180.0)

    ground = project_ray_to_ground(img_line, K, R, t, altitude_m=10.0)
    assert ground is not None, f"projection returned None"

    # The projected line should be on the ground plane (z=0 implied).
    # Since camera is at (0,0,10) looking down with the above rotation,
    # a line at x=200 (right of centre cx=160) maps to positive x.
    # Lower y values in image correspond to further north in world.
    assert ground.x1_m > 0.0, f"expected positive x, got x1={ground.x1_m}"
    assert abs(ground.x1_m - ground.x2_m) < 0.01, "vertical line should stay vertical on ground"
