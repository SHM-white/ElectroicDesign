"""Ray-projection localizer: ground-plane intersection + field-profile association.

Projects image-space line segments onto the ENU ground plane (z=0) using
camera intrinsics, stamped attitude, and altitude/range, then associates
them with known boundary segments from the field profile.  Emits
``BoundaryObservation`` messages with calibrated observable-DOF masks.

Key guard rails
---------------
* Full pose (X, Y, Yaw) requires ≥ 2 nonparallel matched constraints with
  a minimum inter-line angle of 30° and inlier ratio ≥ 0.60.
* A single matched line constrains only the normal distance/orientation,
  producing the ``DOF_YAW`` mask alone.
* Missing calibration, stale IMU (age > threshold), glare (low line count
  while expectation is high), or ambiguous association → ``None`` (no
  observation published).
* YOLO detections are never used as a pose source.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore[import-untyped]

    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
)
from std_msgs.msg import Header

# ed_uav_interfaces BoundaryObservation constants (mirrored here for
# pure-Python use without importing the ROS message at module level).
DOF_X: int = 1
DOF_Y: int = 2
DOF_Z: int = 4
DOF_ROLL: int = 8
DOF_PITCH: int = 16
DOF_YAW: int = 32

# ---------------------------------------------------------------------------
# Internal data types
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class GroundLine:
    """A line segment projected onto the ENU ground plane (z = 0 map frame)."""

    x1_m: float
    y1_m: float
    x2_m: float
    y2_m: float

    @property
    def direction(self) -> tuple[float, float]:
        """Unit direction vector from start to end."""
        dx = self.x2_m - self.x1_m
        dy = self.y2_m - self.y1_m
        length = float(np.hypot(dx, dy))
        if length < 1e-9:
            return (0.0, 0.0)
        return (dx / length, dy / length)

    @property
    def normal(self) -> tuple[float, float]:
        """Unit normal pointing to the *right* of the direction."""
        dx, dy = self.direction
        return (-dy, dx)

    @property
    def midpoint(self) -> tuple[float, float]:
        """Segment midpoint in metres."""
        return ((self.x1_m + self.x2_m) / 2.0, (self.y1_m + self.y2_m) / 2.0)

    @property
    def signed_distance(self) -> float:
        """Signed distance from origin along the normal (n·p for any point on line)."""
        nx, ny = self.normal
        mx, my = self.midpoint
        return float(mx * nx + my * ny)


@dataclass(frozen=True, slots=True)
class PlanarPose:
    """A 3-DOF ENU planar pose (x, y, yaw in radians)."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass
class MatchResult:
    """Outcome of line-to-segment association."""

    ground_line: GroundLine
    segment_id: str
    segment_start: tuple[float, float]
    segment_end: tuple[float, float]
    normal_offset_m: float  #: Signed perpendicular distance (detected – segment).
    angle_offset_rad: float  #: Orientation difference (segment_dir – detected_dir).


# ---------------------------------------------------------------------------
# Ray projection
# ---------------------------------------------------------------------------


def _image_line_to_ground(
    image_line: "ImageLine",  # from boundary_extractor
    K: np.ndarray,
    R_cam_to_world: np.ndarray,
    t_cam_to_world: np.ndarray,
    altitude_m: float,
) -> Optional[GroundLine]:
    """Project an image-space line onto the ground plane (z = 0).

    The camera is assumed to be at altitude *altitude_m* above the ground
    plane.  The ground plane is z = 0 in the world (ENU map) frame.

    Returns ``None`` when either endpoint ray is parallel to the ground,
    points upward, or is behind the camera.
    """
    K_inv = np.linalg.inv(K)

    p1_img = np.array([image_line.x1, image_line.y1, 1.0], dtype=np.float64)
    p2_img = np.array([image_line.x2, image_line.y2, 1.0], dtype=np.float64)

    d1_cam = K_inv @ p1_img
    d2_cam = K_inv @ p2_img

    d1_world = R_cam_to_world @ d1_cam
    d2_world = R_cam_to_world @ d2_cam

    cam_x, cam_y, cam_z = float(t_cam_to_world[0, 0]), float(t_cam_to_world[1, 0]), float(t_cam_to_world[2, 0])

    # Ground intersection: cam_z + λ * d_z = 0  →  λ = -cam_z / d_z.
    dz1 = float(d1_world[2])
    dz2 = float(d2_world[2])

    if abs(dz1) < 1e-9 or abs(dz2) < 1e-9:
        return None

    lam1 = -cam_z / dz1
    lam2 = -cam_z / dz2

    if lam1 < 0.0 or lam2 < 0.0:
        return None

    g1_x = cam_x + lam1 * float(d1_world[0])
    g1_y = cam_y + lam1 * float(d1_world[1])
    g2_x = cam_x + lam2 * float(d2_world[0])
    g2_y = cam_y + lam2 * float(d2_world[1])

    return GroundLine(x1_m=g1_x, y1_m=g1_y, x2_m=g2_x, y2_m=g2_y)


# ---------------------------------------------------------------------------
# Profile segment geometry helpers
# ---------------------------------------------------------------------------


def _segment_direction(
    start: tuple[float, float], end: tuple[float, float]
) -> tuple[float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = float(np.hypot(dx, dy))
    if length < 1e-9:
        return (0.0, 0.0)
    return (dx / length, dy / length)


def _segment_normal(
    start: tuple[float, float], end: tuple[float, float]
) -> tuple[float, float]:
    dx, dy = _segment_direction(start, end)
    return (-dy, dx)


def _segment_midpoint(
    start: tuple[float, float], end: tuple[float, float]
) -> tuple[float, float]:
    return ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)


def _angle_between_directions(
    d1: tuple[float, float], d2: tuple[float, float]
) -> float:
    """Absolute acute angle between two direction vectors, in radians."""
    dot = d1[0] * d2[0] + d1[1] * d2[1]
    dot = max(-1.0, min(1.0, dot))
    return float(np.arccos(abs(dot)))


def _perpendicular_distance(
    line: GroundLine,
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    """Signed perpendicular distance from segment midpoint to the ground line.

    Positive when the segment midpoint is to the right of the line direction.
    """
    nx, ny = line.normal
    mx, my = _segment_midpoint(start, end)
    lm = line.midpoint
    # Distance from segment midpoint to the infinite line through the ground line.
    return float((mx - lm[0]) * nx + (my - lm[1]) * ny)


# ---------------------------------------------------------------------------
# Line association
# ---------------------------------------------------------------------------


def associate_lines(
    ground_lines: list[GroundLine],
    boundary_segments: list[tuple[str, tuple[float, float], tuple[float, float]]],
    *,
    max_angle_deg: float = 20.0,
    max_distance_m: float = 3.0,
) -> list[MatchResult]:
    """Match projected ground lines to known boundary segments.

    Each *boundary_segment* is ``(segment_id, (start_x, start_y), (end_x, end_y))``.

    Association uses orientation similarity first, then perpendicular
    distance.  Each profile segment can match at most one ground line
    (best distance wins); each ground line can match at most one segment.
    """
    max_angle_rad = np.deg2rad(max_angle_deg)
    candidates: list[tuple[int, int, float, float]] = []

    for gi, gline in enumerate(ground_lines):
        gdir = gline.direction
        for si, (sid, sstart, send) in enumerate(boundary_segments):
            sdir = _segment_direction(sstart, send)
            ang = _angle_between_directions(gdir, sdir)
            if ang > max_angle_rad:
                continue
            dist = _perpendicular_distance(gline, sstart, send)
            if abs(dist) > max_distance_m:
                continue
            candidates.append((gi, si, ang, dist))

    if not candidates:
        return []

    # Sort by angle error then distance.
    candidates.sort(key=lambda c: (c[2], abs(c[3])))

    used_ground: set[int] = set()
    used_segment: set[int] = set()
    matches: list[MatchResult] = []

    for gi, si, ang, dist in candidates:
        if gi in used_ground or si in used_segment:
            continue
        used_ground.add(gi)
        used_segment.add(si)
        sid, sstart, send = boundary_segments[si]
        matches.append(
            MatchResult(
                ground_line=ground_lines[gi],
                segment_id=sid,
                segment_start=sstart,
                segment_end=send,
                normal_offset_m=dist,
                angle_offset_rad=ang,
            )
        )

    return matches


# ---------------------------------------------------------------------------
# DOF mask computation
# ---------------------------------------------------------------------------


def _has_nonparallel_constraints(
    matches: list[MatchResult],
    min_angle_deg: float = 30.0,
) -> bool:
    """Return ``True`` when ≥ 2 matches have nonparallel segment normals."""
    if len(matches) < 2:
        return False
    min_angle_rad = np.deg2rad(min_angle_deg)
    max_angle_rad = np.pi - min_angle_rad

    normals = [
        _segment_normal(m.segment_start, m.segment_end) for m in matches
    ]
    for i in range(len(normals)):
        for j in range(i + 1, len(normals)):
            dot = normals[i][0] * normals[j][0] + normals[i][1] * normals[j][1]
            dot = max(-1.0, min(1.0, dot))
            ang = float(np.arccos(abs(dot)))
            if min_angle_rad <= ang <= max_angle_rad:
                return True
    return False


def _solve_planar_correction(
    matches: list[MatchResult],
) -> tuple[float, float, float]:
    """Solve for (Δx, Δy, Δθ) correction from matched constraints.

    Uses a least-squares fit over all normal-distance and angle constraints.
    Returns ``(Δx_m, Δy_m, Δyaw_rad)``.
    """
    if len(matches) == 0:
        return (0.0, 0.0, 0.0)

    # Build the linear system:  J · [Δx, Δy, Δθ] = b
    # For each match:
    #   normal_offset = nx·Δx + ny·Δy + 0·Δθ
    #   angle_offset  = 0·Δx + 0·Δy + 1·Δθ
    n = len(matches)
    J = np.zeros((2 * n, 3), dtype=np.float64)
    b = np.zeros(2 * n, dtype=np.float64)

    for i, m in enumerate(matches):
        nx, ny = _segment_normal(m.segment_start, m.segment_end)
        J[2 * i, 0] = nx
        J[2 * i, 1] = ny
        # The detected line is offset from the true segment by the IMU error.
        # To correct the IMU pose we negate the observed offset: when the
        # detected line is +d north of the segment, the camera was offset
        # -d south → correction = -d in the normal direction.
        b[2 * i] = -m.normal_offset_m
        J[2 * i + 1, 2] = 1.0
        b[2 * i + 1] = -m.angle_offset_rad

    # Solve least-squares:  (J^T·J)^-1·J^T·b
    try:
        correction, _residuals, _rank, _sv = np.linalg.lstsq(J, b, rcond=None)
    except np.linalg.LinAlgError:
        return (0.0, 0.0, 0.0)

    return (
        float(correction[0]),
        float(correction[1]),
        float(correction[2]),
    )


def _compute_inlier_ratio(
    matches: list[MatchResult],
    correction: tuple[float, float, float],
    max_residual_m: float = 0.5,
    max_angle_residual_rad: float = np.deg2rad(10.0),
) -> float:
    """Fraction of matches whose residual is within tolerance after correction."""
    if not matches:
        return 0.0
    dx, dy, dtheta = correction
    inliers = 0
    for m in matches:
        nx, ny = _segment_normal(m.segment_start, m.segment_end)
        # Correction should satisfy n·(dx,dy) = -normal_offset_m.
        pos_residual = abs(m.normal_offset_m + (nx * dx + ny * dy))
        ang_residual = abs(m.angle_offset_rad + dtheta)
        if pos_residual <= max_residual_m and ang_residual <= max_angle_residual_rad:
            inliers += 1
    return inliers / len(matches)


# ---------------------------------------------------------------------------
# Rotation helpers (no tf2 dependency)
# ---------------------------------------------------------------------------


def _yaw_to_rotation(yaw_rad: float) -> np.ndarray:
    """ENU yaw (rotation about Z) to 3×3 rotation matrix."""
    c = np.cos(yaw_rad)
    s = np.sin(yaw_rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    """ENU yaw to quaternion (x, y, z, w)."""
    half = yaw_rad / 2.0
    return (0.0, 0.0, float(np.sin(half)), float(np.cos(half)))


# ---------------------------------------------------------------------------
# Main localizer entry point
# ---------------------------------------------------------------------------


# Re-export for convenience
from ed_uav_perception.boundary_extractor import ImageLine  # noqa: E402


def project_ray_to_ground(
    image_line: ImageLine,
    K: np.ndarray,
    R_cam_to_world: np.ndarray,
    t_cam_to_world: np.ndarray,
    altitude_m: float,
) -> Optional[GroundLine]:
    """Public alias for ``_image_line_to_ground``."""
    return _image_line_to_ground(image_line, K, R_cam_to_world, t_cam_to_world, altitude_m)


def compute_boundary_observation(
    ground_lines: list[GroundLine],
    profile_boundary_segments: list[tuple[str, tuple[float, float], tuple[float, float]]],
    imu_planar_pose: PlanarPose,
    imu_age_sec: float,
    timestamp_sec: float,
    timestamp_nanosec: int,
    frame_id: str,
    *,
    stale_imu_threshold_sec: float = 0.5,
    min_matches_for_full: int = 2,
    full_pose_min_angle_deg: float = 30.0,
    full_pose_min_inlier_ratio: float = 0.60,
    expected_line_count: int = 2,
    glare_line_threshold: int = 1,
    calibration_valid: bool = True,
) -> Optional["BoundaryObservation"]:
    """Compute a BoundaryObservation from projected ground lines and field profile.

    Args:
        ground_lines: Projected lines on the ground plane.
        profile_boundary_segments: Known boundary segments as
            ``(id, (start_x, start_y), (end_x, end_y))`` tuples.
        imu_planar_pose: Current IMU-estimated ENU pose.
        imu_age_sec: Age of the latest IMU attitude sample.
        timestamp_sec: Acquisition timestamp (seconds part).
        timestamp_nanosec: Acquisition timestamp (nanoseconds part).
        frame_id: ROS frame identifier for the observation header.
        stale_imu_threshold_sec: Maximum IMU age before rejection.
        min_matches_for_full: Minimum matched constraints for a full-pose
            observation (≥ 2 needed for X, Y, Yaw).
        full_pose_min_angle_deg: Minimum inter-constraint angle for full pose.
        full_pose_min_inlier_ratio: Minimum fraction of inlier matches.
        expected_line_count: Number of lines expected from the profile
            (used for glare detection).
        glare_line_threshold: If fewer than this many lines are detected
            while *expected_line_count* is higher → glare → no observation.
        calibration_valid: Whether camera calibration is available.

    Returns:
        A ``BoundaryObservation`` message, or ``None`` when conditions
        are insufficient for a valid observation.

    Rejection conditions (checked in order):
        1. Calibration missing.
        2. IMU age exceeds threshold.
        3. Glare detected (few lines but many expected).
        4. No matches after association.
        5. Ambiguous association (too many candidate matches vs. actual).
    """
    # Guard 1: calibration.
    if not calibration_valid:
        return None

    # Guard 2: stale IMU.
    if imu_age_sec > stale_imu_threshold_sec:
        return None

    # Guard 3: glare / low visibility.
    if len(ground_lines) < glare_line_threshold and expected_line_count > glare_line_threshold:
        return None

    # Guard 4: no ground lines at all.
    if not ground_lines:
        return None

    # --- Association ---
    matches = associate_lines(ground_lines, profile_boundary_segments)

    # Guard 5: ambiguous — too many candidate lines but few matched.
    if len(ground_lines) >= 3 and len(matches) < 2:
        return None

    if not matches:
        return None

    # --- Compute correction ---
    correction = _solve_planar_correction(matches)
    inlier_ratio = _compute_inlier_ratio(matches, correction)

    has_nonparallel = _has_nonparallel_constraints(matches, full_pose_min_angle_deg)

    # --- Determine observability ---
    if (
        has_nonparallel
        and len(matches) >= min_matches_for_full
        and inlier_ratio >= full_pose_min_inlier_ratio
    ):
        # Full planar pose (X, Y, Yaw).
        corrected_x = imu_planar_pose.x_m - correction[0]
        corrected_y = imu_planar_pose.y_m - correction[1]
        corrected_yaw = imu_planar_pose.yaw_rad - correction[2]
        observable_dof_mask = DOF_X | DOF_Y | DOF_YAW
    else:
        # Partial: single-line constraint → yaw only.
        # Position is the uncorrected IMU pose (unreliable without
        # a second nonparallel constraint).
        corrected_x = imu_planar_pose.x_m
        corrected_y = imu_planar_pose.y_m
        corrected_yaw = imu_planar_pose.yaw_rad - correction[2]
        observable_dof_mask = DOF_YAW

    # --- Build BoundaryObservation ---
    # We import here to keep the module importable for testing without
    # a live ROS environment (BoundaryObservation requires rclpy init).
    from ed_uav_interfaces.msg import BoundaryObservation  # type: ignore[import-not-found,unused-ignore]

    obs = BoundaryObservation()
    obs.header = Header()
    obs.header.stamp.sec = int(timestamp_sec)
    obs.header.stamp.nanosec = timestamp_nanosec
    obs.header.frame_id = frame_id
    obs.source_sequence = 0
    obs.observable_dof_mask = observable_dof_mask
    obs.constraint_count = len(matches)
    obs.confidence = float(inlier_ratio)
    obs.association_id = matches[0].segment_id if matches else ""

    # Pose
    qx, qy, qz, qw = _quaternion_from_yaw(corrected_yaw)
    obs.pose = PoseWithCovariance()
    obs.pose.pose = Pose(
        position=Point(x=corrected_x, y=corrected_y, z=0.0),
        orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
    )
    # Default covariance: 36 zeros (6×6).
    obs.pose.covariance = [0.0] * 36

    return obs


# ---------------------------------------------------------------------------
# Convenience: extract profile segments as (id, start, end) tuples
# ---------------------------------------------------------------------------


def segments_from_profile(
    profile: "KnownFieldProfile",  # ed_uav_localization.field_profile.model
) -> list[tuple[str, tuple[float, float], tuple[float, float]]]:
    """Convert a ``KnownFieldProfile`` into the segment-tuple list used by
    ``compute_boundary_observation``."""
    return [
        (
            seg.id,
            (float(seg.start.x_m), float(seg.start.y_m)),
            (float(seg.end.x_m), float(seg.end.y_m)),
        )
        for seg in profile.boundary_segments
    ]
