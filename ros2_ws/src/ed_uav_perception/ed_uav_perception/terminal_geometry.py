"""PnP / homography pose estimation for calibrated terminal targets."""

from __future__ import annotations

import numpy as np

try:
    import cv2  # type: ignore[import-untyped]

    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion


# ---------------------------------------------------------------------------
# Rotation matrix ↔ quaternion conversion (no tf2 dependency)
# ---------------------------------------------------------------------------


def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3×3 rotation matrix to a unit quaternion (x, y, z, w)."""
    trace = float(np.trace(R))
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


# ---------------------------------------------------------------------------
# Core PnP solver
# ---------------------------------------------------------------------------


def target_to_pose(
    image_points: np.ndarray,
    object_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray | None = None,
    frame_id: str = "camera_optical",
) -> PoseWithCovarianceStamped | None:
    """Solve camera pose from 4+ known 2D→3D point correspondences via PnP.

    Args:
        image_points:  N×2 array of pixel coordinates  (u, v).
        object_points: N×3 array of world coordinates  (X, Y, Z).
        camera_matrix: 3×3 intrinsic matrix K.
        dist_coeffs:   Optional distortion coefficients (4×1, 5×1, or 8×1).
        frame_id:      Frame ID attached to the returned pose header.

    Returns:
        PoseWithCovarianceStamped giving the camera pose in the object's
        world frame, or ``None`` when the input is insufficient or PnP fails.

    Rejection conditions:
        * Fewer than 4 point correspondences.
        * Missing or malformed camera matrix.
        * PnP solver fails to converge or raises an exception.
        * cv2 is not available.
    """
    if not _HAS_CV2:
        return None

    if image_points is None or object_points is None:
        return None
    if image_points.shape[0] < 4 or object_points.shape[0] < 4:
        return None
    if camera_matrix is None or camera_matrix.shape != (3, 3):
        return None

    if dist_coeffs is None:
        dist_coeffs = np.zeros((4, 1), dtype=np.float64)

    try:
        success, rvec, tvec = cv2.solvePnP(
            object_points.astype(np.float64),
            image_points.astype(np.float64),
            camera_matrix.astype(np.float64),
            dist_coeffs.astype(np.float64),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            return None
    except cv2.error:
        return None

    # PnP returns world→camera transform.
    # Invert to camera→world (the "pose" of the camera in the world frame).
    R_w2c, _ = cv2.Rodrigues(rvec)
    R_c2w = R_w2c.T
    t_c2w = -R_c2w @ tvec.reshape(3, 1)

    qx, qy, qz, qw = _rotation_matrix_to_quaternion(R_c2w)

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = frame_id
    pose.pose.pose = Pose(
        position=Point(x=float(t_c2w[0, 0]), y=float(t_c2w[1, 0]), z=float(t_c2w[2, 0])),
        orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
    )
    return pose
