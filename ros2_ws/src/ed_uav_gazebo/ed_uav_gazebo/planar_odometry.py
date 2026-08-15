"""Pure helpers for planar FAST-LIO odometry with an independent altitude owner."""

from __future__ import annotations

import math
from collections.abc import Sequence


def yaw_only_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    """Discard roll/pitch while preserving FAST-LIO yaw."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("orientation contains non-finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError("orientation quaternion has zero norm")
    x, y, z, w = (value / norm for value in values)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def bounded_planar_covariance(
    covariance: Sequence[float],
    *,
    altitude_variance: float = 0.0025,
) -> list[float]:
    """Keep x/y/yaw uncertainty and explicitly de-weight unobservable roll/pitch."""
    if len(covariance) != 36:
        raise ValueError("odometry covariance must contain 36 values")
    result = [float(value) if math.isfinite(value) else 0.0 for value in covariance]
    for index in range(36):
        result[index] = max(-100.0, min(100.0, result[index]))
    for axis in (0, 1, 5):
        diagonal = axis * 6 + axis
        result[diagonal] = max(1e-6, min(100.0, abs(result[diagonal])))
    # Z is owned by simulation altitude. Roll and pitch are intentionally not claimed.
    for axis in (2, 3, 4):
        for other in range(6):
            result[axis * 6 + other] = 0.0
            result[other * 6 + axis] = 0.0
    result[14] = max(1e-6, min(1.0, altitude_variance))
    result[21] = 1.0
    result[28] = 1.0
    return result


def continuous_altitude(
    previous_m: float | None,
    candidate_m: float,
    elapsed_s: float,
    maximum_rate_m_s: float,
) -> float:
    """Rate-limit simulator teleports without filtering normal vertical flight."""
    if not math.isfinite(candidate_m):
        raise ValueError("altitude is non-finite")
    if previous_m is None or elapsed_s <= 0.0:
        return candidate_m
    limit = max(0.01, maximum_rate_m_s) * elapsed_s
    return previous_m + max(-limit, min(limit, candidate_m - previous_m))
