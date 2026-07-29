"""Pure startup-relative odometry state and metrics."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ed_uav_localization.odometry_accuracy import (
    OdometrySample,
    OdometryValidationError,
    OdometryValidationIssue,
)


@dataclass(frozen=True, slots=True)
class OdometryOffset:
    """A single accepted pose expressed relative to the startup pose."""

    stamp_ns: int
    frame_id: str
    dx_m: float
    dy_m: float
    dz_m: float
    xy_distance_m: float
    distance_3d_m: float
    yaw_delta_rad: float


@dataclass(frozen=True, slots=True)
class StartupRelativeOdometry:
    """Immutable accepted odometry state with a startup pose fixed as origin."""

    origin: OdometrySample | None = None
    last_accepted: OdometrySample | None = None

    def accept(self, sample: OdometrySample) -> StartupRelativeOdometry:
        """Return the next accepted state or reject an invalid follow-up sample."""
        if self.origin is None:
            if self.last_accepted is not None:
                raise AssertionError("uninitialized origin cannot have an accepted sample")
            return StartupRelativeOdometry(origin=sample, last_accepted=sample)

        if self.last_accepted is None:
            raise AssertionError("initialized origin requires an accepted sample")
        if sample.frame_id != self.origin.frame_id:
            raise OdometryValidationError(OdometryValidationIssue.FRAME_CHANGED)
        if sample.stamp_ns <= self.last_accepted.stamp_ns:
            raise OdometryValidationError(OdometryValidationIssue.NON_INCREASING_STAMP)
        return StartupRelativeOdometry(origin=self.origin, last_accepted=sample)

    @property
    def offset(self) -> OdometryOffset | None:
        """Return the last accepted pose relative to the immutable startup origin."""
        if self.origin is None:
            return None
        if self.last_accepted is None:
            raise AssertionError("initialized origin requires an accepted sample")
        dx_m = self.last_accepted.x_m - self.origin.x_m
        dy_m = self.last_accepted.y_m - self.origin.y_m
        dz_m = self.last_accepted.z_m - self.origin.z_m
        yaw_delta_rad = self.last_accepted.yaw_rad - self.origin.yaw_rad
        return OdometryOffset(
            stamp_ns=self.last_accepted.stamp_ns,
            frame_id=self.origin.frame_id,
            dx_m=dx_m,
            dy_m=dy_m,
            dz_m=dz_m,
            xy_distance_m=math.hypot(dx_m, dy_m),
            distance_3d_m=math.hypot(dx_m, dy_m, dz_m),
            yaw_delta_rad=math.atan2(math.sin(yaw_delta_rad), math.cos(yaw_delta_rad)),
        )
