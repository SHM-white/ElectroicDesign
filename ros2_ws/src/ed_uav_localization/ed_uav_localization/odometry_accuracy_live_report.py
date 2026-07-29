"""Live odometry displacement reporting for the accuracy demo."""

from __future__ import annotations

from dataclasses import dataclass

from ed_uav_localization.odometry_accuracy import OdometrySample


@dataclass(frozen=True, slots=True)
class LiveSampleSummary:
    dx_m: float = 0.0
    dy_m: float = 0.0
    dz_m: float = 0.0
    xy_m: float = 0.0
    three_d_m: float = 0.0
    frame_id: str = ""
    age_sec: float = 0.0
    health: str = "waiting"


def live_sample_summary(first: OdometrySample, sample: OdometrySample, receipt_age_sec: float = 0.0) -> LiveSampleSummary:
    dx_m = sample.x_m - first.x_m
    dy_m = sample.y_m - first.y_m
    dz_m = sample.z_m - first.z_m
    age_sec = receipt_age_sec if receipt_age_sec > 0.0 else max(0.0, (sample.stamp_ns - first.stamp_ns) * 1e-9)
    return LiveSampleSummary(
        dx_m=dx_m,
        dy_m=dy_m,
        dz_m=dz_m,
        xy_m=(dx_m * dx_m + dy_m * dy_m) ** 0.5,
        three_d_m=(dx_m * dx_m + dy_m * dy_m + dz_m * dz_m) ** 0.5,
        frame_id=sample.frame_id,
        age_sec=age_sec,
        health="live",
    )
