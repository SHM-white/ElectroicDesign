from __future__ import annotations

import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.odometry_accuracy import OdometrySample
from ed_uav_localization.odometry_accuracy_live_report import live_sample_summary


def test_live_sample_summary_reports_displacement_and_age() -> None:
    first = OdometrySample(stamp_ns=10, frame_id="odom", x_m=1.0, y_m=2.0, z_m=3.0, yaw_rad=0.0)
    sample = OdometrySample(stamp_ns=35, frame_id="odom", x_m=4.0, y_m=6.0, z_m=8.0, yaw_rad=0.0)

    summary = live_sample_summary(first, sample)

    assert summary.dx_m == 3.0
    assert summary.dy_m == 4.0
    assert summary.dz_m == 5.0
    assert summary.xy_m == 5.0
    assert summary.three_d_m == 7.0710678118654755
    assert summary.frame_id == "odom"
    assert summary.age_sec == pytest.approx(2.5e-08)
    assert summary.health == "live"
