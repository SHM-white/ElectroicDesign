"""Camera acquisition provenance, jitter, and recovery tests."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_camera.health import CameraHealth, HealthCode, InvalidHealthPeriodError
from ed_uav_camera.identity import CameraRole


def test_rejects_nonmonotonic_acquisition_stamp_without_reusing_frame() -> None:
    # Given: one accepted image acquisition timestamp.
    health = CameraHealth(CameraRole.NARROW, expected_period_ns=50_000_000)
    health.record_frame(acquisition_stamp_ns=1_000, observed_steady_ns=10_000)

    # When: the source supplies an older acquisition timestamp.
    accepted = health.record_frame(acquisition_stamp_ns=999, observed_steady_ns=20_000)

    # Then: the frame is rejected and the diagnostic is explicit.
    report = health.snapshot(now_steady_ns=20_000, stale_after_ns=100_000)
    assert accepted is False
    assert report.code is HealthCode.NONMONOTONIC_STAMP
    assert report.accepted_frames == 1
    assert report.rejected_nonmonotonic_frames == 1


def test_wide_unplug_does_not_degrade_narrow_camera_health() -> None:
    # Given: both monocular streams have independent health accumulators.
    narrow = CameraHealth(CameraRole.NARROW, expected_period_ns=50)
    wide = CameraHealth(CameraRole.WIDE, expected_period_ns=50)
    narrow.record_frame(acquisition_stamp_ns=100, observed_steady_ns=100)
    wide.record_frame(acquisition_stamp_ns=100, observed_steady_ns=100)
    wide.mark_unplugged(observed_steady_ns=125)
    narrow.record_frame(acquisition_stamp_ns=150, observed_steady_ns=150)

    # When: status is evaluated during the wide-camera outage.
    narrow_report = narrow.snapshot(now_steady_ns=151, stale_after_ns=100)
    wide_report = wide.snapshot(now_steady_ns=151, stale_after_ns=100)

    # Then: narrow remains live while only wide is unavailable.
    assert narrow_report.code is HealthCode.HEALTHY
    assert wide_report.code is HealthCode.UNAVAILABLE


def test_reports_jitter_and_inferred_drop_from_source_time() -> None:
    # Given: a stream expected every 50 milliseconds.
    health = CameraHealth(CameraRole.WIDE, expected_period_ns=50)
    health.record_frame(acquisition_stamp_ns=100, observed_steady_ns=100)

    # When: the next camera acquisition arrives 150 milliseconds later.
    health.record_frame(acquisition_stamp_ns=250, observed_steady_ns=250)

    # Then: diagnostics report a two-frame inferred drop and 100 ms jitter.
    report = health.snapshot(now_steady_ns=250, stale_after_ns=100)
    assert report.inferred_drops == 2
    assert report.max_jitter_ns == 100


def test_restart_recovers_only_the_unplugged_camera() -> None:
    # Given: a wide stream that was unavailable after a disconnect.
    health = CameraHealth(CameraRole.WIDE, expected_period_ns=50)
    health.mark_unplugged(observed_steady_ns=100)

    # When: its supervised driver restarts and provides a fresh frame.
    health.mark_restarted(observed_steady_ns=150)
    health.record_frame(acquisition_stamp_ns=200, observed_steady_ns=200)

    # Then: recovery is visible without losing the recorded restart count.
    report = health.snapshot(now_steady_ns=201, stale_after_ns=100)
    assert report.code is HealthCode.RECOVERED
    assert report.restart_count == 1


def test_rejects_nonpositive_health_period_at_configuration_boundary() -> None:
    # Given: a malformed expected frame period from launch configuration.
    # When: the health accumulator is constructed.
    try:
        CameraHealth(CameraRole.NARROW, expected_period_ns=0)
    except InvalidHealthPeriodError:
        rejected = True
    else:
        rejected = False

    # Then: invalid timing configuration cannot create misleading diagnostics.
    assert rejected is True
