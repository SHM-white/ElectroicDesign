"""Camera-info serial, resolution, and freshness gates."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_camera.calibration import (
    CalibrationDescriptor,
    CalibrationResolutionMismatchError,
    StaleCalibrationError,
    validate_calibration,
)
from ed_uav_camera.identity import CameraBinding, CameraRole
from ed_uav_camera.profiles import CameraMode, Compression


def test_rejects_calibration_with_resolution_mismatch() -> None:
    # Given: a serial-bound calibration recorded for 640x480.
    binding = CameraBinding(CameraRole.NARROW, "NARROW-1", "/dev/v4l/by-id/narrow")
    calibration = CalibrationDescriptor("NARROW-1", 640, 480, 100, 1_000, "file:///cal.yaml")
    mode = CameraMode("MJPG", 1280, 720, 15, Compression.MJPEG, None, 48.0)

    # When: a negotiated stream requests another image resolution.
    with pytest.raises(CalibrationResolutionMismatchError, match="1280x720"):
        validate_calibration(binding, calibration, mode, now_ns=200)

    # Then: matching camera_info cannot be published for a different raster.


def test_rejects_stale_calibration_before_transport_activation() -> None:
    # Given: a calibration validity window that ended in the past.
    binding = CameraBinding(CameraRole.WIDE, "WIDE-1", "/dev/v4l/by-id/wide")
    calibration = CalibrationDescriptor("WIDE-1", 1280, 720, 100, 50, "file:///wide.yaml")
    mode = CameraMode("MJPG", 1280, 720, 15, Compression.MJPEG, None, 48.0)

    # When: the launch preflight evaluates calibration freshness.
    with pytest.raises(StaleCalibrationError, match="WIDE-1"):
        validate_calibration(binding, calibration, mode, now_ns=151)

    # Then: a stale calibration cannot activate a camera stream.
