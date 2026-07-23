"""Stable UVC identity binding acceptance tests."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_camera.identity import (
    CameraBinding,
    CameraRole,
    DuplicateSerialError,
    ObservedCamera,
    SerialMismatchError,
    bind_observed_cameras,
)


def test_rejects_duplicate_expected_serials_before_device_open() -> None:
    # Given: two independent namespaces configured with one physical serial.
    bindings = (
        CameraBinding(CameraRole.NARROW, "SN-42", "/dev/v4l/by-id/narrow"),
        CameraBinding(CameraRole.WIDE, "SN-42", "/dev/v4l/by-id/wide"),
    )

    # When: the transport validates stable bindings.
    with pytest.raises(DuplicateSerialError, match="SN-42"):
        bind_observed_cameras(bindings, ())

    # Then: duplicate serials cannot cause both nodes to open one camera.


def test_rejects_observed_serial_mismatch_at_declared_by_id_path() -> None:
    # Given: the narrow by-id symlink resolves to a camera with another serial.
    bindings = (
        CameraBinding(CameraRole.NARROW, "NARROW-1", "/dev/v4l/by-id/narrow"),
    )
    observed = (
        ObservedCamera("WIDE-9", "/dev/v4l/by-id/narrow"),
    )

    # When: the transport binds the observed device.
    with pytest.raises(SerialMismatchError, match="NARROW-1"):
        bind_observed_cameras(bindings, observed)

    # Then: a swapped or stale symlink is refused before stream startup.
