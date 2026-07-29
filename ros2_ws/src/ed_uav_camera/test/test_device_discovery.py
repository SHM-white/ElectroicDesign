"""Stable V4L2 calibration-device enumeration tests."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_camera.calibration_models import CalibrationBootstrapError
from ed_uav_camera.device_discovery import enumerate_stable_video_devices


def test_enumerates_sorted_index_zero_by_id_devices_with_udev_serials(
    tmp_path: Path, monkeypatch,
) -> None:
    # Given: two stable capture symlinks plus one non-capture interface.
    video0 = tmp_path / "video0"
    video1 = tmp_path / "video1"
    video0.touch()
    video1.touch()
    (tmp_path / "usb-wide-video-index0").symlink_to(video1)
    (tmp_path / "usb-narrow-video-index0").symlink_to(video0)
    (tmp_path / "usb-narrow-video-index1").symlink_to(video0)

    def fake_run(arguments, **_kwargs) -> subprocess.CompletedProcess[str]:
        by_id = str(arguments[-1])
        serial = "NARROW-001" if "narrow" in by_id else "WIDE-001"
        properties = (
            f"ID_SERIAL_SHORT={serial}\n"
            "ID_VENDOR_ID=0AC8\n"
            "ID_MODEL_ID=3460\n"
            "ID_REVISION=0122\n"
        )
        return subprocess.CompletedProcess(arguments, 0, properties, "")

    monkeypatch.setattr(subprocess, "run", fake_run)

    # When: calibration discovery enumerates stable capture endpoints.
    devices = enumerate_stable_video_devices(tmp_path)

    # Then: output is deterministic and excludes numeric identities from persistence.
    assert tuple(device.serial for device in devices) == ("NARROW-001", "WIDE-001")
    assert all(device.by_id.endswith("video-index0") for device in devices)
    assert all("/dev/video" not in device.by_id for device in devices)


def test_uses_lowercase_usb_revision_identity_when_serial_short_is_absent(
    tmp_path: Path, monkeypatch,
) -> None:
    # Given: one stable capture endpoint whose UVC device has no short serial.
    video0 = tmp_path / "video0"
    video0.touch()
    by_id = tmp_path / "usb-w19-video-index0"
    by_id.symlink_to(video0)

    def fake_run(arguments, **_kwargs) -> subprocess.CompletedProcess[str]:
        properties = (
            "ID_SERIAL=PORT_DEPENDENT_VALUE\n"
            "ID_VENDOR_ID=0AC8\n"
            "ID_MODEL_ID=3460\n"
            "ID_REVISION=0122\n"
        )
        return subprocess.CompletedProcess(arguments, 0, properties, "")

    monkeypatch.setattr(subprocess, "run", fake_run)

    # When: calibration discovery enumerates the serialless camera.
    devices = enumerate_stable_video_devices(tmp_path)

    # Then: callers receive the revision-derived identity through the existing schema.
    assert devices[0].serial == "usb-revision:0ac8:3460:0122"
    assert devices[0].by_id == str(by_id)


@pytest.mark.parametrize("missing_property", ["ID_VENDOR_ID", "ID_MODEL_ID", "ID_REVISION"])
def test_fails_when_serialless_camera_lacks_usb_revision_identity_component(
    tmp_path: Path, monkeypatch, missing_property: str,
) -> None:
    # Given: a serialless stable camera missing one required USB identity component.
    video0 = tmp_path / "video0"
    video0.touch()
    (tmp_path / "usb-w19-video-index0").symlink_to(video0)
    properties = {
        "ID_VENDOR_ID": "0ac8",
        "ID_MODEL_ID": "3460",
        "ID_REVISION": "0122",
    }
    del properties[missing_property]

    def fake_run(arguments, **_kwargs) -> subprocess.CompletedProcess[str]:
        output = "".join(f"{key}={value}\n" for key, value in properties.items())
        return subprocess.CompletedProcess(arguments, 0, output, "")

    monkeypatch.setattr(subprocess, "run", fake_run)

    # When/Then: discovery fails closed and names the unavailable component.
    with pytest.raises(CalibrationBootstrapError, match=missing_property):
        enumerate_stable_video_devices(tmp_path)


def test_rejects_duplicate_usb_revision_identities(tmp_path: Path, monkeypatch) -> None:
    # Given: two stable endpoints deriving the same effective camera identity.
    video0 = tmp_path / "video0"
    video1 = tmp_path / "video1"
    video0.touch()
    video1.touch()
    (tmp_path / "usb-narrow-video-index0").symlink_to(video0)
    (tmp_path / "usb-wide-video-index0").symlink_to(video1)

    def fake_run(arguments, **_kwargs) -> subprocess.CompletedProcess[str]:
        properties = "ID_VENDOR_ID=0ac8\nID_MODEL_ID=3460\nID_REVISION=0122\n"
        return subprocess.CompletedProcess(arguments, 0, properties, "")

    monkeypatch.setattr(subprocess, "run", fake_run)

    # When/Then: duplicate effective identities remain an explicit hard failure.
    with pytest.raises(
        CalibrationBootstrapError,
        match="stable V4L2 enumeration contains duplicate serials",
    ):
        enumerate_stable_video_devices(tmp_path)
