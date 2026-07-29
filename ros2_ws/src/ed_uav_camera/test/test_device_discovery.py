"""Stable V4L2 calibration-device enumeration tests."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

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
        return subprocess.CompletedProcess(arguments, 0, f"ID_SERIAL_SHORT={serial}\n", "")

    monkeypatch.setattr(subprocess, "run", fake_run)

    # When: calibration discovery enumerates stable capture endpoints.
    devices = enumerate_stable_video_devices(tmp_path)

    # Then: output is deterministic and excludes numeric identities from persistence.
    assert tuple(device.serial for device in devices) == ("NARROW-001", "WIDE-001")
    assert all(device.by_id.endswith("video-index0") for device in devices)
    assert all("/dev/video" not in device.by_id for device in devices)
