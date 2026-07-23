"""Installed P09 profile catalog and launch-surface checks."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_camera.profiles import ProfileName, load_profile_catalog

CATALOG = PACKAGE_ROOT / "config" / "camera_profiles.yaml"
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "dual_uvc.launch.py"


def test_installed_catalog_has_only_candidate_profiles_for_calibration_and_live_roles() -> None:
    # Given: the package-owned non-hardware profile catalog.
    catalog = load_profile_catalog(CATALOG)

    # When: its named profile contract is read.
    profiles = {profile.name: profile for profile in catalog}

    # Then: full 5MP remains a low-rate candidate and live modes prefer MJPEG.
    assert set(profiles) == set(ProfileName)
    full_calibration = profiles[ProfileName.FULL_CALIBRATION]
    assert full_calibration.role is None
    assert all(candidate.frames_per_second <= 2 for candidate in full_calibration.candidates)
    assert profiles[ProfileName.NARROW_LIVE].candidates[0].fourcc == "MJPG"
    assert profiles[ProfileName.WIDE_LIVE].candidates[0].fourcc == "MJPG"


def test_launch_uses_v4l2_camera_info_manager_path_and_never_stereo_processing() -> None:
    # Given: the dual-camera ROS launch source.
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    # When: its concrete driver and provenance configuration are inspected.
    required_fragments = (
        'package="v4l2_camera"',
        'executable="v4l2_camera_node"',
        '"camera_info_url"',
        '"use_v4l2_buffer_timestamps": True',
        "respawn=True",
        'package="ed_uav_camera"',
        'executable="fake_image_device"',
    )

    # Then: both sources use the approved transport path and no stereo node exists.
    assert all(fragment in source for fragment in required_fragments)
    assert "stereo_image_proc" not in source
