"""USB2 candidate-profile selection and controller-budget tests."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_camera.profiles import (
    CameraMode,
    CameraProfile,
    CameraRole,
    Compression,
    ControllerAssignment,
    ControllerBudgetError,
    MalformedProfileError,
    ProfileName,
    evaluate_controller_budget,
    parse_profile_catalog,
    select_supported_mode,
)


def test_rejects_uncompressed_mode_that_exceeds_usb2_controller_budget() -> None:
    # Given: a requested 1080p30 YUYV mode declared on one USB2 controller.
    mode = CameraMode(
        fourcc="YUYV",
        width=1920,
        height=1080,
        frames_per_second=30,
        compression=Compression.UNCOMPRESSED,
        bits_per_pixel=16,
        declared_peak_mbit_s=None,
    )

    # When: the controller budget evaluates the uncompressed payload.
    with pytest.raises(ControllerBudgetError, match="480"):
        evaluate_controller_budget(
            (ControllerAssignment("usb-0000:00:14.0", CameraRole.WIDE, mode),),
            budget_mbit_s=480,
        )

    # Then: a nominal UVC request cannot overbook one USB2 controller.


def test_counts_two_hubs_on_one_controller_as_one_shared_budget() -> None:
    # Given: narrow and wide cameras behind separate hubs on one controller.
    mode = CameraMode(
        fourcc="YUYV",
        width=1280,
        height=720,
        frames_per_second=30,
        compression=Compression.UNCOMPRESSED,
        bits_per_pixel=16,
        declared_peak_mbit_s=None,
    )
    assignments = (
        ControllerAssignment("usb-0000:00:14.0", CameraRole.NARROW, mode),
        ControllerAssignment("usb-0000:00:14.0", CameraRole.WIDE, mode),
    )

    # When: the planner totals each controller rather than each hub.
    with pytest.raises(ControllerBudgetError, match="usb-0000:00:14.0"):
        evaluate_controller_budget(assignments, budget_mbit_s=480)

    # Then: separate hubs do not create extra USB2 bandwidth.


def test_prefers_declared_mjpeg_candidate_only_when_capability_probe_reports_it() -> None:
    # Given: a wide-live profile ordered MJPEG first, with a YUYV fallback.
    mjpeg = CameraMode("MJPG", 1280, 720, 15, Compression.MJPEG, None, 48.0)
    yuyv = CameraMode("YUYV", 640, 480, 15, Compression.UNCOMPRESSED, 16, None)
    profile = CameraProfile(ProfileName.WIDE_LIVE, CameraRole.WIDE, (mjpeg, yuyv))

    # When: the capability probe reports only the fallback mode.
    selected = select_supported_mode(profile, (yuyv,))

    # Then: the transport negotiates the supported reduced mode, not a guess.
    assert selected == yuyv


def test_rejects_malformed_profile_catalog_before_driver_launch() -> None:
    # Given: untrusted profile data with an unsupported compression label.
    malformed_catalog = {
        "profiles": [
            {
                "name": "wide_live",
                "role": "wide",
                "candidates": [
                    {
                        "fourcc": "MJPG",
                        "width": 1280,
                        "height": 720,
                        "frames_per_second": 15,
                        "compression": "made_up",
                        "declared_peak_mbit_s": 48.0,
                    }
                ],
            }
        ]
    }

    # When: the catalog parser crosses the configuration boundary.
    with pytest.raises(MalformedProfileError, match="compression"):
        parse_profile_catalog(malformed_catalog)

    # Then: malformed format declarations never reach a V4L2 driver.
