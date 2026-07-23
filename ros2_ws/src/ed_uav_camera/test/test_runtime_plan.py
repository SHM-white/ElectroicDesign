"""Dual-camera runtime-plan validation tests."""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_camera.profiles import JsonObject, JsonValue, UnsupportedProfileError
from ed_uav_camera.runtime_plan import load_runtime_plan


def test_accepts_capability_probed_dual_plan_with_independent_namespaces(tmp_path: Path) -> None:
    # Given: a catalog and plan whose modes were explicitly reported by a fake probe.
    catalog = tmp_path / "profiles.json"
    plan = tmp_path / "runtime-plan.json"
    catalog.write_text(json.dumps(profile_catalog()), encoding="utf-8")
    plan.write_text(json.dumps(runtime_plan()), encoding="utf-8")

    # When: launch preflight validates the complete transport plan.
    validated = load_runtime_plan(plan, catalog, now_ns=100)

    # Then: both P03 namespace/frame contracts are ready for independent nodes.
    assert tuple(camera.binding.role.value for camera in validated.cameras) == ("narrow", "wide")
    assert validated.cameras[0].image_topic == "/camera/narrow/image_raw"
    assert validated.cameras[1].camera_info_topic == "/camera/wide/camera_info"


def test_rejects_selected_mode_that_was_not_declared_by_profile_catalog(tmp_path: Path) -> None:
    # Given: a runtime plan claiming an unsupported 4K wide-live mode.
    catalog = tmp_path / "profiles.json"
    plan = tmp_path / "runtime-plan.json"
    catalog.write_text(json.dumps(profile_catalog()), encoding="utf-8")
    invalid_plan = runtime_plan()
    cameras = invalid_plan["cameras"]
    assert isinstance(cameras, list)
    wide = cameras[1]
    assert isinstance(wide, dict)
    mode = wide["mode"]
    assert isinstance(mode, dict)
    mode.update({"width": 3840, "height": 2160, "frames_per_second": 30})
    plan.write_text(json.dumps(invalid_plan), encoding="utf-8")

    # When: preflight matches the reported selection to the approved candidates.
    with pytest.raises(UnsupportedProfileError, match="wide_live"):
        load_runtime_plan(plan, catalog, now_ns=100)

    # Then: a mode cannot be silently treated as supported hardware.


def profile_catalog() -> JsonObject:
    return {
        "profiles": [
            {
                "name": "narrow_live",
                "role": "narrow",
                "candidates": [
                    {
                        "fourcc": "MJPG",
                        "width": 1280,
                        "height": 720,
                        "frames_per_second": 20,
                        "compression": "mjpeg",
                        "declared_peak_mbit_s": 64.0,
                    }
                ],
            },
            {
                "name": "wide_live",
                "role": "wide",
                "candidates": [
                    {
                        "fourcc": "MJPG",
                        "width": 1280,
                        "height": 720,
                        "frames_per_second": 15,
                        "compression": "mjpeg",
                        "declared_peak_mbit_s": 48.0,
                    }
                ],
            },
        ]
    }


def runtime_plan() -> JsonObject:
    return {
        "controller_budget_mbit_s": 384.0,
        "cameras": [
            camera_plan(
                FakeCameraDefinition(
                    "narrow",
                    "NARROW-FAKE",
                    "narrow_live",
                    20,
                    64.0,
                    "camera_narrow_optical_frame",
                )
            ),
            camera_plan(
                FakeCameraDefinition(
                    "wide",
                    "WIDE-FAKE",
                    "wide_live",
                    15,
                    48.0,
                    "camera_wide_optical_frame",
                )
            ),
        ],
    }


@dataclass(frozen=True, slots=True)
class FakeCameraDefinition:
    role: str
    serial: str
    profile: str
    frames_per_second: int
    declared_peak_mbit_s: float
    frame_id: str


def camera_plan(camera: FakeCameraDefinition) -> JsonObject:
    return {
        "role": camera.role,
        "serial": camera.serial,
        "by_id": f"/dev/v4l/by-id/{camera.role}-fake",
        "observed_serial": camera.serial,
        "controller_id": "fake-usb-controller",
        "profile": camera.profile,
        "frame_id": camera.frame_id,
        "mode": {
            "fourcc": "MJPG",
            "width": 1280,
            "height": 720,
            "frames_per_second": camera.frames_per_second,
            "compression": "mjpeg",
            "declared_peak_mbit_s": camera.declared_peak_mbit_s,
        },
        "calibration": {
            "serial": camera.serial,
            "width": 1280,
            "height": 720,
            "captured_at_ns": 0,
            "valid_for_ns": 1_000,
            "camera_info_url": f"file:///{camera.role}-fake.yaml",
        },
    }
