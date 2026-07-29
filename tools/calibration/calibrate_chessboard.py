#!/usr/bin/env python3
"""Direct V4L2/OpenCV selected-camera chessboard calibration bootstrap."""

from __future__ import annotations

# How to run from the repository root:
# PYTHONPATH=ros2_ws/src/ed_uav_camera python3 tools/calibration/calibrate_chessboard.py --help

import argparse
import json
from pathlib import Path
import sys
import time

import cv2

from ed_uav_camera.calibration import CaptureProvenance
from ed_uav_camera.calibration_artifacts import write_artifacts
from ed_uav_camera.calibration_models import (
    ArtifactContext,
    BoardSpec,
    CalibrationBootstrapError,
    CalibrationSelection,
    CapturePolicy,
    CaptureSession,
)
from ed_uav_camera.chessboard_capture import capture_observations
from ed_uav_camera.chessboard_solver import solve_calibration
from ed_uav_camera.device_discovery import StableVideoDevice, enumerate_stable_video_devices
from ed_uav_camera.model import CameraRole


def main(arguments: list[str] | None = None) -> int:
    """Run the bootstrap and report generated paths and metrics as JSON."""
    parser = _parser()
    namespace = parser.parse_args(arguments)
    try:
        board = BoardSpec.parse(namespace.inner_corners, namespace.confirm_square_mm)
        selection, source, direct_v4l2 = _selection(namespace)
        observations = capture_observations(
            CaptureSession(source, selection, board, CapturePolicy(), direct_v4l2)
        )
        solution = solve_calibration(board, selection, observations)
        captured_at_ns = time.time_ns()
        provenance = (
            CaptureProvenance.DIRECT_V4L2
            if direct_v4l2
            else CaptureProvenance.RECORDED_VIDEO_FIXTURE
        )
        paths = write_artifacts(
            namespace.output_dir,
            solution,
            ArtifactContext(
                selection,
                board,
                source,
                provenance,
                captured_at_ns,
                namespace.valid_for_days * 86_400_000_000_000,
            ),
        )
    except (CalibrationBootstrapError, OSError, cv2.error) as error:
        print(str(error), file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "camera_info": str(paths.camera_info),
                "descriptor": str(paths.descriptor),
                "descriptor_file_sha256": str(paths.descriptor_file_sha256),
                "production_eligible": provenance is CaptureProvenance.DIRECT_V4L2,
                "holdout_mean_px": solution.metrics.holdout_mean_px,
                "holdout_max_px": solution.metrics.holdout_max_px,
            },
            sort_keys=True,
        )
    )
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Calibrate one narrow or wide camera from the physical 11-column x 8-row "
            "chessboard with measured 15.0-mm squares."
        )
    )
    parser.add_argument("--input-video", type=Path)
    parser.add_argument("--role", choices=tuple(role.value for role in CameraRole))
    parser.add_argument("--serial")
    parser.add_argument("--observed-serial")
    parser.add_argument("--by-id")
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--inner-corners", default="10x7")
    parser.add_argument("--confirm-square-mm", type=float, required=True)
    parser.add_argument("--valid-for-days", type=int, default=365)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser


def _selection(namespace: argparse.Namespace) -> tuple[CalibrationSelection, str, bool]:
    if namespace.valid_for_days <= 0:
        raise CalibrationBootstrapError("valid-for-days must be positive")
    if namespace.input_video is not None:
        required = {
            "role": namespace.role,
            "serial": namespace.serial,
            "observed-serial": namespace.observed_serial,
            "by-id": namespace.by_id,
        }
        missing = tuple(name for name, value in required.items() if value is None)
        if missing:
            raise CalibrationBootstrapError(
                f"fixture/video mode requires explicit provenance: {', '.join(missing)}"
            )
        role = CameraRole(namespace.role)
        selection = CalibrationSelection(
            role,
            namespace.serial,
            namespace.observed_serial,
            namespace.by_id,
            namespace.width,
            namespace.height,
        )
        return selection, str(namespace.input_video), False
    devices = enumerate_stable_video_devices()
    role = CameraRole(namespace.role) if namespace.role is not None else _choose_role()
    device = _choose_device(devices)
    if namespace.serial is not None and namespace.serial != device.serial:
        raise CalibrationBootstrapError(
            f"selected serial mismatch: expected {namespace.serial!r}, observed {device.serial!r}"
        )
    selection = CalibrationSelection(
        role,
        device.serial,
        device.serial,
        device.by_id,
        namespace.width,
        namespace.height,
    )
    return selection, device.by_id, True


def _choose_role() -> CameraRole:
    try:
        raw_role = input("Camera role [narrow/wide]: ").strip()
    except EOFError as error:
        raise CalibrationBootstrapError("camera role requires interactive input") from error
    try:
        return CameraRole(raw_role)
    except ValueError as error:
        raise CalibrationBootstrapError(f"unsupported camera role {raw_role!r}") from error


def _choose_device(devices: tuple[StableVideoDevice, ...]) -> StableVideoDevice:
    for index, device in enumerate(devices, start=1):
        print(f"{index}: {device.serial}  {device.by_id}")
    try:
        raw_index = input("Select camera number: ").strip()
    except EOFError as error:
        raise CalibrationBootstrapError("camera selection requires interactive input") from error
    try:
        index = int(raw_index) - 1
    except ValueError as error:
        raise CalibrationBootstrapError(f"invalid camera selection {raw_index!r}") from error
    if index < 0 or index >= len(devices):
        raise CalibrationBootstrapError(f"camera selection {raw_index!r} is out of range")
    return devices[index]


if __name__ == "__main__":
    raise SystemExit(main())
