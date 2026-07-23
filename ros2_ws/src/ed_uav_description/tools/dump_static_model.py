#!/usr/bin/env python3
"""Dump the fixed ED UAV model without opening hardware or ROS graph resources."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ed_uav_description.calibration import CalibrationError, load_calibration


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Dump fixed base_link sensor transforms.")
    parser.add_argument("--calibration", required=True, type=Path)
    arguments = parser.parse_args(argv)
    try:
        calibration = load_calibration(arguments.calibration)
    except CalibrationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"STATIC_MODEL: calibration_id={calibration.calibration_id} status={calibration.calibration_status}")
    for frame_name in (
        "fcu_link",
        "lidar_link",
        "camera_narrow_optical_frame",
        "camera_wide_optical_frame",
        "rangefinder_link",
    ):
        transform = calibration.transform_for(frame_name)
        print(
            f"STATIC_TF: base_link -> {frame_name} "
            f"xyz_m={transform.xyz_m} rpy_rad={transform.rpy_rad}"
        )
    return 0


raise SystemExit(main())
