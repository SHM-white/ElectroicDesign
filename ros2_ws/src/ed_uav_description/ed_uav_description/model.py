"""Render the static Xacro model from an already validated calibration."""

from __future__ import annotations

from pathlib import Path

import xacro

from ed_uav_description.calibration import Calibration


def _triple_text(values: tuple[float, float, float]) -> str:
    return " ".join(format(value, ".17g") for value in values)


def render_robot_description(calibration: Calibration, xacro_path: Path) -> str:
    mappings: dict[str, str] = {}
    for frame_name, argument_prefix in (
        ("fcu_link", "fcu"),
        ("lidar_link", "lidar"),
        ("camera_narrow_optical_frame", "camera_narrow"),
        ("camera_wide_optical_frame", "camera_wide"),
        ("rangefinder_link", "rangefinder"),
    ):
        transform = calibration.transform_for(frame_name)
        mappings[f"{argument_prefix}_xyz"] = _triple_text(transform.xyz_m)
        mappings[f"{argument_prefix}_rpy"] = _triple_text(transform.rpy_rad)
    return xacro.process_file(str(xacro_path), mappings=mappings).toxml()
