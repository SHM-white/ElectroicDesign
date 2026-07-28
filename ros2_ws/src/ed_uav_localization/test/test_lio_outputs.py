from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


@dataclass
class Header:
    frame_id: str


@dataclass
class Cloud:
    header: Header


@dataclass
class PoseStamped:
    header: Header


@dataclass
class PathMessage:
    header: Header
    poses: list[PoseStamped]


def test_canonicalize_cloud_when_fast_lio_uses_camera_init_deep_copies_odom_frame() -> None:
    from ed_uav_localization.lio_outputs import canonicalize_cloud

    raw = Cloud(header=Header(frame_id="camera_init"))

    canonical = canonicalize_cloud(raw)

    assert canonical is not raw
    assert canonical.header is not raw.header
    assert canonical.header.frame_id == "odom"
    assert raw.header.frame_id == "camera_init"


def test_canonicalize_path_when_fast_lio_uses_camera_init_normalizes_each_pose_header() -> None:
    from ed_uav_localization.lio_outputs import canonicalize_path

    raw = PathMessage(
        header=Header(frame_id="camera_init"),
        poses=[PoseStamped(header=Header(frame_id="camera_init"))],
    )

    canonical = canonicalize_path(raw)

    assert canonical is not raw
    assert canonical.header.frame_id == "odom"
    assert canonical.poses[0] is not raw.poses[0]
    assert canonical.poses[0].header.frame_id == "odom"
    assert raw.poses[0].header.frame_id == "camera_init"
