from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.field_profile import anchor, loader
from test_field_profile import VALID_PROFILE


@pytest.mark.parametrize("heading_rad", [0.0, math.pi / 2.0, math.pi])
def test_initializes_map_to_odom_for_cardinal_takeoff_headings(heading_rad: float) -> None:
    # Given: a known odom pose at takeoff and a commanded map-frame heading.
    profile = loader.load_profile_text(
        VALID_PROFILE.replace("commanded_heading_rad: 0.6", f"commanded_heading_rad: {heading_rad}"),
        "cardinal.yaml",
    )
    odom_to_base = anchor.Pose2D(x_m=4.0, y_m=-2.0, yaw_rad=0.25)

    # When: the fresh session's map anchor is initialized.
    map_to_odom = anchor.initialize_map_to_odom(profile, odom_to_base)
    mapped_takeoff = map_to_odom.apply(odom_to_base)

    # Then: map origin and heading match the profile without changing continuous odom.
    assert mapped_takeoff.x_m == pytest.approx(10.0)
    assert mapped_takeoff.y_m == pytest.approx(-1.0)
    assert mapped_takeoff.yaw_rad == pytest.approx(heading_rad)
    assert odom_to_base == anchor.Pose2D(x_m=4.0, y_m=-2.0, yaw_rad=0.25)


def test_initializes_map_to_odom_for_arbitrary_heading() -> None:
    # Given: a rotated profile and a nonzero odom pose at takeoff.
    profile = loader.load_profile_text(VALID_PROFILE, "arbitrary.yaml")
    odom_to_base = anchor.Pose2D(x_m=-0.5, y_m=1.25, yaw_rad=-0.4)

    # When: the anchor is derived from the two known poses.
    map_to_odom = anchor.initialize_map_to_odom(profile, odom_to_base)

    # Then: composition lands exactly at the commanded map-frame takeoff pose.
    mapped_takeoff = map_to_odom.apply(odom_to_base)
    assert mapped_takeoff.x_m == pytest.approx(10.0)
    assert mapped_takeoff.y_m == pytest.approx(-1.0)
    assert mapped_takeoff.yaw_rad == pytest.approx(0.6)
