"""Pure per-session initialization of the map-to-odom field anchor."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ed_uav_localization.field_profile.model import KnownFieldProfile


@dataclass(frozen=True, slots=True)
class Pose2D:
    """A planar SI/ENU pose with yaw measured in radians."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True, slots=True)
class MapToOdomTransform:
    """The rigid transform owned solely by the field-anchor publisher."""

    x_m: float
    y_m: float
    yaw_rad: float

    def apply(self, odom_pose: Pose2D) -> Pose2D:
        """Compose the anchor with an odom-frame pose without mutating that pose."""
        cosine = math.cos(self.yaw_rad)
        sine = math.sin(self.yaw_rad)
        return Pose2D(
            x_m=self.x_m + cosine * odom_pose.x_m - sine * odom_pose.y_m,
            y_m=self.y_m + sine * odom_pose.x_m + cosine * odom_pose.y_m,
            yaw_rad=_normalize_angle(self.yaw_rad + odom_pose.yaw_rad),
        )


def initialize_map_to_odom(
    profile: KnownFieldProfile, odom_to_base_at_takeoff: Pose2D
) -> MapToOdomTransform:
    """Derive map->odom from the profile takeoff pose and current odom pose.

    The caller continues publishing the supplied odom->base pose unchanged. Future
    boundary corrections can update only this transform, preserving odometry continuity.
    """
    map_yaw_from_odom = _normalize_angle(
        profile.takeoff.commanded_heading_rad - odom_to_base_at_takeoff.yaw_rad
    )
    cosine = math.cos(map_yaw_from_odom)
    sine = math.sin(map_yaw_from_odom)
    return MapToOdomTransform(
        x_m=profile.takeoff.origin.x_m
        - cosine * odom_to_base_at_takeoff.x_m
        + sine * odom_to_base_at_takeoff.y_m,
        y_m=profile.takeoff.origin.y_m
        - sine * odom_to_base_at_takeoff.x_m
        - cosine * odom_to_base_at_takeoff.y_m,
        yaw_rad=map_yaw_from_odom,
    )


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))
