from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import dataclass
from typing import Protocol, TypeVar

from ed_uav_localization.odometry_math import (
    Quaternion,
    RigidTransform,
    Vector3,
    covariance_matrix,
    covariance_values,
    transform_pose_covariance,
    transform_twist,
    transform_twist_covariance,
)


class Stamp(Protocol):
    sec: int
    nanosec: int


class Header(Protocol):
    stamp: Stamp
    frame_id: str


class Position(Protocol):
    x: float
    y: float
    z: float


class Orientation(Protocol):
    x: float
    y: float
    z: float
    w: float


class Pose(Protocol):
    position: Position
    orientation: Orientation


class PoseWithCovariance(Protocol):
    pose: Pose
    covariance: list[float]


class Twist(Protocol):
    linear: Position
    angular: Position


class TwistWithCovariance(Protocol):
    twist: Twist
    covariance: list[float]


class OdometryMessage(Protocol):
    header: Header
    child_frame_id: str
    pose: PoseWithCovariance
    twist: TwistWithCovariance


Odometry = TypeVar("Odometry", bound=OdometryMessage)


@dataclass(frozen=True, slots=True)
class OdomToBaseTransform:
    stamp: Stamp
    parent_frame: str
    child_frame: str
    translation_x: float
    translation_y: float
    translation_z: float
    rotation_x: float
    rotation_y: float
    rotation_z: float
    rotation_w: float


def normalize_odometry(
    message: Odometry, base_to_lidar: RigidTransform
) -> Odometry | None:
    """Convert FAST-LIO odometry from lidar to canonical base-link frames."""
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    pose_covariance = covariance_matrix(message.pose.covariance)
    twist_covariance = covariance_matrix(message.twist.covariance)
    if not all(
        math.isfinite(value)
        for value in (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
    ) or pose_covariance is None or twist_covariance is None:
        return None
    normalized = deepcopy(message)
    odom_to_lidar = RigidTransform(
        translation=Vector3(x=position.x, y=position.y, z=position.z),
        rotation=Quaternion(
            x=orientation.x,
            y=orientation.y,
            z=orientation.z,
            w=orientation.w,
        ),
    )
    odom_to_base = odom_to_lidar.compose(base_to_lidar.inverse())
    linear_base, angular_base = transform_twist(
        base_to_lidar,
        Vector3(
            x=message.twist.twist.linear.x,
            y=message.twist.twist.linear.y,
            z=message.twist.twist.linear.z,
        ),
        Vector3(
            x=message.twist.twist.angular.x,
            y=message.twist.twist.angular.y,
            z=message.twist.twist.angular.z,
        ),
    )
    normalized.header.frame_id = "odom"
    normalized.child_frame_id = "base_link"
    normalized.pose.pose.position.x = odom_to_base.translation.x
    normalized.pose.pose.position.y = odom_to_base.translation.y
    normalized.pose.pose.position.z = odom_to_base.translation.z
    normalized.pose.pose.orientation.x = odom_to_base.rotation.x
    normalized.pose.pose.orientation.y = odom_to_base.rotation.y
    normalized.pose.pose.orientation.z = odom_to_base.rotation.z
    normalized.pose.pose.orientation.w = odom_to_base.rotation.w
    normalized.twist.twist.linear.x = linear_base.x
    normalized.twist.twist.linear.y = linear_base.y
    normalized.twist.twist.linear.z = linear_base.z
    normalized.twist.twist.angular.x = angular_base.x
    normalized.twist.twist.angular.y = angular_base.y
    normalized.twist.twist.angular.z = angular_base.z
    normalized.pose.covariance = covariance_values(
        transform_pose_covariance(pose_covariance, odom_to_base, base_to_lidar)
    )
    normalized.twist.covariance = covariance_values(
        transform_twist_covariance(twist_covariance, base_to_lidar)
    )
    return normalized


def odom_to_base_transform(message: OdometryMessage) -> OdomToBaseTransform:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return OdomToBaseTransform(
        stamp=message.header.stamp,
        parent_frame="odom",
        child_frame="base_link",
        translation_x=position.x,
        translation_y=position.y,
        translation_z=position.z,
        rotation_x=orientation.x,
        rotation_y=orientation.y,
        rotation_z=orientation.z,
        rotation_w=orientation.w,
    )
