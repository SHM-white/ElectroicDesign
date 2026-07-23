"""Field-anchor publisher — single authority for the map→odom transform.

Initializes from a known field-profile takeoff pose and the current
odom→base_link pose, then publishes a ``geometry_msgs/TransformStamped``
on ``/tf_static``.  Later boundary corrections update only this transform,
isolating them from continuous odometry.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ed_uav_localization.field_profile.anchor import (
    MapToOdomTransform,
    Pose2D,
    initialize_map_to_odom,
)
from ed_uav_localization.field_profile.loader import load_profile
from ed_uav_localization.field_profile.model import KnownFieldProfile
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster


def _extract_yaw(q: Quaternion) -> float:
    """Return ENU yaw from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FieldAnchor(Node):
    """Publish the static map→odom transform derived from a field profile.

    Subscribes to ``/localization/odom`` once at startup to capture the
    odom→base_link pose at takeoff, then broadcasts the map→odom transform
    via ``/tf_static``.

    Parameters
    ----------
    profile_path : str
        Absolute path to a validated field-profile YAML file.
        The profile must be ``profile_type: field``.
    takeoff_timeout_sec : float
        Maximum time (s) to wait for the initial odometry message.
        Default ``10.0``.
    map_frame : str
        Parent frame identifier.  Default ``"map"``.
    odom_frame : str
        Child frame identifier.  Default ``"odom"``.
    """

    def __init__(self) -> None:
        super().__init__("field_anchor")

        self.declare_parameter("profile_path", "")
        self.declare_parameter("takeoff_timeout_sec", 10.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")

        profile_path: str = self.get_parameter("profile_path").value
        takeoff_timeout: float = self.get_parameter("takeoff_timeout_sec").value
        self._map_frame: str = self.get_parameter("map_frame").value
        self._odom_frame: str = self.get_parameter("odom_frame").value

        if not profile_path:
            self.get_logger().error("profile_path parameter is required")
            raise ValueError("profile_path parameter is required")

        # --- Load profile ---
        profile = load_profile(Path(profile_path))
        if not isinstance(profile, KnownFieldProfile):
            self.get_logger().error(
                f"Profile {profile_path} is not a known field profile"
            )
            raise ValueError("Only KnownFieldProfile profiles are supported")
        self._profile: KnownFieldProfile = profile

        # --- Broadcasters ---
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        # --- Wait for first odometry ---
        self._initial_odom: Optional[Odometry] = None
        self._odom_sub = self.create_subscription(
            Odometry,
            "/localization/odom",
            self._takeoff_odom_callback,
            10,
        )

        self._takeoff_timeout = takeoff_timeout
        self._start_time = self.get_clock().now()

        # Timer to check timeout and retry
        self._check_timer = self.create_timer(0.5, self._check_initialization)

        self._initialized = False

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #

    def _takeoff_odom_callback(self, msg: Odometry) -> None:
        if self._initialized or self._initial_odom is not None:
            return
        self._initial_odom = msg
        self.get_logger().info("Captured takeoff odometry for field anchor")

    def _check_initialization(self) -> None:
        if self._initialized:
            return

        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds * 1e-9

        if self._initial_odom is not None:
            self._publish_anchor(self._initial_odom)
            self._initialized = True
            self.get_logger().info("Field anchor initialized and published")
            # Stop the timer and subscription once initialized.
            self.destroy_timer(self._check_timer)
            self.destroy_subscription(self._odom_sub)
            return

        if elapsed > self._takeoff_timeout:
            self.get_logger().warn(
                f"No odometry received within {self._takeoff_timeout:.1f}s"
            )
            # Keep retrying — the timer continues.

    # ------------------------------------------------------------------ #
    #  Anchor publishing                                                  #
    # ------------------------------------------------------------------ #

    def _publish_anchor(self, odom_msg: Odometry) -> None:
        """Derive map→odom from profile takeoff and odom→base, then broadcast."""
        # Extract odom→base_link planar pose from odometry message.
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        odom_pose = Pose2D(
            x_m=float(position.x),
            y_m=float(position.y),
            yaw_rad=_extract_yaw(orientation),
        )

        # Derive the map→odom transform.
        transform = initialize_map_to_odom(self._profile, odom_pose)

        # Build and publish TransformStamped.
        tf_msg = _to_transform_stamped(
            transform,
            stamp=odom_msg.header.stamp,
            map_frame=self._map_frame,
            odom_frame=self._odom_frame,
        )
        self._tf_broadcaster.sendTransform(tf_msg)
        self.get_logger().info(
            f"Published map→odom: "
            f"x={transform.x_m:.3f} y={transform.y_m:.3f} "
            f"yaw={math.degrees(transform.yaw_rad):.1f}°"
        )


# ------------------------------------------------------------------ #
#  Helpers                                                            #
# ------------------------------------------------------------------ #


def _to_transform_stamped(
    transform: MapToOdomTransform,
    *,
    stamp: "builtin_interfaces.msg.Time",  # type: ignore[name-defined]  # noqa: F821
    map_frame: str,
    odom_frame: str,
) -> TransformStamped:
    half = transform.yaw_rad / 2.0
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = map_frame
    msg.child_frame_id = odom_frame
    msg.transform.translation = Vector3(
        x=transform.x_m, y=transform.y_m, z=0.0
    )
    msg.transform.rotation = Quaternion(
        x=0.0, y=0.0, z=math.sin(half), w=math.cos(half),
    )
    return msg
