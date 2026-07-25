"""Localization source supervisor with no-jump source switching.

Pure logic functions are testable without ROS infrastructure.  The
``SourceSupervisor`` node subscribes to LIO odometry, visual odometry,
and boundary observations, then publishes a fused odometry output and a
localization status report.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class LocalizationSource(IntEnum):
    """Which source is currently driving the fused output."""

    NONE = 0
    LIO = 1
    VISUAL = 2


class SourceState(IntEnum):
    """Health state for a single localization source."""

    ACTIVE = 0
    DEGRADED = 1
    LOST = 2


# ---------------------------------------------------------------------------
# Thresholds
# ---------------------------------------------------------------------------


@dataclass
class SupervisorThresholds:
    """Configurable thresholds for the source supervisor.

    All durations are in seconds.
    """

    lio_max_age_active: float = 0.15
    lio_max_age_degraded: float = 0.50
    visual_max_age_active: float = 0.20
    visual_max_age_degraded: float = 0.50
    lost_timeout: float = 1.0
    covariance_blowup: float = 1e6
    visual_stability_duration: float = 0.5
    visual_consecutive_samples: int = 5
    primary_hysteresis: float = 2.0
    max_switch_position_diff_m: float = 0.25
    max_switch_yaw_diff_rad: float = math.radians(10.0)


# ---------------------------------------------------------------------------
# Quaternion helpers  (no ROS types — testable without rclpy)
# ---------------------------------------------------------------------------


def extract_yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return ENU yaw (radians) from quaternion components."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_difference_rad(first: float, second: float) -> float:
    """Shortest absolute angular distance, result in [0, π]."""
    diff = (first - second) % (2.0 * math.pi)
    if diff > math.pi:
        diff -= 2.0 * math.pi
    return abs(diff)


# ---------------------------------------------------------------------------
# Covariance helpers
# ---------------------------------------------------------------------------

_COV_DIAG_IDX = (0, 7, 14, 21, 28, 35)


def _covariance_finite(cov: tuple[float, ...]) -> bool:
    """Return True when every covariance element is finite."""
    return all(math.isfinite(v) for v in cov)


def _covariance_exceeds(cov: tuple[float, ...], blowup: float) -> bool:
    """Return True when any pose diagonal exceeds *blowup*."""
    return any(
        i < len(cov) and abs(cov[i]) > blowup for i in _COV_DIAG_IDX
    )


# ---------------------------------------------------------------------------
# Pure evaluation functions  (testable without ROS)
# ---------------------------------------------------------------------------


def evaluate_source_state(
    *,
    age_sec: Optional[float],
    no_msg_duration_sec: Optional[float],
    time_regression: bool,
    covariance_finite: bool,
    covariance_exceeds: bool,
    max_age_active: float,
    max_age_degraded: float,
    lost_timeout: float,
    covariance_blowup: float,  # noqa: ARG001 — kept for API consistency
) -> SourceState:
    """Evaluate the health state of a single localization source.

    All timing arguments use seconds so callers can supply them without
    importing ROS types.
    """
    if not covariance_finite or covariance_exceeds:
        return SourceState.LOST

    if no_msg_duration_sec is not None and no_msg_duration_sec > lost_timeout:
        return SourceState.LOST

    if age_sec is None:
        return SourceState.LOST

    if age_sec > max_age_degraded or time_regression:
        return SourceState.DEGRADED

    if age_sec > max_age_active:
        return SourceState.DEGRADED

    return SourceState.ACTIVE


def is_visual_stable(
    *,
    consecutive_count: int,
    first_stable_time_sec: Optional[float],
    now_sec: float,
    required_samples: int,
    required_duration: float,
) -> bool:
    """Check whether the visual source meets stability criteria.

    Requires *required_samples* consecutive valid observations spanning at
    least *required_duration* seconds.
    """
    if consecutive_count < required_samples:
        return False
    if first_stable_time_sec is None:
        return False
    return (now_sec - first_stable_time_sec) >= required_duration


def decide_source_switch(
    *,
    current_primary: LocalizationSource,
    lio_state: SourceState,
    visual_state: SourceState,
    visual_stable: bool,
    primary_duration_sec: float,
    thresholds: SupervisorThresholds,
) -> Optional[LocalizationSource]:
    """Decide which source should be the primary.

    Returns ``None`` when no switch is warranted, or the new primary source.
    This is a pure function; alignment checks happen separately.
    """
    if current_primary == LocalizationSource.LIO:
        if lio_state == SourceState.ACTIVE:
            return None  # Already on best source.
        if lio_state == SourceState.LOST:
            # Fall back to visual if available and stable.
            if visual_state != SourceState.LOST and visual_stable:
                if primary_duration_sec < thresholds.primary_hysteresis:
                    return None  # Hysteresis keeps us on (lost) LIO briefly.
                return LocalizationSource.VISUAL
            if visual_state == SourceState.LOST:
                return LocalizationSource.NONE
            return None

    if current_primary == LocalizationSource.VISUAL:
        if lio_state == SourceState.ACTIVE:
            if primary_duration_sec >= thresholds.primary_hysteresis:
                return LocalizationSource.LIO  # LIO recovered, hysteresis satisfied.
            return None  # Hysteresis not yet satisfied.
        if visual_state == SourceState.ACTIVE:
            return None  # Visual still healthy.
        if visual_state == SourceState.LOST:
            if lio_state != SourceState.LOST:
                return LocalizationSource.LIO  # Fall back to LIO (even if degraded).
            return LocalizationSource.NONE
        return None

    if current_primary == LocalizationSource.NONE:
        if lio_state == SourceState.ACTIVE:
            return LocalizationSource.LIO
        if visual_state != SourceState.LOST and visual_stable:
            return LocalizationSource.VISUAL
        return None

    return None


def poses_aligned(
    *,
    current_x: float,
    current_y: float,
    current_yaw: float,
    candidate_x: float,
    candidate_y: float,
    candidate_yaw: float,
    max_position_diff_m: float,
    max_yaw_diff_rad: float,
) -> bool:
    """Check whether a switch would cause unacceptable pose discontinuity.

    Returns ``True`` when the candidate pose is close enough to the current
    fused pose for a smooth (no-jump) switch.
    """
    pos_diff = math.hypot(candidate_x - current_x, candidate_y - current_y)
    if pos_diff > max_position_diff_m:
        return False
    yaw_diff = yaw_difference_rad(current_yaw, candidate_yaw)
    return yaw_diff <= max_yaw_diff_rad


# ---------------------------------------------------------------------------
# Node  (imports rclpy / ROS message types lazily — see _ROS_IMPORTS guard)
# ---------------------------------------------------------------------------


def _ros_time_float(stamp: "builtin_interfaces.msg.Time") -> float:  # type: ignore[name-defined]  # noqa: F821
    """Convert a ROS Time message to seconds since epoch."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SourceSupervisor(Node):
    """Localization source supervisor with no-jump switching.

    The ``__init__`` method performs deferred ROS imports so that the pure
    logic above remains importable without rclpy (e.g. in pytest).

    Parameters
    ----------
    lio_max_age_active : float
        Max LIO age (s) for ACTIVE state.  Default ``0.15``.
    lio_max_age_degraded : float
        Max LIO age (s) for DEGRADED state.  Default ``0.50``.
    visual_max_age_active : float
        Max visual age (s) for ACTIVE state.  Default ``0.20``.
    visual_max_age_degraded : float
        Max visual age (s) for DEGRADED state.  Default ``0.50``.
    lost_timeout : float
        Seconds without any message before declaring LOST.  Default ``1.0``.
    covariance_blowup : float
        Diagonal pose-covariance threshold for LOST.  Default ``1e6``.
    visual_stability_duration : float
        Minimum duration (s) of stable visual before candidate.  Default ``0.5``.
    visual_consecutive_samples : int
        Consecutive valid visual samples required.  Default ``5``.
    primary_hysteresis : float
        Minimum time (s) on current primary before switching back.  Default ``2.0``.
    max_switch_position_diff_m : float
        Max position jump (m) allowed on switch.  Default ``0.25``.
    max_switch_yaw_diff_rad : float
        Max yaw jump (rad) allowed on switch.  Default ``0.175`` (~10°).
    publish_rate : float
        Periodic evaluation rate (Hz).  Default ``20.0``.
    """

    # Deferred ROS imports — set during __init__.
    _Odometry: type = None  # type: ignore[assignment]
    _BoundaryObservation: type = None  # type: ignore[assignment]
    _LocalizationStatus: type = None  # type: ignore[assignment]

    def __init__(self) -> None:
        # --- Deferred ROS imports ---
        from rclpy.duration import Duration
        from rclpy.time import Time

        from ed_uav_interfaces.msg import BoundaryObservation as _BO
        from ed_uav_interfaces.msg import LocalizationStatus as _LS
        from nav_msgs.msg import Odometry as _Odom

        SourceSupervisor._Odometry = _Odom
        SourceSupervisor._BoundaryObservation = _BO
        SourceSupervisor._LocalizationStatus = _LS

        # --- Init the actual ROS node ---
        super().__init__("source_supervisor")

        # --- Parameters ---
        self.declare_parameter("lio_max_age_active", 0.15)  # type: ignore[attr-defined]
        self.declare_parameter("lio_max_age_degraded", 0.50)  # type: ignore[attr-defined]
        self.declare_parameter("visual_max_age_active", 0.20)  # type: ignore[attr-defined]
        self.declare_parameter("visual_max_age_degraded", 0.50)  # type: ignore[attr-defined]
        self.declare_parameter("lost_timeout", 1.0)  # type: ignore[attr-defined]
        self.declare_parameter("covariance_blowup", 1e6)  # type: ignore[attr-defined]
        self.declare_parameter("visual_stability_duration", 0.5)  # type: ignore[attr-defined]
        self.declare_parameter("visual_consecutive_samples", 5)  # type: ignore[attr-defined]
        self.declare_parameter("primary_hysteresis", 2.0)  # type: ignore[attr-defined]
        self.declare_parameter("max_switch_position_diff_m", 0.25)  # type: ignore[attr-defined]
        self.declare_parameter("max_switch_yaw_diff_rad", math.radians(10.0))  # type: ignore[attr-defined]
        self.declare_parameter("publish_rate", 20.0)  # type: ignore[attr-defined]

        self._thresholds = SupervisorThresholds(
            lio_max_age_active=self.get_parameter("lio_max_age_active").value,  # type: ignore[attr-defined]
            lio_max_age_degraded=self.get_parameter("lio_max_age_degraded").value,  # type: ignore[attr-defined]
            visual_max_age_active=self.get_parameter("visual_max_age_active").value,  # type: ignore[attr-defined]
            visual_max_age_degraded=self.get_parameter("visual_max_age_degraded").value,  # type: ignore[attr-defined]
            lost_timeout=self.get_parameter("lost_timeout").value,  # type: ignore[attr-defined]
            covariance_blowup=self.get_parameter("covariance_blowup").value,  # type: ignore[attr-defined]
            visual_stability_duration=self.get_parameter("visual_stability_duration").value,  # type: ignore[attr-defined]
            visual_consecutive_samples=self.get_parameter("visual_consecutive_samples").value,  # type: ignore[attr-defined]
            primary_hysteresis=self.get_parameter("primary_hysteresis").value,  # type: ignore[attr-defined]
            max_switch_position_diff_m=self.get_parameter("max_switch_position_diff_m").value,  # type: ignore[attr-defined]
            max_switch_yaw_diff_rad=self.get_parameter("max_switch_yaw_diff_rad").value,  # type: ignore[attr-defined]
        )

        # --- LIO state ---
        self._lio_last_time: Optional[Time] = None  # type: ignore[valid-type]
        self._lio_previous_stamp: Optional[Time] = None  # type: ignore[valid-type]
        self._lio_time_regression: bool = False
        self._lio_cov_finite: bool = True
        self._lio_cov_exceeds: bool = False
        self._lio_last_valid_time: Optional[Time] = None  # type: ignore[valid-type]
        self._lio_latest_odom: Optional["_Odom"] = None  # type: ignore[name-defined]

        # --- Visual state ---
        self._vis_last_time: Optional[Time] = None  # type: ignore[valid-type]
        self._vis_previous_stamp: Optional[Time] = None  # type: ignore[valid-type]
        self._vis_time_regression: bool = False
        self._vis_cov_finite: bool = True
        self._vis_cov_exceeds: bool = False
        self._vis_last_valid_time: Optional[Time] = None  # type: ignore[valid-type]
        self._vis_latest_odom: Optional["_Odom"] = None  # type: ignore[name-defined]

        # Visual stability tracking
        self._vis_consecutive_count: int = 0
        self._vis_first_stable_time_sec: Optional[float] = None  # ROS time

        # Boundary observation state
        self._last_boundary_obs: Optional["_BO"] = None  # type: ignore[name-defined]

        # --- Switching state ---
        self._primary_source: LocalizationSource = LocalizationSource.NONE
        self._primary_since: Optional[Time] = None  # type: ignore[valid-type]
        self._fused_odom: Optional["_Odom"] = None  # type: ignore[name-defined]
        self._override_reason: str = "uninitialized"

        # --- Subscriptions ---
        self._lio_sub = self.create_subscription(  # type: ignore[attr-defined]
            _Odom, "/localization/lio/odom", self._lio_callback, 10
        )
        self._vis_sub = self.create_subscription(  # type: ignore[attr-defined]
            _Odom, "/localization/visual/odom", self._visual_callback, 10
        )
        self._boundary_sub = self.create_subscription(  # type: ignore[attr-defined]
            _BO,
            "/localization/boundary_observation",
            self._boundary_callback,
            10,
        )

        # --- Publishers ---
        self._odom_pub = self.create_publisher(  # type: ignore[attr-defined]
            _Odom, "/localization/odom", 10
        )
        self._status_pub = self.create_publisher(  # type: ignore[attr-defined]
            _LS, "/localization/status", 10
        )

        # --- Periodic evaluation ---
        publish_rate = self.get_parameter("publish_rate").value  # type: ignore[attr-defined]
        period = 1.0 / max(float(publish_rate), 1.0)
        self._timer = self.create_timer(period, self._evaluate_and_publish)  # type: ignore[attr-defined]

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #

    def _lio_callback(self, msg: "_Odometry") -> None:  # type: ignore[name-defined]  # noqa: F821
        from rclpy.time import Time
        stamp = Time.from_msg(msg.header.stamp)
        if self._lio_previous_stamp is not None and stamp <= self._lio_previous_stamp:
            self._lio_time_regression = True
        else:
            self._lio_time_regression = False
        self._lio_previous_stamp = stamp
        self._lio_last_time = stamp
        self._lio_last_valid_time = stamp
        self._lio_latest_odom = msg
        cov = tuple(msg.pose.covariance)
        self._lio_cov_finite = _covariance_finite(cov)
        self._lio_cov_exceeds = _covariance_exceeds(
            cov, self._thresholds.covariance_blowup
        )

    def _visual_callback(self, msg: "_Odometry") -> None:  # type: ignore[name-defined]  # noqa: F821
        from rclpy.time import Time
        stamp = Time.from_msg(msg.header.stamp)
        if self._vis_previous_stamp is not None and stamp <= self._vis_previous_stamp:
            self._vis_time_regression = True
        else:
            self._vis_time_regression = False
        self._vis_previous_stamp = stamp
        self._vis_last_time = stamp
        self._vis_last_valid_time = stamp
        self._vis_latest_odom = msg
        cov = tuple(msg.pose.covariance)
        self._vis_cov_finite = _covariance_finite(cov)
        self._vis_cov_exceeds = _covariance_exceeds(
            cov, self._thresholds.covariance_blowup
        )

        # Track consecutive validity for stability check.
        if (
            self._vis_cov_finite
            and not self._vis_cov_exceeds
            and not self._vis_time_regression
        ):
            if self._vis_consecutive_count == 0:
                self._vis_first_stable_time_sec = _ros_time_float(msg.header.stamp)
            self._vis_consecutive_count += 1
        else:
            self._vis_consecutive_count = 0
            self._vis_first_stable_time_sec = None

    def _boundary_callback(self, msg: "_BoundaryObservation") -> None:  # type: ignore[name-defined]  # noqa: F821
        self._last_boundary_obs = msg

    # ------------------------------------------------------------------ #
    #  Periodic evaluation                                                #
    # ------------------------------------------------------------------ #

    def _evaluate_and_publish(self) -> None:
        from rclpy.duration import Duration
        from rclpy.time import Time

        now: Time = self.get_clock().now()  # type: ignore[attr-defined]
        now_sec = now.nanoseconds * 1e-9

        # --- Evaluate source states ---
        lio_age, lio_no_msg = self._compute_ages(
            self._lio_last_time, self._lio_last_valid_time, now
        )
        vis_age, vis_no_msg = self._compute_ages(
            self._vis_last_time, self._vis_last_valid_time, now
        )

        lio_state = evaluate_source_state(
            age_sec=lio_age,
            no_msg_duration_sec=lio_no_msg,
            time_regression=self._lio_time_regression,
            covariance_finite=self._lio_cov_finite,
            covariance_exceeds=self._lio_cov_exceeds,
            max_age_active=self._thresholds.lio_max_age_active,
            max_age_degraded=self._thresholds.lio_max_age_degraded,
            lost_timeout=self._thresholds.lost_timeout,
            covariance_blowup=self._thresholds.covariance_blowup,
        )
        vis_state = evaluate_source_state(
            age_sec=vis_age,
            no_msg_duration_sec=vis_no_msg,
            time_regression=self._vis_time_regression,
            covariance_finite=self._vis_cov_finite,
            covariance_exceeds=self._vis_cov_exceeds,
            max_age_active=self._thresholds.visual_max_age_active,
            max_age_degraded=self._thresholds.visual_max_age_degraded,
            lost_timeout=self._thresholds.lost_timeout,
            covariance_blowup=self._thresholds.covariance_blowup,
        )

        # --- Visual stability ---
        visual_stable = is_visual_stable(
            consecutive_count=self._vis_consecutive_count,
            first_stable_time_sec=self._vis_first_stable_time_sec,
            now_sec=now_sec,
            required_samples=self._thresholds.visual_consecutive_samples,
            required_duration=self._thresholds.visual_stability_duration,
        )

        # --- Primary duration ---
        primary_duration = 0.0
        if self._primary_since is not None:
            primary_duration = (now - self._primary_since).nanoseconds * 1e-9

        # --- Switching decision ---
        candidate = decide_source_switch(
            current_primary=self._primary_source,
            lio_state=lio_state,
            visual_state=vis_state,
            visual_stable=visual_stable,
            primary_duration_sec=primary_duration,
            thresholds=self._thresholds,
        )

        self._override_reason = ""

        if candidate is not None and candidate != self._primary_source:
            aligned, reason = self._check_alignment_for(candidate)
            if aligned:
                self._primary_source = candidate
                self._primary_since = now
                self._override_reason = f"switched to {candidate.name}"
            else:
                self._override_reason = reason
        elif self._primary_source == LocalizationSource.NONE:
            self._override_reason = "no valid source"

        # --- Publish fused odometry ---
        odom = self._select_odom()
        if odom is not None:
            self._fused_odom = odom
            self._odom_pub.publish(odom)  # type: ignore[attr-defined]

        # --- Publish status ---
        self._publish_status(now, lio_state, vis_state, lio_age)

    # ------------------------------------------------------------------ #
    #  Helpers                                                            #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _compute_ages(
        last_time: "Optional[Time]",  # type: ignore[name-defined]  # noqa: F821
        last_valid_time: "Optional[Time]",  # type: ignore[name-defined]  # noqa: F821
        now: "Time",  # type: ignore[name-defined]  # noqa: F821
    ) -> tuple[Optional[float], Optional[float]]:
        age = (
            (now - last_time).nanoseconds * 1e-9
            if last_time is not None
            else None
        )
        no_msg = (
            (now - last_valid_time).nanoseconds * 1e-9
            if last_valid_time is not None
            else None
        )
        return age, no_msg

    def _check_alignment_for(
        self, candidate: LocalizationSource
    ) -> tuple[bool, str]:
        """Check whether switching to *candidate* would cause a position/yaw jump."""
        if self._fused_odom is None:
            return True, ""

        candidate_odom = self._get_odom_for(candidate)
        if candidate_odom is None:
            return False, f"no odometry for {candidate.name}"

        current_x = self._fused_odom.pose.pose.position.x
        current_y = self._fused_odom.pose.pose.position.y
        current_q = self._fused_odom.pose.pose.orientation
        current_yaw = extract_yaw_from_quat(
            current_q.x, current_q.y, current_q.z, current_q.w
        )

        cand_x = candidate_odom.pose.pose.position.x
        cand_y = candidate_odom.pose.pose.position.y
        cand_q = candidate_odom.pose.pose.orientation
        cand_yaw = extract_yaw_from_quat(
            cand_q.x, cand_q.y, cand_q.z, cand_q.w
        )

        aligned = poses_aligned(
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            candidate_x=cand_x,
            candidate_y=cand_y,
            candidate_yaw=cand_yaw,
            max_position_diff_m=self._thresholds.max_switch_position_diff_m,
            max_yaw_diff_rad=self._thresholds.max_switch_yaw_diff_rad,
        )
        if not aligned:
            pos_diff = math.hypot(cand_x - current_x, cand_y - current_y)
            yaw_diff = math.degrees(
                yaw_difference_rad(current_yaw, cand_yaw)
            )
            return False, (
                f"reject {candidate.name}: "
                f"pos_diff={pos_diff:.3f}m, yaw_diff={yaw_diff:.1f}°"
            )
        return True, ""

    def _get_odom_for(self, source: LocalizationSource) -> "Optional[_Odometry]":  # type: ignore[name-defined]  # noqa: F821
        if source == LocalizationSource.LIO:
            return self._lio_latest_odom
        if source == LocalizationSource.VISUAL:
            return self._vis_latest_odom
        return None

    def _select_odom(self) -> "Optional[_Odometry]":  # type: ignore[name-defined]  # noqa: F821
        """Return the odometry message for the current primary source."""
        return self._get_odom_for(self._primary_source)

    def _publish_status(
        self,
        now: "Time",  # type: ignore[name-defined]  # noqa: F821
        lio_state: SourceState,
        vis_state: SourceState,
        lio_age: Optional[float],
    ) -> None:
        msg = self._LocalizationStatus()  # type: ignore[misc]
        msg.acquisition_stamp = now.to_msg()
        msg.source_sequence = 0

        if self._primary_source == LocalizationSource.LIO:
            msg.source = self._LocalizationStatus.SOURCE_LIO  # type: ignore[attr-defined]
        elif self._primary_source == LocalizationSource.VISUAL:
            msg.source = self._LocalizationStatus.SOURCE_VISUAL_BOUNDARY  # type: ignore[attr-defined]
        else:
            msg.source = self._LocalizationStatus.SOURCE_NONE  # type: ignore[attr-defined]

        if lio_state == SourceState.LOST and vis_state == SourceState.LOST:
            msg.state = self._LocalizationStatus.STATE_LOST  # type: ignore[attr-defined]
        elif lio_state == SourceState.DEGRADED or vis_state == SourceState.DEGRADED:
            msg.state = self._LocalizationStatus.STATE_DEGRADED  # type: ignore[attr-defined]
        elif self._primary_source != LocalizationSource.NONE:
            msg.state = self._LocalizationStatus.STATE_ACTIVE  # type: ignore[attr-defined]
        else:
            msg.state = self._LocalizationStatus.STATE_UNINITIALIZED  # type: ignore[attr-defined]

        msg.age_sec = float(lio_age) if lio_age is not None else -1.0
        msg.map_to_odom_valid = True
        msg.reason = self._override_reason[:96]
        self._status_pub.publish(msg)  # type: ignore[attr-defined]


def main(args: list[str] | None = None) -> None:
    """Run the source supervisor ROS node."""
    rclpy.init(args=args)
    node = SourceSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
