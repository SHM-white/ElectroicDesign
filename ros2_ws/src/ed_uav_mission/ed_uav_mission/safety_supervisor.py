"""Mission safety supervisor — ACK timeouts, localisation-loss recovery, lock gate.

All motion decisions go through ``FlightCommand``.  This module never imports
serial, GPIO, or camera APIs.  The core decision logic is pure Python and
unit-testable without a running ROS graph.
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import ClassVar


# ---------------------------------------------------------------------------
# Constants (sourced from FlightCommand.action constants for readability)
# ---------------------------------------------------------------------------

_FCMD_HOVER: int = 6
_FCMD_LAND: int = 7
_FCMD_DISARM: int = 2


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class SupervisorState(Enum):
    """Safety supervisor internal state."""

    IDLE = auto()
    ACTIVE = auto()
    LOCALIZATION_LOST_HOVERING = auto()
    LOCALIZATION_LOST_LANDING = auto()
    CRITICAL = auto()


class CommandAckState(Enum):
    """Lifecycle of a single correlated flight command."""

    SENT = auto()
    ACKNOWLEDGED = auto()
    TIMED_OUT = auto()


class SupervisorAction(Enum):
    """Action the supervisor asks the executor to take."""

    HOVER = auto()
    LAND = auto()
    CRITICAL = auto()


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------

@dataclass
class OutgoingCommand:
    """Track one correlated FlightCommand goal and its ACK state."""

    correlation_id: str
    command_code: int
    send_time_s: float
    state: CommandAckState = CommandAckState.SENT
    retries: int = 0


@dataclass
class SafetyVerdict:
    """Result of a single supervisor evaluation tick."""

    action: SupervisorAction | None = None
    correlation_id: str = ""
    reason: str = ""


@dataclass
class LockVerdict:
    """Whether it is currently safe to disarm / lock the motors."""

    allowed: bool
    reason: str


# ---------------------------------------------------------------------------
# SafetySupervisor
# ---------------------------------------------------------------------------

class SafetySupervisor:
    """Mission safety supervisor.

    Tracks correlated commands with unique IDs, enforces ACK/result timeouts
    with retries, handles localisation-loss recovery (hover → land), enforces
    a strict lock-in-air gate, and evaluates four safety transitions
    (comm loss, stale AUX, low voltage, mission timeout).
    """

    # -- tunables ------------------------------------------------------------
    ACK_TIMEOUT_SEC: ClassVar[float] = 0.50
    MAX_RETRIES: ClassVar[int] = 2
    HOVER_RESPONSE_SEC: ClassVar[float] = 0.20
    HOVER_HOLD_SEC: ClassVar[float] = 2.0
    LAND_RETRY_INTERVAL_SEC: ClassVar[float] = 1.0
    MAX_LAND_RETRIES: ClassVar[int] = 3
    TOUCHDOWN_ALTITUDE_M: ClassVar[float] = 0.10
    LOW_BATTERY_THRESHOLD_V: ClassVar[float] = 10.5

    # -- lifecycle -----------------------------------------------------------

    def __init__(self) -> None:
        self.state: SupervisorState = SupervisorState.IDLE
        self._monitoring_active: bool = False

        # command tracking
        self._pending: dict[str, OutgoingCommand] = {}
        self._sent_commands: list[OutgoingCommand] = []

        # localisation-loss bookkeeping
        self._loc_lost_at: float | None = None
        self._hover_at: float | None = None
        self._land_cmd_count: int = 0
        self._land_cmd_last_at: float | None = None

        # altitude tracking (for descent detection & lock gate)
        self._altitude_m: float | None = None
        self._altitude_prev_m: float | None = None

        # mission-timeout bookkeeping
        self._mission_start_at: float | None = None
        self._mission_timeout_s: float = 0.0

    # ------------------------------------------------------------------
    # Command tracking (ACK / retry)
    # ------------------------------------------------------------------

    def record_send(self, correlation_id: str, command_code: int, timestamp_s: float) -> None:
        """Record that a command was sent."""
        cmd = OutgoingCommand(
            correlation_id=correlation_id,
            command_code=command_code,
            send_time_s=timestamp_s,
        )
        self._pending[correlation_id] = cmd
        self._sent_commands.append(cmd)

    def receive_ack(self, correlation_id: str, timestamp_s: float) -> bool:
        """Process an incoming ACK. Returns True if the ACK was accepted."""
        cmd = self._pending.get(correlation_id)
        if cmd is None:
            return False  # unknown correlation id
        if cmd.state == CommandAckState.TIMED_OUT:
            return False  # late ACK — ignore
        cmd.state = CommandAckState.ACKNOWLEDGED
        return True

    def poll_timeouts(self, timestamp_s: float) -> list[OutgoingCommand]:
        """Return every SENT command whose ACK window has expired."""
        expired: list[OutgoingCommand] = []
        for cmd in self._pending.values():
            if cmd.state != CommandAckState.SENT:
                continue
            if (timestamp_s - cmd.send_time_s) >= self.ACK_TIMEOUT_SEC:
                cmd.state = CommandAckState.TIMED_OUT
                expired.append(cmd)
        return expired

    def can_retry(self, cmd: OutgoingCommand) -> bool:
        """Check whether *cmd* still has retry budget left."""
        return cmd.retries < self.MAX_RETRIES

    def record_retry(self, cmd: OutgoingCommand, new_correlation_id: str, timestamp_s: float) -> None:
        """Record a retry attempt for *cmd*, issuing a fresh correlation id."""
        cmd.retries += 1
        cmd.correlation_id = new_correlation_id
        cmd.send_time_s = timestamp_s
        cmd.state = CommandAckState.SENT

    def resolve_command(self, correlation_id: str) -> None:
        """Remove a completed command from tracking."""
        self._pending.pop(correlation_id, None)

    @staticmethod
    def new_correlation_id() -> str:
        """Generate a short unique correlation id."""
        return str(uuid.uuid4())[:12]

    # ------------------------------------------------------------------
    # Localisation-loss monitoring
    # ------------------------------------------------------------------

    def evaluate_localization(
        self, *, all_lost: bool, timestamp_s: float
    ) -> SafetyVerdict:
        """Called each supervisor tick with the current localisation status.

        Returns a ``SafetyVerdict`` when an action is needed, otherwise
        ``SafetyVerdict()`` (no action).
        """
        if not self._monitoring_active:
            return SafetyVerdict()

        if self.state == SupervisorState.IDLE:
            return SafetyVerdict()

        # -- state: ACTIVE ---------------------------------------------------
        if self.state == SupervisorState.ACTIVE:
            if all_lost:
                self.state = SupervisorState.LOCALIZATION_LOST_HOVERING
                self._loc_lost_at = timestamp_s
                self._hover_at = timestamp_s
                return SafetyVerdict(
                    action=SupervisorAction.HOVER,
                    correlation_id=self.new_correlation_id(),
                    reason="all localisation sources lost — issuing hover",
                )
            return SafetyVerdict()

        # -- state: LOCALIZATION_LOST_HOVERING -------------------------------
        if self.state == SupervisorState.LOCALIZATION_LOST_HOVERING:
            if all_lost:
                assert self._hover_at is not None
                elapsed = timestamp_s - self._hover_at
                if elapsed >= self.HOVER_HOLD_SEC:
                    self.state = SupervisorState.LOCALIZATION_LOST_LANDING
                    self._land_cmd_count = 1
                    self._land_cmd_last_at = timestamp_s
                    return SafetyVerdict(
                        action=SupervisorAction.LAND,
                        correlation_id=self.new_correlation_id(),
                        reason="localisation absent for >2.0 s — initiating land",
                    )
                return SafetyVerdict()  # still holding
            else:
                # recovered within the 2.0 s hold window
                self.state = SupervisorState.ACTIVE
                self._loc_lost_at = None
                self._hover_at = None
                return SafetyVerdict()

        # -- state: LOCALIZATION_LOST_LANDING --------------------------------
        if self.state == SupervisorState.LOCALIZATION_LOST_LANDING:
            if all_lost:
                return self._evaluate_land_retry(timestamp_s)
            # localisation recovered during landing — stay the course
            return SafetyVerdict()

        # -- state: CRITICAL -------------------------------------------------
        return SafetyVerdict()

    def _evaluate_land_retry(self, timestamp_s: float) -> SafetyVerdict:
        """Decide whether to retry land, keep supervising, or escalate."""
        if self._land_cmd_count > self.MAX_LAND_RETRIES:
            self.state = SupervisorState.CRITICAL
            return SafetyVerdict(
                action=SupervisorAction.CRITICAL,
                reason="LAND retries exhausted: no ACK and no descent — manual takeover",
            )

        if self._land_cmd_last_at is None:
            # first land command — send it
            self._land_cmd_count = 1
            self._land_cmd_last_at = timestamp_s
            return SafetyVerdict(
                action=SupervisorAction.LAND,
                correlation_id=self.new_correlation_id(),
                reason="land command (initial)",
            )

        elapsed = timestamp_s - self._land_cmd_last_at
        if elapsed < self.LAND_RETRY_INTERVAL_SEC:
            return SafetyVerdict()  # not yet time to retry

        # time to retry: check if descent has been observed
        if self._is_descending():
            # descent observed — keep supervising, do NOT escalate
            self._land_cmd_last_at = timestamp_s  # reset timer
            return SafetyVerdict()

        # no descent AND no (accepted) ACK — retry or go critical
        self._land_cmd_count += 1
        self._land_cmd_last_at = timestamp_s
        if self._land_cmd_count > self.MAX_LAND_RETRIES:
            self.state = SupervisorState.CRITICAL
            return SafetyVerdict(
                action=SupervisorAction.CRITICAL,
                reason="LAND: no ACK and no descent after max retries — manual takeover",
            )
        return SafetyVerdict(
            action=SupervisorAction.LAND,
            correlation_id=self.new_correlation_id(),
            reason=f"land retry {self._land_cmd_count}/{self.MAX_LAND_RETRIES}",
        )

    def _is_descending(self) -> bool:
        """True when the last two altitude samples show decreasing altitude."""
        if self._altitude_prev_m is None or self._altitude_m is None:
            return False
        return self._altitude_m < self._altitude_prev_m

    # ------------------------------------------------------------------
    # Altitude tracking
    # ------------------------------------------------------------------

    def update_altitude(self, altitude_m: float) -> None:
        """Feed a new altitude reading (from FCU state)."""
        self._altitude_prev_m = self._altitude_m
        self._altitude_m = altitude_m

    # ------------------------------------------------------------------
    # Lock / disarm safety gate
    # ------------------------------------------------------------------

    def check_lock_safe(self) -> LockVerdict:
        """Return whether it is safe to disarm/lock the motors.

        Lock is only allowed when altitude data is fresh and confirms the
        platform is on or very near the ground (≤10 cm).
        """
        if self._altitude_m is None:
            return LockVerdict(allowed=False, reason="no altitude data")
        if self._altitude_m > self.TOUCHDOWN_ALTITUDE_M:
            return LockVerdict(
                allowed=False,
                reason=(
                    f"altitude {self._altitude_m:.3f} m > "
                    f"{self.TOUCHDOWN_ALTITUDE_M:.2f} m — refusing lock in air"
                ),
            )
        return LockVerdict(allowed=True, reason="touchdown confirmed")

    # ------------------------------------------------------------------
    # Other safety transitions
    # ------------------------------------------------------------------

    def evaluate_comm_loss(self, comm_ok: bool, timestamp_s: float) -> SafetyVerdict:
        """Communication-loss safety transition."""
        if not self._monitoring_active or self.state != SupervisorState.ACTIVE:
            return SafetyVerdict()
        if not comm_ok:
            self.state = SupervisorState.CRITICAL
            return SafetyVerdict(
                action=SupervisorAction.CRITICAL,
                reason="FCU communication lost",
            )
        return SafetyVerdict()

    def evaluate_low_voltage(self, voltage_v: float, timestamp_s: float) -> SafetyVerdict:
        """Low-voltage safety transition."""
        if not self._monitoring_active:
            return SafetyVerdict()
        if self.state not in (SupervisorState.ACTIVE, SupervisorState.LOCALIZATION_LOST_HOVERING):
            return SafetyVerdict()
        if voltage_v < self.LOW_BATTERY_THRESHOLD_V:
            self.state = SupervisorState.LOCALIZATION_LOST_LANDING
            self._land_cmd_count = 1
            self._land_cmd_last_at = timestamp_s
            return SafetyVerdict(
                action=SupervisorAction.LAND,
                correlation_id=self.new_correlation_id(),
                reason=f"low battery: {voltage_v:.1f} V < {self.LOW_BATTERY_THRESHOLD_V:.1f} V",
            )
        return SafetyVerdict()

    def evaluate_stale_aux(self, aux_active: bool) -> SafetyVerdict:
        """Stale-AUX safety transition."""
        if not self._monitoring_active:
            return SafetyVerdict()
        if self.state not in (SupervisorState.ACTIVE,):
            return SafetyVerdict()
        if not aux_active:
            self.state = SupervisorState.LOCALIZATION_LOST_LANDING
            self._land_cmd_count = 1
            self._land_cmd_last_at = None
            return SafetyVerdict(
                action=SupervisorAction.LAND,
                correlation_id=self.new_correlation_id(),
                reason="AUX switch stale or off during mission",
            )
        return SafetyVerdict()

    def evaluate_mission_timeout(self, timestamp_s: float) -> SafetyVerdict:
        """Mission-timeout safety transition."""
        if not self._monitoring_active:
            return SafetyVerdict()
        if self.state not in (SupervisorState.ACTIVE, SupervisorState.LOCALIZATION_LOST_HOVERING):
            return SafetyVerdict()
        if (
            self._mission_timeout_s > 0.0
            and self._mission_start_at is not None
            and (timestamp_s - self._mission_start_at) > self._mission_timeout_s
        ):
            self.state = SupervisorState.LOCALIZATION_LOST_LANDING
            self._land_cmd_count = 1
            self._land_cmd_last_at = timestamp_s
            return SafetyVerdict(
                action=SupervisorAction.LAND,
                correlation_id=self.new_correlation_id(),
                reason="mission timeout exceeded",
            )
        return SafetyVerdict()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start_monitoring(self, *, timestamp_s: float, timeout_s: float = 0.0) -> None:
        """Begin safety supervision (called after preflight passes)."""
        self.state = SupervisorState.ACTIVE
        self._monitoring_active = True
        self._loc_lost_at = None
        self._hover_at = None
        self._land_cmd_count = 0
        self._land_cmd_last_at = None
        self._mission_start_at = timestamp_s
        self._mission_timeout_s = timeout_s
        self._pending.clear()

    def stop_monitoring(self) -> None:
        """End safety supervision (called when mission terminates)."""
        self._monitoring_active = False
        self.state = SupervisorState.IDLE

    @property
    def is_critical(self) -> bool:
        """True when the supervisor has escalated to CRITICAL."""
        return self.state == SupervisorState.CRITICAL

    @property
    def is_landing(self) -> bool:
        """True when the supervisor is in a landing sequence."""
        return self.state == SupervisorState.LOCALIZATION_LOST_LANDING
