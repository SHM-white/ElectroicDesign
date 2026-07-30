"""Immutable domain model for the two 2026 D-task mission branches."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum, IntEnum
from threading import Lock
from types import MappingProxyType
from typing import Final, Literal


class DTaskKind(IntEnum):
    PAYLOAD_DROP = 1
    DYNAMIC_LANDING = 2


class DTaskPhase(str, Enum):
    PRE_ARM = "pre_arm"
    WAITING_START = "waiting_start"
    TAKEOFF = "takeoff"
    STABILIZING = "stabilizing"
    ACQUIRING = "acquiring"
    ESCORTING = "escorting"
    TRACKING = "tracking"
    RELEASING = "releasing"
    DESCENDING = "descending"
    VEHICLE_DWELL = "vehicle_dwell"
    RETAKEOFF = "retakeoff"
    RETURNING_HOME = "returning_home"
    LANDING_HOME = "landing_home"
    SAFE_HOVER = "safe_hover"
    SAFE_RETURN = "safe_return"
    SAFE_LAND = "safe_land"
    STABILITY_PRE_HOVER = "stability_pre_hover"
    STABILITY_SQUARE = "stability_square"
    STABILITY_CIRCLE = "stability_circle"
    STABILITY_POST_HOVER = "stability_post_hover"
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"


class DTaskEffect(str, Enum):
    TAKEOFF = "takeoff"
    HOVER = "hover"
    TRACK_TARGET = "track_target"
    RELEASE_PAYLOAD = "release_payload"
    DESCEND_TO_VEHICLE = "descend_to_vehicle"
    RETURN_HOME = "return_home"
    LAND_HOME = "land_home"
    STABILITY_HOVER = "stability_hover"
    STABILITY_WAYPOINT = "stability_waypoint"


class DTaskFault(str, Enum):
    NEVER_STARTED = "never_started"
    SELECTION_MISSING = "selection_missing"
    SELECTION_MISMATCH = "selection_mismatch"
    B_DEADLINE_MISSED = "b_deadline_missed"
    D_DEADLINE_MISSED = "d_deadline_missed"
    MISSION_DEADLINE_MISSED = "mission_deadline_missed"
    TARGET_STALE = "target_stale"
    TARGET_OUTLIER = "target_outlier"
    VEHICLE_LOST = "vehicle_lost"
    PAYLOAD_UNKNOWN = "payload_unknown"
    CONTACT_INTERRUPTED = "contact_interrupted"
    FLIGHT_COMMAND_FAILED = "flight_command_failed"
    CAPABILITY_BLOCKED = "capability_blocked"
    CANCELLED = "cancelled"
    LOCALIZATION_LOST = "localization_lost"


class RouteStage(IntEnum):
    START = 0
    B = 1
    D = 2
    A = 3
    COMPLETE = 4


class PayloadState(IntEnum):
    UNKNOWN = 0
    SECURED = 1
    RELEASED = 2


@dataclass(frozen=True, slots=True)
class DTaskBranch:
    task: DTaskKind
    nominal_phases: tuple[DTaskPhase, ...]


D_TASK_BRANCHES: Final[Mapping[DTaskKind, DTaskBranch]] = MappingProxyType(
    {
        DTaskKind.PAYLOAD_DROP: DTaskBranch(
            task=DTaskKind.PAYLOAD_DROP,
            nominal_phases=(
                DTaskPhase.WAITING_START,
                DTaskPhase.TAKEOFF,
                DTaskPhase.STABILIZING,
                DTaskPhase.ACQUIRING,
                DTaskPhase.ESCORTING,
                DTaskPhase.RELEASING,
                DTaskPhase.RETURNING_HOME,
                DTaskPhase.LANDING_HOME,
                DTaskPhase.SUCCEEDED,
            ),
        ),
        DTaskKind.DYNAMIC_LANDING: DTaskBranch(
            task=DTaskKind.DYNAMIC_LANDING,
            nominal_phases=(
                DTaskPhase.WAITING_START,
                DTaskPhase.TAKEOFF,
                DTaskPhase.STABILIZING,
                DTaskPhase.ACQUIRING,
                DTaskPhase.TRACKING,
                DTaskPhase.DESCENDING,
                DTaskPhase.VEHICLE_DWELL,
                DTaskPhase.RETAKEOFF,
                DTaskPhase.RETURNING_HOME,
                DTaskPhase.LANDING_HOME,
                DTaskPhase.SUCCEEDED,
            ),
        ),
    }
)


@dataclass(frozen=True, slots=True)
class DTaskSelection:
    mission_id: str
    mission_profile_id: str
    deployment_preset_id: str
    target_revision: str
    task: DTaskKind
    committed_at_s: float


@dataclass(frozen=True, slots=True)
class DTaskState:
    phase: DTaskPhase
    task: DTaskKind
    phase_started_at_s: float
    mission_started_at_s: float | None = None
    release_attempted: bool = False
    fault: DTaskFault | None = None
    reason: str = ""


@dataclass(frozen=True, slots=True)
class DTaskTransition:
    state: DTaskState
    effect: DTaskEffect | None = None
    complete: bool = False


@dataclass(frozen=True, slots=True)
class SelectionAccepted:
    selection: DTaskSelection
    accepted: Literal[True] = True


@dataclass(frozen=True, slots=True)
class SelectionRejected:
    reason: str
    accepted: Literal[False] = False


SelectionResult = SelectionAccepted | SelectionRejected


class SelectionStore:
    """Atomically commit one immutable pre-arm selection for a mission run."""

    def __init__(self) -> None:
        self._lock: Lock = Lock()
        self._selection: DTaskSelection | None = None

    @property
    def selection(self) -> DTaskSelection | None:
        return self._selection

    def commit(self, selection: DTaskSelection, *, pre_arm: bool) -> SelectionResult:
        with self._lock:
            if not pre_arm:
                return SelectionRejected(reason="mission selection is pre-arm only")
            if self._selection is not None:
                return SelectionRejected(reason="mission selection is already committed")
            self._selection = selection
            return SelectionAccepted(selection=selection)

    def clear_after_terminal(self) -> None:
        with self._lock:
            self._selection = None
