from __future__ import annotations

from dataclasses import dataclass

import pytest
from builtin_interfaces.msg import Time
from d_task_fakes import payload_config
from ed_uav_interfaces.action import ExecuteMission
from ed_uav_interfaces.msg import FcuState, LocalizationStatus, MissionStatus
from rclpy.action import GoalResponse

from ed_uav_mission.competition_runtime import CompetitionRuntime
from ed_uav_mission.d_task_events import DTaskEvent, SafetyInterrupted
from ed_uav_mission.d_task_model import DTaskFault, DTaskKind, DTaskPhase, DTaskSelection, RouteStage
from ed_uav_mission.d_task_ros import DTaskRosBoundary
from ed_uav_mission.executor import MissionExecutorNode, PreflightCode, validate_preflight
from ed_uav_mission.mission_model import CompetitionParams, MissionConfig, MissionType, StabilityParams
from ed_uav_mission.stability_runner import StabilityRunner
from ed_uav_mission.state_machine import MissionFSM
from test_d_task_action_runtime import FakeActionSurface, RecordingStabilityCallbacks, _run_immediate


def _stability_config() -> MissionConfig:
    return MissionConfig(
        version=1,
        mission_id="task3-stability",
        mission_type=MissionType.STABILITY_TEST,
        field_profile_id="task3-field",
        stability_params=StabilityParams(),
    )


def _task3_selection(task: DTaskKind) -> DTaskSelection:
    return DTaskSelection(
        mission_id="task3-stability",
        mission_profile_id="task3-profile",
        deployment_preset_id="simulation",
        target_revision="d2026-circle-cross-v1",
        task=task,
        committed_at_s=0.0,
    )


@dataclass(frozen=True, slots=True)
class _Profile:
    profile_id: str = "task3-field"


@dataclass(frozen=True, slots=True)
class _SelectionBoundary:
    selection: DTaskSelection | None


class _Logger:
    def info(self, _message: str) -> None:
        return None


class _GoalHarness:
    def __init__(self, committed_selection: DTaskSelection | None) -> None:
        self._fsm = MissionFSM()
        self._mission_config = _stability_config()
        self._profile = _Profile()
        self._d_task_boundary = _SelectionBoundary(committed_selection)

    @staticmethod
    def get_logger() -> _Logger:
        return _Logger()


class _PreflightHarness:
    def __init__(self) -> None:
        fcu = FcuState()
        fcu.communication_ok = True
        fcu.source = FcuState.SOURCE_SIMULATOR
        fcu.motors_armed = True
        localization = LocalizationStatus()
        localization.state = LocalizationStatus.STATE_ACTIVE
        localization.map_to_odom_valid = True
        self._latest_fcu = fcu
        self._latest_localization = localization
        self._simulation_only = True
        self._profile = _Profile()
        self._calibration_valid = True
        self._mission_config = _stability_config()


class _StatusPublisher:
    def __init__(self) -> None:
        self.messages: list[MissionStatus] = []

    def publish(self, message: MissionStatus) -> None:
        self.messages.append(message)


class _ClockNow:
    @staticmethod
    def to_msg() -> Time:
        return Time()


class _BoundaryNode:
    def __init__(self) -> None:
        self.status_publisher = _StatusPublisher()

    @staticmethod
    def create_subscription(*_args, **_kwargs) -> None:
        return None

    @staticmethod
    def create_service(*_args, **_kwargs) -> None:
        return None

    def create_publisher(self, *_args, **_kwargs) -> _StatusPublisher:
        return self.status_publisher

    @staticmethod
    def create_timer(*_args, **_kwargs) -> None:
        return None

    @staticmethod
    def get_clock() -> _BoundaryNode:
        return _BoundaryNode()

    @staticmethod
    def now() -> _ClockNow:
        return _ClockNow()


class _HardLockStabilityCallbacks(RecordingStabilityCallbacks):
    def __init__(self, event: DTaskEvent) -> None:
        super().__init__()
        self._event = event
        self.landing_requests = 0

    async def next_event(self) -> DTaskEvent:
        return self._event

    async def land_home(self, feedback: ExecuteMission.Feedback) -> None:
        self.landing_requests += 1
        await super().land_home(feedback)


class _RecoveryHarness:
    def __init__(self) -> None:
        self._airborne = True
        self._hard_lock_active = True
        self.recovery_calls: list[str] = []

    async def _send_hover(self, _duration_sec: float, *, recovery: bool) -> None:
        assert recovery is True
        self.recovery_calls.append("hover")

    async def _send_land(self, *, recovery: bool) -> None:
        assert recovery is True
        self.recovery_calls.append("land")

    async def _send_disarm(self, *, recovery: bool) -> None:
        assert recovery is True
        self.recovery_calls.append("disarm")


def _goal_request() -> ExecuteMission.Goal:
    request = ExecuteMission.Goal()
    request.mission_id = "task3-stability"
    request.field_profile_id = "task3-field"
    return request


@pytest.mark.parametrize(
    "committed_selection",
    [None, _task3_selection(DTaskKind.PAYLOAD_DROP), _task3_selection(DTaskKind.DYNAMIC_LANDING)],
    ids=["missing", "task1", "task2"],
)
def test_stability_goal_requires_committed_task3_selection(
    committed_selection: DTaskSelection | None,
) -> None:
    # Given: a Task3 mission with no selection or an adjacent Task1/Task2 selection.
    executor = _GoalHarness(committed_selection)

    # When: the mission action receives its loaded mission goal.
    response = MissionExecutorNode._on_goal(executor, _goal_request())

    # Then: only a committed Task3 selection may arm this mission path.
    assert response is GoalResponse.REJECT


def test_stability_goal_accepts_committed_task3_selection() -> None:
    # Given: a Task3 mission and a matching immutable Task3 selection.
    executor = _GoalHarness(_task3_selection(DTaskKind.STABILITY_TEST))

    # When: the mission action receives its loaded mission goal.
    response = MissionExecutorNode._on_goal(executor, _goal_request())

    # Then: the committed Task3 selection is eligible to proceed to preflight.
    assert response is GoalResponse.ACCEPT


@pytest.mark.parametrize(
    ("gate", "expected"),
    [
        ("fcu_communication_ok", PreflightCode.NO_FCU_LINK),
        ("fcu_motors_armed", PreflightCode.MOTORS_NOT_ARMED),
        ("localization_active", PreflightCode.LOCALIZATION_LOST),
        ("map_to_odom_valid", PreflightCode.LOCALIZATION_LOST),
        ("profile_loaded", PreflightCode.PROFILE_INVALID),
        ("calibration_valid", PreflightCode.CALIBRATION_MISSING),
    ],
)
def test_task3_preflight_requires_core_flight_state(
    gate: str,
    expected: PreflightCode,
) -> None:
    # Given: every Task3 preflight input is valid except one named safety gate.
    preflight = {
        "fcu_communication_ok": True,
        "fcu_source": FcuState.SOURCE_SIMULATOR,
        "fcu_motors_armed": True,
        "simulation_only": True,
        "localization_active": True,
        "map_to_odom_valid": True,
        "profile_loaded": True,
        "calibration_valid": True,
    }
    preflight[gate] = False

    # When: the pure preflight boundary evaluates the gate set.
    result = validate_preflight(**preflight)

    # Then: no missing flight-state input authorizes Task3 execution.
    assert result.code is expected


def test_task3_executor_preflight_does_not_require_aux_permission() -> None:
    # Given: FCU, motors, localization, profile, and calibration are valid.
    executor = _PreflightHarness()

    # When: the executor wires observed state into its preflight check.
    result = MissionExecutorNode._run_preflight(executor)

    # Then: the common mission preflight accepts without an AUX start gate.
    assert result.code is PreflightCode.OK


def test_task3_runtime_runs_without_competition_params() -> None:
    # Given: the unchanged StabilityRunner callback surface and a committed Task3 selection.
    stability_callbacks = RecordingStabilityCallbacks()
    runtime = CompetitionRuntime(
        FakeActionSurface(events=[]).callbacks(),
        payload_config(),
        stability_callbacks=stability_callbacks,
    )
    failure: RuntimeError | None = None

    # When: Task3 supplies its stability parameters without competition parameters.
    async def scenario() -> None:
        await runtime.run(
            None,
            _task3_selection(DTaskKind.STABILITY_TEST),
            ExecuteMission.Feedback(),
            stability_params=StabilityParams(),
        )

    try:
        _run_immediate(scenario())
    except RuntimeError as error:
        failure = error

    # Then: the existing runner completes its 17 motion commands without a competition branch.
    assert failure is None, f"Task3 runtime rejected stability-only parameters: {failure}"
    assert len(stability_callbacks.moves) == 17


def test_task3_boundary_publishes_initial_pre_arm_status() -> None:
    # Given: a new mission boundary before a vehicle bridge has made a selection decision.
    node = _BoundaryNode()

    # When: the Task3 boundary is created for its selection service.
    DTaskRosBoundary(
        node,
        "task3-stability",
        CompetitionParams(
            mission_profile_id="task3-profile",
            deployment_preset_id="simulation",
            target_revision="d2026-circle-cross-v1",
        ),
        lambda: True,
        lambda: True,
    )

    # Then: the bridge receives the idle PRE_ARM state that permits a selection request.
    assert len(node.status_publisher.messages) == 1
    status = node.status_publisher.messages[0]
    assert status.state == MissionStatus.STATE_PRE_ARM
    assert status.complete is False


def test_task3_geometry_remains_four_square_and_thirteen_circle_moves() -> None:
    # Given: the existing Task3 runner and its default frozen geometry parameters.
    runner = StabilityRunner(RecordingStabilityCallbacks(), StabilityParams())

    # When: square and circle moves are derived from the same map pose.
    square = runner._square_waypoints(0.0, 0.0, 0.0)
    circle = runner._circle_waypoints(0.0, 0.0, 0.0)

    # Then: the trajectory stays at four square corners plus thirteen circle segments.
    assert [waypoint.label for waypoint in square] == [
        "stability_square_1",
        "stability_square_2",
        "stability_square_3",
        "stability_square_4",
    ]
    assert [waypoint.label for waypoint in circle] == [
        f"stability_circle_{index}" for index in range(1, 14)
    ]


def test_task3_hard_lock_aborts_without_post_lock_recovery() -> None:
    # Given: an active Task3 runner receives an explicit hard-lock safety indication.
    hard_lock = getattr(DTaskFault, "HARD_LOCKED", None)
    assert hard_lock is not None, "DTaskFault.HARD_LOCKED must represent the physical hard lock"
    callbacks = _HardLockStabilityCallbacks(
        SafetyInterrupted(1.0, hard_lock, "physical hard lock asserted")
    )
    runner = StabilityRunner(callbacks, StabilityParams())

    # When: the first square waypoint checks the active safety event stream.
    with pytest.raises(RuntimeError, match="hard lock"):
        _run_immediate(runner.run(_task3_selection(DTaskKind.STABILITY_TEST), ExecuteMission.Feedback()))

    # Then: no post-lock hover or land command continues the stability mission.
    assert callbacks.moves == []
    assert callbacks.hovers == [5.0]
    assert callbacks.landing_requests == 0
    assert callbacks.phases[-1] is DTaskPhase.ABORTED


def test_hard_lock_suppresses_executor_hover_land_and_disarm_recovery() -> None:
    # Given: an airborne executor has received the explicit hard-lock indication.
    executor = _RecoveryHarness()

    # When: the action boundary handles the terminal stability fault.
    _run_immediate(MissionExecutorNode._recover_after_airborne_failure(executor))

    # Then: it never issues hover, land, or disarm recovery after the physical lock.
    assert executor.recovery_calls == []
