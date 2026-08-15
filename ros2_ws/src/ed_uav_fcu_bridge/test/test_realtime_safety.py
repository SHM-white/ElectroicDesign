from __future__ import annotations

import importlib
import importlib.util
import sys
import threading
from pathlib import Path
from types import ModuleType

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.actions import (
    CommandRejectedError,
    CommandRequest,
    FlightActionController,
)
from ed_uav_fcu_bridge.telemetry import (
    AuxSample,
    LinkSample,
    PositionSample,
    StatusSample,
    TelemetrySnapshot,
)


def _module(name: str) -> ModuleType:
    qualified_name = f"ed_uav_fcu_bridge.{name}"
    assert importlib.util.find_spec(qualified_name) is not None, f"missing {name} module"
    return importlib.import_module(qualified_name)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0
        self.sleeps: list[float] = []

    def __call__(self) -> float:
        return self.now

    def sleep(self, duration_s: float) -> None:
        self.sleeps.append(duration_s)
        self.now += duration_s


class SnapshotSequence:
    def __init__(self, snapshots: tuple[TelemetrySnapshot, ...]) -> None:
        self._snapshots = snapshots
        self._index = 0

    def __call__(self, _steady_now: float) -> TelemetrySnapshot:
        index = min(self._index, len(self._snapshots) - 1)
        self._index += 1
        return self._snapshots[index]


class BlockingSleeper:
    def __init__(self) -> None:
        self.entered = threading.Event()
        self.release = threading.Event()

    def __call__(self, _duration_s: float) -> None:
        self.entered.set()
        assert self.release.wait(timeout=1.0)


def _snapshot(
    *,
    source_sequence: int = 1,
    forward_m: float = 0.0,
    status_mode: int = 2,
    aux1_us: int = 1500,
    primary_channels_us: tuple[int, int, int, int] = (1500, 1500, 1500, 1500),
) -> TelemetrySnapshot:
    channels = primary_channels_us + (aux1_us, 1500, 1500, 1500, 1500, 1800)
    return TelemetrySnapshot(
        position=PositionSample(source_sequence, 0.0, forward_m, 0.0, True, 0.0, None),
        status=StatusSample(1, 0.0, status_mode, True, True, 0.0, None),
        aux=AuxSample(1, 0.0, channels, True, 0.0, None),
        flow_diagnostic=None,
        altitude_m=None,
        battery_voltage_v=None,
        link=LinkSample(1, 0.0, True, 0.0),
    )


def test_manual_channels_and_aux_mode_do_not_gate_realtime_control() -> None:
    # Given: deliberately non-centered sticks and an AUX value outside the old window.
    realtime = _module("realtime_control")
    config = realtime.RealtimeControlConfig(enable_realtime_control=True)

    # When / Then: position validity and the configured backend are sufficient.
    assert realtime.nonzero_control_allowed(
        config,
        _snapshot(primary_channels_us=(1200, 1800, 1100, 1900), aux1_us=1700),
    )


def test_mode_gate_accepts_values_strictly_inside_each_manual_deadband() -> None:
    # Given: roll/pitch, throttle/yaw, and AUX1 are just inside their limits.
    realtime = _module("realtime_control")
    config = realtime.RealtimeControlConfig(enable_realtime_control=True)
    snapshot = _snapshot(
        primary_channels_us=(1461, 1539, 1421, 1579),
        aux1_us=1401,
    )

    # When / Then: the complete fresh mode-2 gate permits control.
    assert realtime.nonzero_control_allowed(config, snapshot)


def test_hover_does_not_abort_on_obsolete_mode_gate_changes() -> None:
    # Given: HOVER sees a mode change that used to be treated as a software lock.
    realtime = _module("realtime_control")
    clock = FakeClock()
    written: list[bytes] = []
    controller = realtime.RealtimeController(
        realtime.RealtimeDependencies(
            written.append,
            SnapshotSequence((_snapshot(), _snapshot(status_mode=3))),
            clock,
            clock.sleep,
        ),
        realtime.RealtimeControlConfig(
            enable_realtime_control=True,
            stop_frame_count=2,
        ),
    )

    # When: HOVER runs for a bounded duration.
    result = controller.execute(realtime.RealtimeHoverRequest(0.04), lambda: False)

    # Then: it completes normally; the independent emergency latch is tested separately.
    assert result.code is realtime.RealtimeResultCode.SUCCEEDED
    assert clock.sleeps == [0.02, 0.02]
    assert len(written) == 4


def test_move_requires_three_distinct_fresh_arrivals_at_target() -> None:
    # Given: MOVE reaches the target once, then sees only the same cached sample.
    realtime = _module("realtime_control")
    clock = FakeClock()
    written: list[bytes] = []
    snapshots = SnapshotSequence(
        (
            _snapshot(source_sequence=1),
            _snapshot(source_sequence=2, forward_m=1.0),
        )
    )
    controller = realtime.RealtimeController(
        realtime.RealtimeDependencies(written.append, snapshots, clock, clock.sleep),
        realtime.RealtimeControlConfig(
            enable_realtime_control=True,
            arrival_confirmation_samples=3,
        ),
    )
    request = realtime.RealtimeMoveRequest(
        realtime.PositionTarget(forward_m=1.0, right_m=0.0),
        30,
        0.08,
    )

    # When: no third distinct source sequence arrives before the deadline.
    result = controller.execute(request, lambda: False)

    # Then: one cached in-tolerance sample cannot complete MOVE.
    assert result.code is realtime.RealtimeResultCode.TIMEOUT


def test_shared_arbiter_rejects_legacy_while_realtime_executes() -> None:
    # Given: a realtime MOVE holding the semantic command lease in another thread.
    realtime = _module("realtime_control")
    arbiter_module = _module("command_arbiter")
    arbiter = arbiter_module.CommandArbiter()
    sleeper = BlockingSleeper()
    cancelled = threading.Event()
    written: list[bytes] = []
    controller = realtime.RealtimeController(
        realtime.RealtimeDependencies(
            written.append,
            SnapshotSequence((_snapshot(),)),
            FakeClock(),
            sleeper,
        ),
        realtime.RealtimeControlConfig(enable_realtime_control=True),
        arbiter,
    )
    legacy = FlightActionController(written.append, arbiter)
    request = realtime.RealtimeMoveRequest(
        realtime.PositionTarget(forward_m=1.0, right_m=0.0),
        30,
        1.0,
    )
    result_codes: list[str] = []
    worker = threading.Thread(
        target=lambda: result_codes.append(
            controller.execute(request, cancelled.is_set).code.name
        )
    )
    worker.start()
    assert sleeper.entered.wait(timeout=1.0)

    # When: a legacy command attempts to start during the active realtime stream.
    with pytest.raises(CommandRejectedError, match="another FCU command"):
        legacy.start(CommandRequest.unlock(), steady_now=0.0, timeout_s=0.5)
    cancelled.set()
    sleeper.release.set()
    worker.join(timeout=1.0)

    # Then: no 0xE0 write overlaps, and the lease is reusable after terminal stops.
    assert not worker.is_alive()
    assert result_codes == ["CANCELLED"]
    assert all(frame[2] == 0x41 for frame in written)
    legacy.start(CommandRequest.unlock(), steady_now=1.0, timeout_s=0.5)
    assert written[-1][2] == 0xE0


def test_shared_arbiter_rejects_realtime_while_legacy_waits_for_ack() -> None:
    # Given: a legacy command owns the shared semantic command lease.
    realtime = _module("realtime_control")
    arbiter_module = _module("command_arbiter")
    arbiter = arbiter_module.CommandArbiter()
    written: list[bytes] = []
    legacy = FlightActionController(written.append, arbiter)
    legacy.start(CommandRequest.unlock(), steady_now=0.0, timeout_s=0.5)
    controller = realtime.RealtimeController(
        realtime.RealtimeDependencies(
            written.append,
            SnapshotSequence((_snapshot(),)),
            FakeClock(),
            FakeClock().sleep,
        ),
        realtime.RealtimeControlConfig(enable_realtime_control=True),
        arbiter,
    )

    # When: realtime HOVER attempts to overlap the outstanding ACK command.
    result = controller.execute(realtime.RealtimeHoverRequest(0.1), lambda: False)

    # Then: it is rejected before any 0x41 frame is written.
    assert result.code is realtime.RealtimeResultCode.REJECTED
    assert [frame[2] for frame in written] == [0xE0]
