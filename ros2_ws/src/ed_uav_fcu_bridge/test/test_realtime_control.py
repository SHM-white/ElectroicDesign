from __future__ import annotations

import importlib
import importlib.util
import struct
import sys
import threading
from pathlib import Path
from types import ModuleType

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.actions import CommandRequest
from ed_uav_fcu_bridge.telemetry import (
    AuxSample,
    LinkSample,
    PositionSample,
    StatusSample,
    TelemetrySnapshot,
)
from ed_uav_fcu_bridge.v7_codec import decode_frame


def _realtime_module() -> ModuleType:
    module_name = "ed_uav_fcu_bridge.realtime_control"
    assert importlib.util.find_spec(module_name) is not None, "missing realtime-control backend module"
    return importlib.import_module(module_name)


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


class BlockingWriter:
    def __init__(self) -> None:
        self.first_entered = threading.Event()
        self.second_entered = threading.Event()
        self.release_first = threading.Event()
        self._guard = threading.Lock()
        self._active = 0
        self._calls = 0
        self.concurrent_entry = False
        self.written: list[bytes] = []

    def __call__(self, data: bytes) -> int:
        with self._guard:
            self._active += 1
            self._calls += 1
            call_number = self._calls
            self.concurrent_entry = self.concurrent_entry or self._active > 1
        if call_number == 1:
            self.first_entered.set()
            assert self.release_first.wait(timeout=1.0)
        else:
            self.second_entered.set()
        self.written.append(data)
        with self._guard:
            self._active -= 1
        return len(data)


def _snapshot(
    *,
    source_sequence: int = 1,
    forward_m: float = 0.0,
    right_m: float = 0.0,
    position_valid: bool = True,
    status_mode: int = 2,
    status_valid: bool = True,
    aux1_us: int = 1500,
    primary_channels_us: tuple[int, int, int, int] = (1500, 1500, 1500, 1500),
    aux_valid: bool = True,
) -> TelemetrySnapshot:
    channels = primary_channels_us + (aux1_us, 1500, 1500, 1500, 1500, 1800)
    return TelemetrySnapshot(
        position=PositionSample(source_sequence, 0.0, forward_m, right_m, position_valid, 0.0, None),
        status=StatusSample(1, 0.0, status_mode, True, status_valid, 0.0, None),
        aux=AuxSample(1, 0.0, channels, aux_valid, 0.0, None),
        flow_diagnostic=None,
        altitude_m=None,
        battery_voltage_v=None,
        link=LinkSample(1, 0.0, True, 0.0),
    )


def _control_fields(raw: bytes) -> tuple[int, ...]:
    frame = decode_frame(raw)
    assert frame.frame_id == 0x41
    return struct.unpack("<7h", frame.data)


@pytest.mark.parametrize(
    ("config_enabled", "snapshot", "expected"),
    (
        (True, _snapshot(), True),
        (False, _snapshot(), False),
        (True, _snapshot(position_valid=False), False),
        (True, _snapshot(status_mode=3), True),
        (True, _snapshot(status_valid=False), True),
        (True, _snapshot(aux1_us=1700), True),
        (True, _snapshot(primary_channels_us=(1300, 1500, 1500, 1500)), True),
        (True, _snapshot(aux_valid=False), True),
    ),
)
def test_nonzero_stream_uses_only_backend_and_position_availability(
    config_enabled: bool,
    snapshot: TelemetrySnapshot,
    expected: bool,
) -> None:
    # Given: one combination of explicit enable and telemetry values.
    realtime = _realtime_module()
    config = realtime.RealtimeControlConfig(enable_realtime_control=config_enabled)

    # When: permission for a nonzero realtime frame is evaluated.
    actual = realtime.nonzero_control_allowed(config, snapshot)

    # Then: obsolete mode/stick/AUX windows do not participate.
    assert actual is expected


def test_move_stream_maps_ros_target_to_forward_and_adjustable_right_speed() -> None:
    # Given: a MOVE from the origin to ROS x=right 0.30 m, y=forward 0.40 m.
    realtime = _realtime_module()
    clock = FakeClock()
    written: list[bytes] = []
    snapshots = SnapshotSequence(
        (
            _snapshot(source_sequence=1),
            _snapshot(source_sequence=2, forward_m=0.40, right_m=0.30),
            _snapshot(source_sequence=3, forward_m=0.40, right_m=0.30),
            _snapshot(source_sequence=4, forward_m=0.40, right_m=0.30),
        )
    )
    config = realtime.RealtimeControlConfig(
        enable_realtime_control=True,
        stop_frame_count=2,
        position_tolerance_m=0.01,
        proportional_gain_cmps_per_m=100.0,
    )
    controller = realtime.RealtimeController(
        realtime.RealtimeDependencies(written.append, snapshots, clock, clock.sleep),
        config,
    )
    request = realtime.RealtimeMoveRequest(
        target=realtime.PositionTarget(forward_m=0.40, right_m=0.30),
        max_speed_cmps=100,
        timeout_s=1.0,
    )

    # When: the closed loop observes three consecutive fresh arrivals at the target.
    result = controller.execute(request, lambda: False)

    # Then: SPD_X is forward, SPD_Y owns one explicit firmware-axis sign, and completion stops.
    assert result.code is realtime.RealtimeResultCode.SUCCEEDED
    assert _control_fields(written[0]) == (
        0,
        0,
        0,
        0,
        40,
        30 * realtime.REALTIME_SPD_Y_SIGN,
        0,
    )
    assert clock.sleeps == [0.02, 0.02, 0.02]
    assert len(written) == 5
    assert [_control_fields(frame) for frame in written[-2:]] == [(0,) * 7, (0,) * 7]


def test_hover_streams_zero_velocity_for_the_requested_duration() -> None:
    # Given: a 50 ms hover, a 20 ms candidate stream period, and two terminal stop frames.
    realtime = _realtime_module()
    clock = FakeClock()
    written: list[bytes] = []
    config = realtime.RealtimeControlConfig(
        enable_realtime_control=True,
        stream_period_s=0.02,
        stop_frame_count=2,
    )
    controller = realtime.RealtimeController(
        realtime.RealtimeDependencies(
            written.append,
            SnapshotSequence((_snapshot(), _snapshot(), _snapshot())),
            clock,
            clock.sleep,
        ),
        config,
    )

    # When: HOVER reaches its monotonic duration without cancellation.
    result = controller.execute(realtime.RealtimeHoverRequest(duration_s=0.05), lambda: False)

    # Then: three streamed frames cover the duration and two configurable stop frames finish it.
    assert result.code is realtime.RealtimeResultCode.SUCCEEDED
    assert clock.sleeps == [0.02, 0.02, 0.02]
    assert len(written) == 5
    assert all(_control_fields(frame) == (0,) * 7 for frame in written)


@pytest.mark.parametrize(
    ("snapshots", "cancel_after_s", "timeout_s", "expected_code"),
    (
        (
            (_snapshot(),),
            0.02,
            1.0,
            "CANCELLED",
        ),
        (
            (_snapshot(), _snapshot(position_valid=False)),
            None,
            1.0,
            "CONTROL_GATED",
        ),
        (
            (_snapshot(),),
            None,
            0.02,
            "TIMEOUT",
        ),
    ),
)
def test_cancel_failure_and_timeout_end_with_configurable_zero_frames(
    snapshots: tuple[TelemetrySnapshot, ...],
    cancel_after_s: float | None,
    timeout_s: float,
    expected_code: str,
) -> None:
    # Given: an active MOVE that will either be canceled or lose its mode gate.
    realtime = _realtime_module()
    clock = FakeClock()
    written: list[bytes] = []
    config = realtime.RealtimeControlConfig(enable_realtime_control=True, stop_frame_count=3)
    controller = realtime.RealtimeController(
        realtime.RealtimeDependencies(written.append, SnapshotSequence(snapshots), clock, clock.sleep),
        config,
    )
    request = realtime.RealtimeMoveRequest(
        target=realtime.PositionTarget(forward_m=1.0, right_m=0.0),
        max_speed_cmps=30,
        timeout_s=timeout_s,
    )

    # When: the terminal condition is observed on the next 20 ms loop.
    result = controller.execute(
        request,
        lambda: cancel_after_s is not None and clock.now >= cancel_after_s,
    )

    # Then: the matching result is returned and exactly three trailing stop frames are sent.
    assert result.code.name == expected_code
    assert clock.sleeps == [0.02]
    assert _control_fields(written[0])[4] > 0
    assert [_control_fields(frame) for frame in written[-3:]] == [(0,) * 7] * 3


def test_hover_ignores_obsolete_mode_gate_during_hover() -> None:
    # Given: telemetry changes FCU mode during a bounded HOVER.
    realtime = _realtime_module()
    clock = FakeClock()
    written: list[bytes] = []
    config = realtime.RealtimeControlConfig(
        enable_realtime_control=True,
        stream_period_s=0.02,
        stop_frame_count=2,
    )
    controller = realtime.RealtimeController(
        realtime.RealtimeDependencies(
            written.append,
            SnapshotSequence((
                _snapshot(),
                _snapshot(),
                _snapshot(),
                _snapshot(status_mode=3),
            )),
            clock,
            clock.sleep,
        ),
        config,
    )

    # When: the hover loop runs through that change.
    result = controller.execute(realtime.RealtimeHoverRequest(duration_s=0.08), lambda: False)

    # Then: it completes and writes its configured trailing stop frames.
    assert result.code is realtime.RealtimeResultCode.SUCCEEDED
    assert clock.sleeps == [0.02, 0.02, 0.02, 0.02]
    assert len(written) == 6
    assert all(_control_fields(frame) == (0,) * 7 for frame in written[-2:])


def test_source_macro_false_restores_legacy_move_and_hover_vectors(monkeypatch: pytest.MonkeyPatch) -> None:
    # Given: realtime MOVE/HOVER is the default source selection.
    realtime = _realtime_module()
    assert realtime.ED_UAV_LINGXIAO_REALTIME_CONTROL is True
    assert realtime.use_realtime_backend(realtime_capable_command=True)

    # When: the source macro is switched off for one-step rollback.
    monkeypatch.setattr(realtime, "ED_UAV_LINGXIAO_REALTIME_CONTROL", False)
    selected = realtime.use_realtime_backend(realtime_capable_command=True)
    move = CommandRequest.move(100, 30, 90).to_frame()
    hover = CommandRequest.hover().to_frame()

    # Then: MOVE/HOVER use the old path and both golden vectors remain byte-identical.
    assert not selected
    assert move.hex().upper() == "AAFFE00B10020364001E005A00000085E7"
    assert hover.hex().upper() == "AAFFE00B1000040000000000000000A8A0"


def test_semantic_arbiter_prevents_concurrent_realtime_requests() -> None:
    # Given: a realtime HOVER blocked inside the first complete frame write.
    realtime = _realtime_module()
    clock = FakeClock()
    written: list[bytes] = []
    snapshots = SnapshotSequence((_snapshot(), _snapshot(), _snapshot()))
    blocking = BlockingWriter()
    arbiter = realtime.CommandArbiter()
    controller = realtime.RealtimeController(
        realtime.RealtimeDependencies(blocking, snapshots, clock, clock.sleep),
        realtime.RealtimeControlConfig(enable_realtime_control=True, stream_period_s=0.02, stop_frame_count=1),
        arbiter,
    )
    first_result: realtime.RealtimeResult | None = None

    def run_first() -> None:
        nonlocal first_result
        first_result = controller.execute(realtime.RealtimeHoverRequest(duration_s=0.02), lambda: False)

    first = threading.Thread(target=run_first)
    first.start()
    assert blocking.first_entered.wait(timeout=1.0)

    # When: a second realtime request arrives while the first request owns the semantic arbiter.
    second = controller.execute(
        realtime.RealtimeMoveRequest(
            target=realtime.PositionTarget(forward_m=1.0, right_m=0.0),
            max_speed_cmps=30,
            timeout_s=0.5,
        ),
        lambda: False,
    )

    # Then: the second request is rejected immediately and the first request completes after release.
    assert second.code is realtime.RealtimeResultCode.REJECTED
    blocking.release_first.set()
    first.join(timeout=1.0)
    assert first_result is not None
    assert first_result.code is realtime.RealtimeResultCode.SUCCEEDED


def test_serialized_writer_prevents_legacy_and_realtime_frame_interleaving() -> None:
    # Given: a bottom-half writer held inside its first complete-frame write.
    realtime = _realtime_module()
    bottom_half = BlockingWriter()
    writer = realtime.SerializedWireWriter(bottom_half)
    legacy_frame = CommandRequest.unlock().to_frame()
    realtime_frame = bytes.fromhex("AAFF410E0000000000000000000000000000F8C5")
    first = threading.Thread(target=writer, args=(legacy_frame,))
    second = threading.Thread(target=writer, args=(realtime_frame,))

    # When: the realtime frame attempts to write while the legacy frame owns the lock.
    first.start()
    assert bottom_half.first_entered.wait(timeout=1.0)
    second.start()
    second_was_blocked = not bottom_half.second_entered.wait(timeout=0.02)
    bottom_half.release_first.set()
    first.join(timeout=1.0)
    second.join(timeout=1.0)

    # Then: the bottom half sees two ordered complete calls and no concurrent entry.
    assert second_was_blocked
    assert not first.is_alive() and not second.is_alive()
    assert not bottom_half.concurrent_entry
    assert bottom_half.written == [legacy_frame, realtime_frame]
