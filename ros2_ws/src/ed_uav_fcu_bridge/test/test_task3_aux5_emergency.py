from __future__ import annotations

import struct
import sys
import threading
from pathlib import Path
from typing import Final

import pytest

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.actions import CommandRequest
from ed_uav_fcu_bridge.realtime_control import RealtimeDependencies
from ed_uav_fcu_bridge.realtime_policy import (
    RealtimeControlConfig,
    RealtimeHoverRequest,
    nonzero_control_allowed,
)
from ed_uav_fcu_bridge.session import BridgeConfig, NativeV7Bridge
from ed_uav_fcu_bridge.telemetry import FreshnessPolicy, TelemetryCache, TelemetrySnapshot
from ed_uav_fcu_bridge.v7_codec import build_frame, cmd_lock, decode_frame


def _aux1_frame(aux1_us: int) -> bytes:
    channels = (1500, 1500, 1500, 1500, aux1_us, 1601, 1602, 1603, 1604, 1605)
    return build_frame(0xFF, 0x40, struct.pack("<10h", *channels))


def _fresh_task3_snapshot(aux1_us: int) -> TelemetrySnapshot:
    cache = TelemetryCache(FreshnessPolicy())
    assert cache.ingest_raw(build_frame(0xFF, 0x08, struct.pack("<ii", 0, 0)), 10.0)
    assert cache.ingest_raw(build_frame(0xFF, 0x06, bytes((2, 1))), 10.0)
    assert cache.ingest_raw(_aux1_frame(aux1_us), 10.0)
    return cache.snapshot(steady_now=10.1)


def _bridge(written: list[bytes], *, realtime: bool = False) -> NativeV7Bridge:
    return NativeV7Bridge(
        written.append,
        BridgeConfig(
            realtime_control=RealtimeControlConfig(
                enable_realtime_control=realtime,
                stop_frame_count=1,
            )
        ),
    )


class DeterministicClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, duration_s: float) -> None:
        self.now += duration_s


class BlockingSleeper:
    def __init__(self, clock: DeterministicClock) -> None:
        self.entered = threading.Event()
        self.release = threading.Event()
        self._clock = clock

    def __call__(self, duration_s: float) -> None:
        self.entered.set()
        assert self.release.wait(timeout=1.0)
        self._clock.advance(duration_s)


def test_complete_0x40_maps_index_4_to_rc_channel_5_aux1() -> None:
    # Given: one checksum-valid, complete 10-channel V7 RC frame with distinct later channels.
    cache = TelemetryCache(FreshnessPolicy())
    raw = _aux1_frame(1510)
    assert decode_frame(raw).frame_id == 0x40
    assert cache.ingest_raw(raw, steady_now=1.0)

    # When: the fresh telemetry state is observed.
    snapshot = cache.snapshot(steady_now=1.1)

    # Then: channel index 4 is exposed as manual AUX1/channel 5, never one of the later AUX channels.
    assert snapshot.aux is not None
    assert snapshot.aux.channels_us[4] == 1510
    assert snapshot.aux.aux1_us == 1510
    assert snapshot.aux.aux1_us != snapshot.aux.channels_us[5]


@pytest.mark.parametrize("aux1_us", (1200, 1400, 1500, 1600, 1799))
def test_non_emergency_aux1_values_do_not_gate_realtime_mode(aux1_us: int) -> None:
    # Given: fresh position telemetry and a non-emergency AUX1 value.
    snapshot = _fresh_task3_snapshot(aux1_us)
    config = RealtimeControlConfig(enable_realtime_control=True)

    # When: realtime permission is evaluated.
    actual = nonzero_control_allowed(config, snapshot)

    # Then: AUX1 is not a mode gate; only 1800..2000 is handled by the latch below.
    assert actual is True


@pytest.mark.parametrize(
    ("aux1_us", "locks"),
    (
        pytest.param(1200, False, id="failsafe-low-bound"),
        pytest.param(1300, False, id="failsafe-low-interior"),
        pytest.param(1400, False, id="failsafe-low-upper-bound"),
        pytest.param(1600, False, id="failsafe-high-lower-bound"),
        pytest.param(1700, False, id="failsafe-high-interior"),
        pytest.param(1799, False, id="below-hard-lock-range"),
        pytest.param(1800, True, id="hard-lock-lower-bound"),
        pytest.param(1900, True, id="hard-lock-interior"),
        pytest.param(2000, True, id="hard-lock-upper-bound"),
        pytest.param(2001, False, id="above-hard-lock-range"),
    ),
)
def test_only_fresh_aux1_hard_lock_range_latches_the_serial_owner(
    aux1_us: int,
    locks: bool,
) -> None:
    # Given: a serial-owning bridge with a checksum-valid, complete RC frame.
    written: list[bytes] = []
    bridge = _bridge(written)

    # When: channel 5/AUX1 is received at the bridge boundary.
    bridge.feed(_aux1_frame(aux1_us), steady_now=2.0)

    # Then: only the inclusive 1800..2000 hard-lock range changes the latched owner state.
    assert getattr(bridge, "emergency_lock_latched", False) is locks, (
        "NativeV7Bridge must expose emergency_lock_latched from fresh complete AUX1"
    )
    assert tuple(written) == ((cmd_lock(),) if locks else ())


def test_stale_malformed_and_incomplete_aux1_frames_never_latch_a_hard_lock() -> None:
    # Given: three bridges holding an old frame, a bad-checksum frame, or a short valid frame.
    stale_written: list[bytes] = []
    stale_bridge = _bridge(stale_written)
    assert stale_bridge.telemetry.ingest_raw(_aux1_frame(1900), steady_now=0.0)
    malformed = bytearray(_aux1_frame(1900))
    malformed[-1] ^= 0xFF
    malformed_written: list[bytes] = []
    malformed_bridge = _bridge(malformed_written)
    incomplete_written: list[bytes] = []
    incomplete_bridge = _bridge(incomplete_written)
    incomplete = build_frame(0xFF, 0x40, struct.pack("<5h", 1500, 1500, 1500, 1500, 1900))

    # When: stale cached state is revisited and malformed or incomplete serial input arrives.
    stale_bridge.feed(build_frame(0xFF, 0x0D, struct.pack("<H", 1200)), steady_now=0.501)
    malformed_bridge.feed(bytes(malformed), steady_now=1.0)
    incomplete_bridge.feed(incomplete, steady_now=1.0)

    # Then: none may create the safety latch or emit a motor-lock command.
    for bridge, written in (
        (stale_bridge, stale_written),
        (malformed_bridge, malformed_written),
        (incomplete_bridge, incomplete_written),
    ):
        assert getattr(bridge, "emergency_lock_latched", False) is False
        assert written == []


def test_hard_lock_is_latched_and_emits_one_decodable_cmd_lock_frame() -> None:
    # Given: a serial owner that sees a hard-lock AUX1 frame followed by safe and repeated high values.
    written: list[bytes] = []
    bridge = _bridge(written)

    # When: the complete frames cross the hard-lock range and later leave it.
    bridge.feed(_aux1_frame(1900), steady_now=3.0)
    bridge.feed(_aux1_frame(1950), steady_now=3.1)
    bridge.feed(_aux1_frame(1500), steady_now=3.2)

    # Then: one persistent lock transition produces exactly the native one-key lock command.
    assert getattr(bridge, "emergency_lock_latched", False) is True, (
        "fresh complete AUX1 hard-lock input must latch at the serial owner"
    )
    assert written == [cmd_lock()]
    frame = decode_frame(written[0])
    assert frame.frame_id == 0xE0
    assert frame.data[:3] == bytes((0x10, 0x00, 0x02))


def test_hard_lock_preempts_pending_ack_command() -> None:
    # Given: an ACK-waiting high-level command owns the shared FCU command lease.
    written: list[bytes] = []
    bridge = _bridge(written)
    pending = bridge.start(CommandRequest.hover(), steady_now=4.0, timeout_s=1.0)

    # When: a complete AUX1 emergency-lock frame arrives before its acknowledgement.
    bridge.feed(_aux1_frame(1900), steady_now=4.1)

    # Then: the pending command has been preempted and the lock command follows its existing write.
    assert getattr(bridge, "emergency_lock_latched", False) is True, (
        "hard lock must preempt an ACK-waiting command at the serial owner"
    )
    assert bridge.actions.pending is None
    assert written == [pending.raw, cmd_lock()]


def test_hard_lock_preempts_active_realtime_without_post_lock_0x41() -> None:
    # Given: a valid realtime HOVER blocked after its first 0x41 frame is fully written.
    written: list[bytes] = []
    bridge = _bridge(written, realtime=True)
    bridge.feed(build_frame(0xFF, 0x08, struct.pack("<ii", 0, 0)), steady_now=0.0)
    bridge.feed(build_frame(0xFF, 0x06, bytes((2, 1))), steady_now=0.0)
    bridge.feed(_aux1_frame(1500), steady_now=0.0)
    clock = DeterministicClock()
    sleeper = BlockingSleeper(clock)
    dependencies = bridge.realtime._dependencies
    bridge.realtime._dependencies = RealtimeDependencies(
        dependencies.writer,
        bridge.snapshot,
        clock,
        sleeper,
    )
    worker = threading.Thread(
        target=lambda: bridge.realtime.execute(RealtimeHoverRequest(0.04), lambda: False),
    )
    worker.start()
    assert sleeper.entered.wait(timeout=1.0)

    # When: serial input latches the hard lock while the realtime command holds the arbiter.
    try:
        bridge.feed(_aux1_frame(1900), steady_now=0.01)
        assert getattr(bridge, "emergency_lock_latched", False) is True, (
            "hard lock must preempt active realtime despite its command lease"
        )
        lock_index = written.index(cmd_lock())
    finally:
        sleeper.release.set()
        worker.join(timeout=1.0)

    # Then: the realtime worker terminates and no control frame follows the lock on the serial wire.
    assert not worker.is_alive()
    assert all(decode_frame(raw).frame_id != 0x41 for raw in written[lock_index + 1 :])
