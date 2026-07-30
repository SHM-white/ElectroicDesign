from __future__ import annotations

import struct
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.telemetry import FreshnessPolicy, TelemetryCache
from ed_uav_fcu_bridge.v7_codec import build_frame


def position_frame(x_cm: int, y_cm: int) -> bytes:
    return build_frame(0xFF, 0x08, struct.pack("<ii", x_cm, y_cm))


def aux_frame(
    aux6_us: int,
    *,
    aux1_us: int = 1500,
    primary_channels_us: tuple[int, int, int, int] = (1500, 1500, 1500, 1500),
) -> bytes:
    channels = [1500] * 10
    channels[:4] = primary_channels_us
    channels[4] = aux1_us
    channels[9] = aux6_us
    return build_frame(0xFF, 0x40, struct.pack("<10h", *channels))


def test_0x08_is_the_only_continuous_position_cache() -> None:
    # Given: two 0x08 positions surrounding a large, valid mode-2 0x51 diagnostic.
    cache = TelemetryCache(FreshnessPolicy())
    cache.ingest_raw(position_frame(100, -200), steady_now=10.0)
    diagnostic_data = bytes((2, 1)) + struct.pack(
        "<hhhhhhB", 1, 2, 3, 4, 30000, -30000, 200
    ) + bytes(240)
    cache.ingest_raw(build_frame(0xFF, 0x51, diagnostic_data), steady_now=10.1)
    cache.ingest_raw(position_frame(125, -180), steady_now=10.2)

    # When: state is sampled after all three frames.
    snapshot = cache.snapshot(steady_now=10.25)

    # Then: position came only from 0x08 while 0x51 remains separately stamped.
    assert snapshot.position is not None
    assert (snapshot.position.forward_m, snapshot.position.right_m) == (1.25, -1.8)
    assert snapshot.position.source_sequence == 2
    assert snapshot.flow_diagnostic is not None
    assert snapshot.flow_diagnostic.integrated_x_cm == 30000
    assert snapshot.flow_diagnostic.source_sequence == 1


def test_stale_0x08_position_becomes_invalid_using_steady_age() -> None:
    # Given: a single 0x08 sample with the frozen 0.20 second freshness default.
    cache = TelemetryCache(FreshnessPolicy())
    cache.ingest_raw(position_frame(1, 2), steady_now=3.0)

    # When: time crosses its steady freshness deadline.
    fresh = cache.snapshot(steady_now=3.20)
    stale = cache.snapshot(steady_now=3.201)

    # Then: the sample age is explicit and stale data is not presented as valid position.
    assert fresh.position is not None and fresh.position.valid
    assert stale.position is not None and not stale.position.valid
    assert stale.position.steady_age_s > 0.20


def test_aux_start_permission_requires_a_fresh_rc_frame() -> None:
    # Given: a high AUX6 switch value and a 0.50 second AUX/status policy.
    cache = TelemetryCache(FreshnessPolicy())
    cache.ingest_raw(aux_frame(1800), steady_now=4.0)

    # When: start permission is queried on either side of its deadline.
    fresh = cache.has_fresh_start_switch(steady_now=4.50)
    stale = cache.has_fresh_start_switch(steady_now=4.501)

    # Then: stale AUX cannot authorize mission start.
    assert fresh
    assert not stale


def test_0x40_retains_all_channels_for_realtime_mode_gating() -> None:
    # Given: one complete RC frame with distinct primary, AUX1, and AUX6 values.
    cache = TelemetryCache(FreshnessPolicy())
    expected_channels = (1450, 1500, 1550, 1490, 1510, 1500, 1500, 1500, 1500, 1800)
    cache.ingest_raw(
        aux_frame(
            1800,
            aux1_us=1510,
            primary_channels_us=(1450, 1500, 1550, 1490),
        ),
        steady_now=5.0,
    )

    # When: the fresh telemetry snapshot is read.
    snapshot = cache.snapshot(steady_now=5.1)

    # Then: all ten channels remain available while AUX6 keeps its start semantics.
    assert snapshot.aux is not None
    assert snapshot.aux.channels_us == expected_channels
    assert snapshot.aux.aux1_us == 1510
    assert snapshot.aux.aux6_us == 1800
    assert cache.has_fresh_start_switch(steady_now=5.1)


def test_status_and_link_track_source_sequence_and_steady_age_independently() -> None:
    # Given: a status frame and an unrelated valid V7 frame.
    cache = TelemetryCache(FreshnessPolicy())
    cache.ingest_raw(build_frame(0xFF, 0x06, bytes((3, 1, 0, 0, 0))), steady_now=7.0)
    cache.ingest_raw(build_frame(0xFF, 0x0D, struct.pack("<H", 1240)), steady_now=7.1)

    # When: a snapshot is read.
    snapshot = cache.snapshot(steady_now=7.2)

    # Then: state provenance and sequence do not reuse position or diagnostic counters.
    assert snapshot.status is not None
    assert snapshot.status.source_sequence == 1
    assert snapshot.status.steady_age_s == pytest.approx(0.2)
    assert snapshot.link.source_sequence == 2
    assert snapshot.link.valid
