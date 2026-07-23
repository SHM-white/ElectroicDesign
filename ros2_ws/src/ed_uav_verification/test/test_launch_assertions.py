from __future__ import annotations

from ed_uav_verification.assertions import LaunchAssertions
from ed_uav_verification.faults import FaultKind, FaultWindow
from ed_uav_verification.model import ScenarioConfig, Stream
from ed_uav_verification.scenario import DeterministicScenario


def test_launch_assertions_reject_regressed_clock_and_dead_lidar() -> None:
    """Given time rollback and process death, when replay completes, then both are asserted."""
    config = ScenarioConfig(
        seed=31,
        duration_seconds=2,
        rate_hz=20,
        faults=(
            FaultWindow(FaultKind.TIME_REGRESSION, Stream.LIDAR_IMU, start_tick=5, duration_ticks=4),
            FaultWindow(FaultKind.PROCESS_DEATH, Stream.LIDAR_POINTS, start_tick=10, duration_ticks=4),
        ),
    )

    report = DeterministicScenario(config).run()

    LaunchAssertions(report).assert_fault_matrix(config.faults)
    LaunchAssertions(report).assert_no_stale_reuse()
