"""Tests for localization source supervisor switching logic."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.source_supervisor import (
    LocalizationSource,
    SourceState,
    SupervisorThresholds,
    decide_source_switch,
    evaluate_source_state,
    is_visual_stable,
    poses_aligned,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _default_thresholds(**overrides: float | int) -> SupervisorThresholds:
    kwargs: dict[str, float | int] = {
        "lio_max_age_active": 0.15,
        "lio_max_age_degraded": 0.50,
        "visual_max_age_active": 0.20,
        "visual_max_age_degraded": 0.50,
        "lost_timeout": 1.0,
        "covariance_blowup": 1e6,
        "visual_stability_duration": 0.5,
        "visual_consecutive_samples": 5,
        "primary_hysteresis": 2.0,
        "max_switch_position_diff_m": 0.25,
        "max_switch_yaw_diff_rad": math.radians(10.0),
    }
    kwargs.update(overrides)
    return SupervisorThresholds(**kwargs)  # type: ignore[arg-type]


def _active_state() -> SourceState:
    return SourceState.ACTIVE


def _degraded_state() -> SourceState:
    return SourceState.DEGRADED


def _lost_state() -> SourceState:
    return SourceState.LOST


# ---------------------------------------------------------------------------
# evaluate_source_state
# ---------------------------------------------------------------------------


def test_source_active_when_fresh() -> None:
    """Fresh inputs with finite covariance yield ACTIVE."""
    state = evaluate_source_state(
        age_sec=0.05,
        no_msg_duration_sec=0.05,
        time_regression=False,
        covariance_finite=True,
        covariance_exceeds=False,
        max_age_active=0.15,
        max_age_degraded=0.50,
        lost_timeout=1.0,
        covariance_blowup=1e6,
    )
    assert state == SourceState.ACTIVE


def test_source_degraded_when_stale() -> None:
    """An input older than max_age_active but within max_age_degraded yields DEGRADED."""
    state = evaluate_source_state(
        age_sec=0.30,
        no_msg_duration_sec=0.30,
        time_regression=False,
        covariance_finite=True,
        covariance_exceeds=False,
        max_age_active=0.15,
        max_age_degraded=0.50,
        lost_timeout=1.0,
        covariance_blowup=1e6,
    )
    assert state == SourceState.DEGRADED


def test_source_lost_when_no_messages() -> None:
    """No input at all yields LOST."""
    state = evaluate_source_state(
        age_sec=None,
        no_msg_duration_sec=None,
        time_regression=False,
        covariance_finite=True,
        covariance_exceeds=False,
        max_age_active=0.15,
        max_age_degraded=0.50,
        lost_timeout=1.0,
        covariance_blowup=1e6,
    )
    assert state == SourceState.LOST


def test_source_lost_on_timeout() -> None:
    """No valid messages for longer than lost_timeout yields LOST."""
    state = evaluate_source_state(
        age_sec=0.05,
        no_msg_duration_sec=1.50,
        time_regression=False,
        covariance_finite=True,
        covariance_exceeds=False,
        max_age_active=0.15,
        max_age_degraded=0.50,
        lost_timeout=1.0,
        covariance_blowup=1e6,
    )
    assert state == SourceState.LOST


def test_source_lost_on_nonfinite_covariance() -> None:
    """Non-finite covariance immediately yields LOST."""
    state = evaluate_source_state(
        age_sec=0.05,
        no_msg_duration_sec=0.05,
        time_regression=False,
        covariance_finite=False,
        covariance_exceeds=False,
        max_age_active=0.15,
        max_age_degraded=0.50,
        lost_timeout=1.0,
        covariance_blowup=1e6,
    )
    assert state == SourceState.LOST


def test_source_lost_on_covariance_blowup() -> None:
    """Covariance exceeding the blowup threshold yields LOST."""
    state = evaluate_source_state(
        age_sec=0.05,
        no_msg_duration_sec=0.05,
        time_regression=False,
        covariance_finite=True,
        covariance_exceeds=True,
        max_age_active=0.15,
        max_age_degraded=0.50,
        lost_timeout=1.0,
        covariance_blowup=1e6,
    )
    assert state == SourceState.LOST


def test_source_degraded_on_time_regression() -> None:
    """Time regression yields DEGRADED even when inputs are fresh."""
    state = evaluate_source_state(
        age_sec=0.05,
        no_msg_duration_sec=0.05,
        time_regression=True,
        covariance_finite=True,
        covariance_exceeds=False,
        max_age_active=0.15,
        max_age_degraded=0.50,
        lost_timeout=1.0,
        covariance_blowup=1e6,
    )
    assert state == SourceState.DEGRADED


# ---------------------------------------------------------------------------
# is_visual_stable
# ---------------------------------------------------------------------------


def test_visual_stable_when_criteria_met() -> None:
    """Sufficient consecutive samples and duration → stable."""
    assert is_visual_stable(
        consecutive_count=5,
        first_stable_time_sec=0.0,
        now_sec=0.6,
        required_samples=5,
        required_duration=0.5,
    )


def test_visual_not_stable_when_too_few_samples() -> None:
    """Insufficient consecutive samples → not stable."""
    assert not is_visual_stable(
        consecutive_count=4,
        first_stable_time_sec=0.0,
        now_sec=1.0,
        required_samples=5,
        required_duration=0.5,
    )


def test_visual_not_stable_when_insufficient_duration() -> None:
    """Enough samples but not enough time → not stable."""
    assert not is_visual_stable(
        consecutive_count=6,
        first_stable_time_sec=0.0,
        now_sec=0.3,
        required_samples=5,
        required_duration=0.5,
    )


def test_visual_not_stable_when_no_start_time() -> None:
    """No first_stable_time → not stable."""
    assert not is_visual_stable(
        consecutive_count=10,
        first_stable_time_sec=None,
        now_sec=1.0,
        required_samples=5,
        required_duration=0.5,
    )


# ---------------------------------------------------------------------------
# poses_aligned
# ---------------------------------------------------------------------------


def test_poses_aligned_when_close() -> None:
    """Small position and yaw difference → aligned."""
    assert poses_aligned(
        current_x=1.0,
        current_y=2.0,
        current_yaw=0.0,
        candidate_x=1.1,
        candidate_y=2.1,
        candidate_yaw=math.radians(5.0),
        max_position_diff_m=0.25,
        max_yaw_diff_rad=math.radians(10.0),
    )


def test_poses_not_aligned_when_position_jump() -> None:
    """Position difference exceeds threshold → not aligned."""
    assert not poses_aligned(
        current_x=0.0,
        current_y=0.0,
        current_yaw=0.0,
        candidate_x=0.5,
        candidate_y=0.0,
        candidate_yaw=0.0,
        max_position_diff_m=0.25,
        max_yaw_diff_rad=math.radians(10.0),
    )


def test_poses_not_aligned_when_yaw_jump() -> None:
    """Yaw difference exceeds threshold → not aligned."""
    assert not poses_aligned(
        current_x=0.0,
        current_y=0.0,
        current_yaw=0.0,
        candidate_x=0.0,
        candidate_y=0.0,
        candidate_yaw=math.radians(20.0),
        max_position_diff_m=0.25,
        max_yaw_diff_rad=math.radians(10.0),
    )


def test_poses_aligned_with_yaw_wraparound() -> None:
    """Yaw difference wraps correctly across ±π."""
    # current_yaw = 179°, candidate_yaw = -179° → diff = 2°
    assert poses_aligned(
        current_x=0.0,
        current_y=0.0,
        current_yaw=math.radians(179.0),
        candidate_x=0.0,
        candidate_y=0.0,
        candidate_yaw=math.radians(-179.0),
        max_position_diff_m=0.25,
        max_yaw_diff_rad=math.radians(10.0),
    )


# ---------------------------------------------------------------------------
# decide_source_switch
# ---------------------------------------------------------------------------


class TestLIOPrimaryWhenFresh:
    """LIO is the preferred source when fresh."""

    def test_lio_primary_when_both_fresh(self) -> None:
        """LIO ACTIVE → no switch (stays LIO)."""
        result = decide_source_switch(
            current_primary=LocalizationSource.LIO,
            lio_state=SourceState.ACTIVE,
            visual_state=SourceState.ACTIVE,
            visual_stable=True,
            primary_duration_sec=5.0,
            thresholds=_default_thresholds(),
        )
        assert result is None  # Stay on LIO.

    def test_switch_to_lio_from_none(self) -> None:
        """From NONE, fresh LIO → switch to LIO."""
        result = decide_source_switch(
            current_primary=LocalizationSource.NONE,
            lio_state=SourceState.ACTIVE,
            visual_state=SourceState.ACTIVE,
            visual_stable=True,
            primary_duration_sec=0.0,
            thresholds=_default_thresholds(),
        )
        assert result == LocalizationSource.LIO

    def test_keep_lio_when_visual_also_active(self) -> None:
        """LIO ACTIVE and visual ACTIVE → LIO preferred, no switch."""
        result = decide_source_switch(
            current_primary=LocalizationSource.LIO,
            lio_state=SourceState.ACTIVE,
            visual_state=SourceState.ACTIVE,
            visual_stable=True,
            primary_duration_sec=0.1,
            thresholds=_default_thresholds(),
        )
        assert result is None


class TestVisualCandidateStable:
    """Visual source becomes a candidate when stable and LIO degrades."""

    def test_visual_candidate_when_lio_lost_and_visual_stable(self) -> None:
        """LIO LOST, visual stable, hysteresis satisfied → switch to VISUAL."""
        result = decide_source_switch(
            current_primary=LocalizationSource.LIO,
            lio_state=SourceState.LOST,
            visual_state=SourceState.ACTIVE,
            visual_stable=True,
            primary_duration_sec=3.0,  # > hysteresis 2.0s
            thresholds=_default_thresholds(),
        )
        assert result == LocalizationSource.VISUAL

    def test_hysteresis_blocks_switch_when_too_early(self) -> None:
        """LIO LOST but hysteresis not yet satisfied → no switch."""
        result = decide_source_switch(
            current_primary=LocalizationSource.LIO,
            lio_state=SourceState.LOST,
            visual_state=SourceState.ACTIVE,
            visual_stable=True,
            primary_duration_sec=0.5,  # < hysteresis 2.0s
            thresholds=_default_thresholds(),
        )
        assert result is None

    def test_visual_not_stable_blocks_switch(self) -> None:
        """LIO LOST but visual not yet stable → no switch."""
        result = decide_source_switch(
            current_primary=LocalizationSource.LIO,
            lio_state=SourceState.LOST,
            visual_state=SourceState.ACTIVE,
            visual_stable=False,
            primary_duration_sec=3.0,
            thresholds=_default_thresholds(),
        )
        assert result is None

    def test_recover_to_lio_after_hysteresis(self) -> None:
        """Currently on VISUAL, LIO recovers, hysteresis satisfied → LIO."""
        result = decide_source_switch(
            current_primary=LocalizationSource.VISUAL,
            lio_state=SourceState.ACTIVE,
            visual_state=SourceState.ACTIVE,
            visual_stable=True,
            primary_duration_sec=3.0,  # > hysteresis
            thresholds=_default_thresholds(),
        )
        assert result == LocalizationSource.LIO

    def test_recover_to_lio_blocked_by_hysteresis(self) -> None:
        """Currently on VISUAL, LIO recovers but hysteresis not yet satisfied."""
        result = decide_source_switch(
            current_primary=LocalizationSource.VISUAL,
            lio_state=SourceState.ACTIVE,
            visual_state=SourceState.ACTIVE,
            visual_stable=True,
            primary_duration_sec=1.0,  # < hysteresis
            thresholds=_default_thresholds(),
        )
        assert result is None


class TestLostWhenNoSources:
    """System transitions to NONE when all sources are lost."""

    def test_lost_when_both_sources_lost_from_lio(self) -> None:
        """From LIO, both sources LOST → NONE."""
        result = decide_source_switch(
            current_primary=LocalizationSource.LIO,
            lio_state=SourceState.LOST,
            visual_state=SourceState.LOST,
            visual_stable=False,
            primary_duration_sec=5.0,
            thresholds=_default_thresholds(),
        )
        assert result == LocalizationSource.NONE

    def test_lost_when_both_sources_lost_from_visual(self) -> None:
        """From VISUAL, both sources LOST → NONE."""
        result = decide_source_switch(
            current_primary=LocalizationSource.VISUAL,
            lio_state=SourceState.LOST,
            visual_state=SourceState.LOST,
            visual_stable=False,
            primary_duration_sec=5.0,
            thresholds=_default_thresholds(),
        )
        assert result == LocalizationSource.NONE

    def test_stay_none_when_no_sources(self) -> None:
        """From NONE, no sources available → stay NONE."""
        result = decide_source_switch(
            current_primary=LocalizationSource.NONE,
            lio_state=SourceState.LOST,
            visual_state=SourceState.LOST,
            visual_stable=False,
            primary_duration_sec=0.0,
            thresholds=_default_thresholds(),
        )
        assert result is None


class TestNoJumpOnAlignedSwitch:
    """Alignment checks prevent pose discontinuity on switch."""

    def test_aligned_switch_allowed(self) -> None:
        """Small pose difference → switch allowed."""
        aligned = poses_aligned(
            current_x=1.0,
            current_y=2.0,
            current_yaw=0.0,
            candidate_x=1.1,
            candidate_y=2.1,
            candidate_yaw=math.radians(5.0),
            max_position_diff_m=0.25,
            max_yaw_diff_rad=math.radians(10.0),
        )
        assert aligned

    def test_reject_unaligned_switch_position(self) -> None:
        """Large position jump → rejected."""
        aligned = poses_aligned(
            current_x=0.0,
            current_y=0.0,
            current_yaw=0.0,
            candidate_x=1.0,
            candidate_y=0.0,
            candidate_yaw=0.0,
            max_position_diff_m=0.25,
            max_yaw_diff_rad=math.radians(10.0),
        )
        assert not aligned

    def test_reject_unaligned_switch_yaw(self) -> None:
        """Large yaw jump → rejected."""
        aligned = poses_aligned(
            current_x=0.0,
            current_y=0.0,
            current_yaw=0.0,
            candidate_x=0.05,
            candidate_y=0.0,
            candidate_yaw=math.radians(15.0),
            max_position_diff_m=0.25,
            max_yaw_diff_rad=math.radians(10.0),
        )
        assert not aligned

    def test_boundary_position_diff_allowed(self) -> None:
        """Position difference exactly at threshold → allowed."""
        aligned = poses_aligned(
            current_x=0.0,
            current_y=0.0,
            current_yaw=0.0,
            candidate_x=0.25,
            candidate_y=0.0,
            candidate_yaw=0.0,
            max_position_diff_m=0.25,
            max_yaw_diff_rad=math.radians(10.0),
        )
        assert aligned

    def test_boundary_yaw_diff_allowed(self) -> None:
        """Yaw difference exactly at threshold → allowed."""
        aligned = poses_aligned(
            current_x=0.0,
            current_y=0.0,
            current_yaw=0.0,
            candidate_x=0.0,
            candidate_y=0.0,
            candidate_yaw=math.radians(10.0),
            max_position_diff_m=0.25,
            max_yaw_diff_rad=math.radians(10.0),
        )
        assert aligned
