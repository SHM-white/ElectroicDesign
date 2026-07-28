from __future__ import annotations

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = PACKAGE_ROOT / "ed_uav_localization" / "source_supervisor.py"


def test_none_transition_when_both_sources_are_lost_skips_alignment_and_clears_output() -> None:
    # Given
    source_text = SOURCE_PATH.read_text(encoding="utf-8")

    # When
    none_branch = source_text.index("if candidate == LocalizationSource.NONE:")
    alignment_check = source_text.index("self._check_alignment_for(candidate)")
    clear_fused_output = source_text.index("self._fused_odom = None")
    fused_selection = source_text.index("odom = self._select_odom()")

    # Then
    assert none_branch < alignment_check
    assert none_branch < clear_fused_output < fused_selection
    assert "self._primary_source = LocalizationSource.NONE" in source_text


def test_map_to_odom_status_when_anchor_is_unavailable_uses_tf_buffer_readiness() -> None:
    # Given
    source_text = SOURCE_PATH.read_text(encoding="utf-8")

    # When
    readiness_helper = source_text.index("def _map_to_odom_available(self) -> bool:")
    status_assignment = source_text.index(
        "msg.map_to_odom_valid = self._map_to_odom_available()"
    )

    # Then
    assert "from tf2_ros import Buffer, TransformBroadcaster, TransformListener" in source_text
    assert "self._tf_buffer = Buffer()" in source_text
    assert "self._tf_listener = TransformListener(self._tf_buffer, self)" in source_text
    assert readiness_helper < status_assignment
    assert 'return bool(self._tf_buffer.can_transform("map", "odom", Time()))' in source_text
    assert "msg.map_to_odom_valid = True" not in source_text
