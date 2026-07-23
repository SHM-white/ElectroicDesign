from __future__ import annotations

import subprocess
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
DESCRIPTION_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_description"
DESCRIPTION_CHECKER = DESCRIPTION_ROOT / "tools" / "verify_static_tf.py"
MODEL = DESCRIPTION_ROOT / "urdf" / "ed_uav.urdf.xacro"


def run_checker(urdf_path: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(DESCRIPTION_CHECKER), "--urdf", str(urdf_path)],
        capture_output=True,
        check=False,
        text=True,
    )


def test_description_declares_each_approved_static_sensor_edge_once() -> None:
    # Given: the checked-in robot model.
    # When: fixed-joint TF ownership is checked.
    result = run_checker(MODEL)

    # Then: only the five contract-owned sensor edges are present exactly once.
    assert result.returncode == 0, result.stderr
    assert result.stdout == "DESCRIPTION: GREEN\n"


def test_static_description_rejects_forbidden_world_frame_tf(tmp_path: Path) -> None:
    # Given: a model that attempts to publish a localization-owned world edge.
    model = tmp_path / "forbidden.urdf"
    model.write_text(
        """<robot name=\"forbidden\">
  <link name=\"map\"/><link name=\"odom\"/>
  <joint name=\"map_to_odom\" type=\"fixed\">
    <parent link=\"map\"/><child link=\"odom\"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )

    # When: static TF ownership is checked.
    result = run_checker(model)

    # Then: the reserved localization edge is rejected.
    assert result.returncode != 0
    assert "DESCRIPTION: RED: forbidden static TF: map -> odom" in result.stderr


def test_static_description_rejects_duplicate_sensor_tf(tmp_path: Path) -> None:
    # Given: a model that duplicates a base-to-sensor transform.
    model = tmp_path / "duplicate.urdf"
    model.write_text(
        """<robot name=\"duplicate\">
  <link name=\"base_link\"/><link name=\"fcu_link\"/>
  <joint name=\"one\" type=\"fixed\">
    <parent link=\"base_link\"/><child link=\"fcu_link\"/>
  </joint>
  <joint name=\"two\" type=\"fixed\">
    <parent link=\"base_link\"/><child link=\"fcu_link\"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )

    # When: static TF ownership is checked.
    result = run_checker(model)

    # Then: duplicate publication authority is rejected.
    assert result.returncode != 0
    assert "DESCRIPTION: RED: duplicate static TF: base_link -> fcu_link" in result.stderr
