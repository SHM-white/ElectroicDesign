from __future__ import annotations

import subprocess
import sys
import xml.etree.ElementTree as element_tree
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
    root = element_tree.parse(MODEL).getroot()
    edges = [
        (joint.find("parent").get("link"), joint.find("child").get("link"))
        for joint in root.findall("joint")
        if joint.get("type") == "fixed"
    ]
    sensor_edges = {
        ("base_link", "fcu_link"),
        ("base_link", "lidar_link"),
        ("base_link", "camera_narrow_optical_frame"),
        ("base_link", "camera_wide_optical_frame"),
        ("base_link", "rangefinder_link"),
    }
    visualization_edges = {
        ("base_link", "illustrative_forward_link"),
        ("base_link", "illustrative_up_link"),
    }

    # Then: five sensor edges and two visualization-only edges each have one owner.
    assert set(edges) == sensor_edges | visualization_edges
    assert all(edges.count(edge) == 1 for edge in sensor_edges | visualization_edges)


def test_robot_model_has_labeled_visual_geometry_without_physics_claims() -> None:
    # Given: the checked-in visualization model.
    root = element_tree.parse(MODEL).getroot()
    base_link = root.find("./link[@name='base_link']")

    # When: the base link's renderable geometry is inspected.
    visuals = [] if base_link is None else base_link.findall("visual")

    # Then: clearly labeled illustrative visuals exist without collision or inertial data.
    assert {visual.get("name") for visual in visuals} == {"illustrative_base_body"}
    assert all(visual.find("geometry") is not None for visual in visuals)
    assert base_link is not None
    assert base_link.find("collision") is None
    assert base_link.find("inertial") is None


def test_visualization_links_use_inline_colors_and_separated_orientation_markers() -> None:
    # Given: the checked-in synthetic visualization model.
    root = element_tree.parse(MODEL).getroot()
    links = {
        name: root.find(f"./link[@name='{name}']")
        for name in ("base_link", "illustrative_forward_link", "illustrative_up_link")
    }
    assert all(link is not None for link in links.values())
    base_link = links["base_link"]
    forward_link = links["illustrative_forward_link"]
    up_link = links["illustrative_up_link"]
    assert base_link is not None and forward_link is not None and up_link is not None
    assert len(forward_link.findall("visual")) == 1
    assert len(up_link.findall("visual")) == 1
    visuals = {
        visual.get("name"): visual
        for link in (base_link, forward_link, up_link)
        for visual in link.findall("visual")
    }

    # When: each visual's material and marker geometry are inspected.
    colors = {
        name: visual.find("./material/color").get("rgba")
        for name, visual in visuals.items()
        if visual.find("./material/color") is not None
    }
    joints = {
        joint.find("child").get("link"): joint
        for joint in root.findall("joint")
        if joint.find("child") is not None
    }
    forward_origin = joints["illustrative_forward_link"].find("origin")
    forward_size = forward_link.find("./visual/geometry/box").get("size")
    up_origin = joints["illustrative_up_link"].find("origin")
    up_cylinder = up_link.find("./visual/geometry/cylinder")

    # Then: Humble receives explicit colors and unmistakable synthetic markers.
    assert colors == {
        "illustrative_base_body": "0.18 0.35 0.58 1.0",
        "illustrative_forward_marker": "0.95 0.45 0.12 1.0",
        "illustrative_up_marker": "0.20 0.75 0.45 1.0",
    }
    assert forward_origin is not None
    assert tuple(float(value) for value in forward_origin.get("xyz").split()) == (0.52, 0.0, 0.06)
    assert tuple(float(value) for value in forward_size.split()) == (0.24, 0.18, 0.12)
    assert up_origin is not None
    assert tuple(float(value) for value in up_origin.get("xyz").split()) == (-0.24, 0.0, 0.26)
    assert up_cylinder is not None
    assert up_cylinder.get("radius") == "0.08"
    assert up_cylinder.get("length") == "0.28"


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
