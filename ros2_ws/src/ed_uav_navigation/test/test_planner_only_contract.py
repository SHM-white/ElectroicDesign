from __future__ import annotations

from pathlib import Path
from typing import Final
from xml.etree import ElementTree


REPOSITORY_ROOT: Final = Path(__file__).resolve().parents[4]
PACKAGE_NAME: Final = "ed_uav_navigation"
PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config" / "planner_only.yaml"
MAP_YAML_PATH: Final = PACKAGE_ROOT / "maps" / "simulation_arena.yaml"
MAP_PGM_PATH: Final = PACKAGE_ROOT / "maps" / "simulation_arena.pgm"
LAUNCH_PATH: Final = PACKAGE_ROOT / "launch" / "planner_only.launch.py"
FORBIDDEN_PRODUCTION_TOKENS: Final = frozenset(
    (
        "controller_server",
        "bt_navigator",
        "waypoint_follower",
        "behavior_server",
        "velocity_smoother",
        "cmd_vel",
        "px4",
        "moveit",
        "nav3d",
        "local_costmap",
        "controller_plugins",
    )
)


def read_pgm(path: Path) -> tuple[int, int, int, list[int]]:
    tokens = " ".join(
        line.split("#", maxsplit=1)[0] for line in path.read_text(encoding="ascii").splitlines()
    ).split()
    assert tokens[:1] == ["P2"], "map must be an ASCII PGM"
    width, height, maximum = (int(token) for token in tokens[1:4])
    pixels = [int(token) for token in tokens[4:]]
    assert len(pixels) == width * height, "PGM pixel count must match its dimensions"
    return width, height, maximum, pixels


def occupancy_at(
    pixels: list[int], width: int, height: int, resolution: float, x_m: float, y_m: float
) -> int:
    column = int((x_m + 8.0) / resolution)
    row = height - 1 - int((y_m + 8.0) / resolution)
    return pixels[row * width + column]


def test_package_installs_planner_only_assets() -> None:
    # Given: the intended ament_python planner package surface.
    required_paths = (
        PACKAGE_ROOT / "package.xml",
        PACKAGE_ROOT / "setup.py",
        PACKAGE_ROOT / "setup.cfg",
        PACKAGE_ROOT / PACKAGE_NAME / "__init__.py",
        PACKAGE_ROOT / "resource" / PACKAGE_NAME,
        LAUNCH_PATH,
        CONFIG_PATH,
        MAP_YAML_PATH,
        MAP_PGM_PATH,
    )

    # When: the source tree is inspected before an installation exists.
    missing = [path.relative_to(REPOSITORY_ROOT).as_posix() for path in required_paths if not path.is_file()]

    # Then: every planner-only asset is present and setup installs it.
    assert not missing, f"missing navigation package assets: {missing}"
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert "launch" in setup_source
    assert "config" in setup_source
    assert "maps" in setup_source
    assert ".pgm" in setup_source


def test_package_declares_released_nav2_planning_dependencies() -> None:
    # Given: planner-only services require released Nav2 packages at runtime.
    package_xml = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()

    # When: all declared package dependencies are collected.
    dependencies = {element.text for element in package_xml.findall("./exec_depend")}

    # Then: the planner service and its costmap/map/lifecycle dependencies are explicit.
    assert package_xml.findtext("name") == PACKAGE_NAME
    assert {
        "ament_index_python",
        "launch",
        "launch_ros",
        "nav2_lifecycle_manager",
        "nav2_costmap_2d",
        "nav2_map_server",
        "nav2_msgs",
        "nav2_navfn_planner",
        "nav2_planner",
    } <= dependencies


def test_configuration_exposes_navfn_compute_path_to_pose_service() -> None:
    # Given: the static planner and costmap parameter file.
    source = CONFIG_PATH.read_text(encoding="utf-8")

    # When: the planner and fixed-frame settings are inspected.
    required_tokens = (
        "planner_server:",
        "planner_plugins: [\"GridBased\"]",
        "nav2_navfn_planner/NavfnPlanner",
        "global_frame: map",
        "robot_base_frame: base_link",
        "use_sim_time: true",
        "robot_radius: 0.35",
        "nav2_costmap_2d::StaticLayer",
        "nav2_costmap_2d::InflationLayer",
        "inflation_radius: 0.75",
        "autostart: true",
    )

    # Then: Navfn has the prerequisites to offer planning without motion execution.
    assert all(token in source for token in required_tokens)


def test_launch_starts_only_map_planner_and_navigation_lifecycle_nodes() -> None:
    # Given: the dedicated launch file is the navigation process boundary.
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    # When: its node declarations are inspected as static source.
    required_nodes = (
        'package="nav2_map_server"',
        'executable="map_server"',
        'name="map_server"',
        'package="nav2_planner"',
        'executable="planner_server"',
        'name="planner_server"',
        'package="nav2_lifecycle_manager"',
        'executable="lifecycle_manager"',
        'name="lifecycle_manager_navigation"',
        "yaml_filename",
    )

    # Then: only the three planned service nodes are launched.
    assert source.count("Node(") == 3
    assert all(token in source for token in required_nodes)


def test_static_arena_map_covers_the_gazebo_bounds_and_obstacles_conservatively() -> None:
    # Given: the 16 m by 16 m Gazebo arena and its static occupancy map.
    map_yaml = MAP_YAML_PATH.read_text(encoding="utf-8")
    width, height, maximum, pixels = read_pgm(MAP_PGM_PATH)

    # When: map geometry and named obstacle centers are translated to grid cells.
    resolution = 0.5
    border_cells = (
        pixels[row * width + column]
        for row in range(height)
        for column in range(width)
        if row in (0, 1, height - 2, height - 1) or column in (0, 1, width - 2, width - 1)
    )

    # Then: walls, the block, and the cylinder occupy space in the map frame.
    assert "image: simulation_arena.pgm" in map_yaml
    assert "resolution: 0.5" in map_yaml
    assert "origin: [-8.0, -8.0, 0.0]" in map_yaml
    assert width == height == 32
    assert maximum == 254
    assert set(pixels) <= {0, 254}
    assert all(pixel == 0 for pixel in border_cells)
    assert occupancy_at(pixels, width, height, resolution, 2.5, 1.5) == 0
    assert occupancy_at(pixels, width, height, resolution, -2.2, -1.8) == 0


def test_production_assets_exclude_motion_and_3d_navigation_components() -> None:
    # Given: only checked-in package production files, excluding this contract test.
    sources = (
        (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8"),
        (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8"),
        LAUNCH_PATH.read_text(encoding="utf-8"),
        CONFIG_PATH.read_text(encoding="utf-8"),
        MAP_YAML_PATH.read_text(encoding="utf-8"),
    )

    # When: prohibited execution and 3D-navigation identifiers are searched.
    combined_source = "\n".join(sources).lower()

    # Then: this package remains a fixed-altitude XY planning service only.
    present = sorted(token for token in FORBIDDEN_PRODUCTION_TOKENS if token in combined_source)
    assert not present, f"planner-only production assets include forbidden tokens: {present}"


def test_humble_dockerfile_installs_released_nav2_runtime_packages() -> None:
    # Given: the Humble image declares ROS packages via apt, not source pins.
    dockerfile = (REPOSITORY_ROOT / "docker" / "Dockerfile.humble").read_text(encoding="utf-8")

    # When: active package-list lines are inspected.
    package_lines = "\n".join(line for line in dockerfile.splitlines() if not line.lstrip().startswith("#"))

    # Then: released Navigation2 and its bringup package are image dependencies.
    assert "ros-humble-navigation2" in package_lines
    assert "ros-humble-nav2-bringup" in package_lines
    assert "ros-humble-nav2-costmap-2d" in package_lines
    assert "github.com/ros-navigation" not in package_lines
