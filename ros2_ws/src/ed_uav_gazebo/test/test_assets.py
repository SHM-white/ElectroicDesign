from pathlib import Path
from itertools import permutations
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).parents[1]


def test_model_has_native_quadrotor_dynamics() -> None:
    # Given: the installed simulator model asset.
    model = ElementTree.parse(PACKAGE_ROOT / "models" / "ed_quadrotor" / "model.sdf")
    xml = model.getroot()
    plugins = xml.findall(".//plugin")

    # When: the vehicle dynamics contract is inspected.
    plugin_names = [plugin.attrib["name"] for plugin in plugins]
    motor_plugins = [name for name in plugin_names if "MulticopterMotorModel" in name]

    # Then: one native velocity controller and four native motor models are present.
    assert "ignition::gazebo::systems::MulticopterVelocityControl" in plugin_names
    assert len(motor_plugins) == 4
    assert all(plugin.attrib["filename"] == "ignition-gazebo-multicopter-motor-model-system" for plugin in plugins if "MulticopterMotorModel" in plugin.attrib["name"])


def test_quadrotor_allocation_matrix_has_full_control_rank() -> None:
    # Given: Fortress rotor geometry and control directions from the model.
    xml = ElementTree.parse(PACKAGE_ROOT / "models" / "ed_quadrotor" / "model.sdf").getroot()
    links = {link.attrib["name"]: link for link in xml.findall(".//link")}
    joint_children = {
        joint.attrib["name"]: joint.findtext("child", default="")
        for joint in xml.findall(".//joint")
    }
    controller = next(
        plugin
        for plugin in xml.findall(".//plugin")
        if "MulticopterVelocityControl" in plugin.attrib["name"]
    )
    columns: list[tuple[float, float, float, float]] = []
    for rotor in controller.findall("./rotorConfiguration/rotor"):
        joint_name = rotor.findtext("jointName", default="")
        pose = links[joint_children[joint_name]].findtext("pose", default="").split()
        x_m, y_m = float(pose[0]), float(pose[1])
        direction = float(rotor.findtext("direction", default="0"))
        moment_constant = float(rotor.findtext("momentConstant", default="0"))
        columns.append((y_m, -x_m, -direction * moment_constant, 1.0))

    # When: the determinant of Fortress' normalized 4x4 allocation matrix is computed.
    matrix = tuple(tuple(column[row] for column in columns) for row in range(4))
    determinant = 0.0
    for permutation in permutations(range(4)):
        inversions = sum(
            permutation[left] > permutation[right]
            for left in range(4)
            for right in range(left + 1, 4)
        )
        product = 1.0
        for row, column in enumerate(permutation):
            product *= matrix[row][column]
        determinant += (-1.0 if inversions % 2 else 1.0) * product

    # Then: all four commanded axes are linearly independent.
    assert abs(determinant) > 1e-9


def test_controller_directions_match_motor_turning_directions() -> None:
    # Given: controller and motor plugins share one direction contract per joint.
    xml = ElementTree.parse(PACKAGE_ROOT / "models" / "ed_quadrotor" / "model.sdf").getroot()
    plugins = xml.findall(".//plugin")
    controller = next(
        plugin for plugin in plugins if "MulticopterVelocityControl" in plugin.attrib["name"]
    )
    controller_directions = {
        rotor.findtext("jointName", default=""): int(rotor.findtext("direction", default="0"))
        for rotor in controller.findall("./rotorConfiguration/rotor")
    }
    motor_directions = {
        plugin.findtext("jointName", default=""): plugin.findtext("turningDirection", default="")
        for plugin in plugins
        if "MulticopterMotorModel" in plugin.attrib["name"]
    }

    # Then: positive controller direction means CCW and negative means CW.
    assert motor_directions == {
        joint_name: "ccw" if direction == 1 else "cw"
        for joint_name, direction in controller_directions.items()
    }


def test_bridge_exposes_frozen_simulator_topics() -> None:
    # Given: the explicit bridge contract.
    bridge = (PACKAGE_ROOT / "config" / "bridge.yaml").read_text(encoding="utf-8")

    # When: required graph names are searched.
    topics = {
        "/camera/narrow/image_raw",
        "/camera/narrow/camera_info",
        "/camera/wide/image_raw",
        "/camera/wide/camera_info",
        "/lidar/points",
        "/lidar/imu",
        "/rangefinder/range",
        "/simulation/ground_truth/odom",
        "/simulation/cmd_vel",
        "/simulation/enable",
        "/clock",
    }

    # Then: all externally visible topics are represented by the bridge contract.
    assert all(topic in bridge for topic in topics)
    assert "sensor_msgs/msg/PointCloud2" in bridge
    assert bridge.count("ros_topic_name: /clock") == 1


def test_world_is_local_and_contains_arena_and_vehicle() -> None:
    # Given: the simulator world asset.
    world = ElementTree.parse(PACKAGE_ROOT / "worlds" / "ed_uav_arena.sdf")
    xml = world.getroot()
    world_text = (PACKAGE_ROOT / "worlds" / "ed_uav_arena.sdf").read_text(encoding="utf-8")

    # When: the world dependencies and useful geometry are inspected.
    includes = [include.findtext("uri") for include in xml.findall(".//include")]

    # Then: only the local vehicle is included and the arena has real obstacles.
    assert includes == ["model://ed_quadrotor"]
    assert "ground_plane" in world_text
    assert "obstacle_block" in world_text
    assert "obstacle_cylinder" in world_text
    assert "https://fuel.gazebosim.org" not in world_text
