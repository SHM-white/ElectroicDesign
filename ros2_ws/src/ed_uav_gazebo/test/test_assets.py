from pathlib import Path
from itertools import permutations
from xml.etree import ElementTree

import yaml


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
    bridge_path = PACKAGE_ROOT / "config" / "bridge.yaml"
    bridge = bridge_path.read_text(encoding="utf-8")
    bridge_entries = yaml.safe_load(bridge)

    # When: required graph names are searched.
    topics = {
        "/camera/narrow/image_raw",
        "/camera/narrow/camera_info",
        "/camera/wide/image_raw",
        "/camera/wide/camera_info",
        "/lidar/points_raw",
        "/lidar/imu",
        "/rangefinder/range",
        "/simulation/ground_truth/odom",
        "/simulation/car/odom",
        "/simulation/cmd_vel",
        "/simulation/car/cmd_vel",
        "/simulation/enable",
        "/clock",
    }

    # Then: all externally visible topics are represented by the bridge contract.
    assert all(topic in bridge for topic in topics)
    assert "sensor_msgs/msg/PointCloud2" in bridge
    assert bridge.count("ros_topic_name: /clock") == 1
    point_cloud_bridge = next(
        entry
        for entry in bridge_entries
        if entry["ros_topic_name"] == "/lidar/points_raw"
    )
    assert point_cloud_bridge["gz_topic_name"] == "/lidar/points/points"
    assert "/lidar/points" not in {entry["ros_topic_name"] for entry in bridge_entries}


def test_sim_rviz_has_connected_mapping_and_localization_debug_displays() -> None:
    # Given: the RViz asset launched with the integrated Gazebo graph.
    config = yaml.safe_load(
        (PACKAGE_ROOT / "rviz" / "sim.rviz").read_text(encoding="utf-8")
    )
    manager = config["Visualization Manager"]

    # When: the engineering panels and display sources are inspected.
    panel_classes = {panel["Class"] for panel in config["Panels"]}
    displays = manager["Displays"]
    topic_displays = {
        display["Name"]: (
            display["Class"],
            display["Topic"]
            if isinstance(display["Topic"], str)
            else display["Topic"]["Value"],
        )
        for display in displays
        if "Topic" in display
    }
    non_topic_displays = {
        display["Name"]: display["Class"]
        for display in displays
        if "Topic" not in display
    }

    # Then: only connected mapping, localization, sensor, and camera data is shown.
    assert manager["Global Options"]["Fixed Frame"] == "map"
    assert panel_classes == {
        "rviz_common/Displays",
        "rviz_common/Selection",
        "rviz_common/Tool Properties",
        "rviz_common/Views",
    }
    assert non_topic_displays == {
        "Grid": "rviz_default_plugins/Grid",
        "TF": "rviz_default_plugins/TF",
        "Robot": "rviz_default_plugins/RobotModel",
    }
    assert topic_displays == {
        "Map": ("rviz_default_plugins/Map", "/map"),
        "Registered": ("rviz_default_plugins/PointCloud2", "/localization/lio/cloud_registered"),
        "LIO Map": ("rviz_default_plugins/PointCloud2", "/localization/lio/map"),
        "Fused Odom": ("rviz_default_plugins/Odometry", "/localization/odom"),
        "LIO Path": ("rviz_default_plugins/Path", "/localization/lio/path"),
        "Narrow": ("rviz_default_plugins/Image", "/camera/narrow/image_raw"),
        "Wide": ("rviz_default_plugins/Image", "/camera/wide/image_raw"),
    }
    assert not {"/plan", "/global_plan", "/cmd_vel"} & {
        topic for _, topic in topic_displays.values()
    }


def test_fast_lio_gazebo_config_declares_pointcloud_imu_and_mapping_outputs() -> None:
    # Given: Gazebo supplies standard ROS PointCloud2 and Imu messages.
    config_path = PACKAGE_ROOT / "config" / "fast_lio_gazebo.yaml"

    # When: the FAST-LIO simulation configuration is inspected.
    assert config_path.is_file()
    config = config_path.read_text(encoding="utf-8")

    # Then: it selects the standard-message pipeline and persistent mapping outputs.
    required_tokens = (
        "/lidar/points",
        "/lidar/imu",
        "lidar_type: 2",
        "scan_rate: 10",
        "feature_extract_enable: false",
        "extrinsic_est_en: false",
        "extrinsic_T: [0.0, 0.0, 0.0]",
        "extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]",
        "path_en: true",
        "scan_publish_en: true",
        "dense_publish_en: true",
        "pcd_save_en: false",
    )
    assert all(token in config for token in required_tokens)
    assert "CustomMsg" not in config
    parameters = yaml.safe_load(config)["/**"]["ros__parameters"]
    assert parameters["feature_extract_enable"] is False
    assert parameters["preprocess"]["lidar_type"] == 2
    assert "feature_extract_enable" not in parameters["preprocess"]
    assert parameters["point_filter_num"] == 3
    assert "point_filter_num" not in parameters["preprocess"]
    assert parameters["preprocess"]["timestamp_unit"] == 0


def test_fast_lio_simulation_launch_isolated_tf_and_calibrated_adapter() -> None:
    # Given: upstream FAST-LIO broadcasts TF unconditionally.
    launch_path = PACKAGE_ROOT / "launch" / "fast_lio_simulation.launch.py"

    # When: the simulation-specific launch composition is inspected.
    assert launch_path.is_file()
    launch = launch_path.read_text(encoding="utf-8")

    # Then: standard Gazebo topics drive FAST-LIO while its TF remains private.
    required_tokens = (
        'package="fast_lio"',
        'package="ed_uav_gazebo"',
        'executable="gazebo_pointcloud_normalizer"',
        'executable="fastlio_mapping"',
        '"input_topic": "/lidar/points_raw"',
        '"output_topic": "/lidar/points"',
        '"scan_rate_hz": 10.0',
        "fast_lio_gazebo.yaml",
        '("/tf", "/fast_lio/tf")',
        'package="ed_uav_localization"',
        'executable="lio_adapter"',
        '"output_topic": "/localization/lio/planar_raw"',
        'executable="planar_odom_fuser"',
        '"altitude_topic": "/simulation/ground_truth/odom"',
        '"output_topic": "/localization/lio/odom"',
        '"calibration_file": calibration_file',
        '"use_sim_time": use_sim_time',
    )
    assert all(token in launch for token in required_tokens)
    assert launch.index('executable="gazebo_pointcloud_normalizer"') < launch.index('executable="fastlio_mapping"')
    assert "TransformBroadcaster" not in launch
    setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert "gazebo_pointcloud_normalizer = ed_uav_gazebo.gazebo_pointcloud_normalizer:main" in setup
    assert "planar_odom_fuser = ed_uav_gazebo.planar_odom_fuser:main" in setup


def test_sim_launch_defaults_to_fast_lio_with_explicit_ground_truth_fallback() -> None:
    # Given: simulation has FAST-LIO and ground-truth localization modes.
    sim_launch = (PACKAGE_ROOT / "launch" / "sim.launch.py").read_text(encoding="utf-8")
    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

    # When: mode selection and runtime dependencies are inspected.
    required_tokens = (
        '"localization_mode"',
        'default_value="fast_lio"',
        'choices=("fast_lio", "ground_truth")',
        "fast_lio_simulation.launch.py",
        "planner_only.launch.py",
        "ground_truth_mode",
        'executable="sim_localization"',
        '"publish_odom_to_base_link_tf": False',
    )

    # Then: FAST-LIO is the default and ground truth is an explicit conditional fallback.
    assert all(token in sim_launch for token in required_tokens)
    assert sim_launch.count("condition=IfCondition(fast_lio_mode)") == 1
    assert '"publish_odom_to_base_link_tf": ground_truth_mode' not in sim_launch
    assert (
        '"profile_path": str(profile),\n'
        "                }.items(),\n"
        "            ),"
    ) in sim_launch
    assert "<depend>fast_lio</depend>" in package_xml
    assert "<depend>ed_uav_navigation</depend>" in package_xml


def test_gazebo_sensor_poses_and_sim_fcu_tf_gate_match_synthetic_calibration() -> None:
    # Given: the synthetic calibration fixes the lidar and lidar IMU rigid poses.
    model = ElementTree.parse(PACKAGE_ROOT / "models" / "ed_quadrotor" / "model.sdf").getroot()
    fcu_source = (PACKAGE_ROOT / "ed_uav_gazebo" / "sim_fcu.py").read_text(encoding="utf-8")

    # When: sensor poses and FCU TF ownership are inspected statically.
    sensor_poses = {
        sensor.attrib["name"]: sensor.findtext("pose", default="")
        for sensor in model.findall(".//sensor")
        if sensor.attrib["name"] in {"lidar", "lidar_imu"}
    }

    # Then: FAST-LIO sees a co-located lidar/IMU and FCU TF can be disabled.
    assert sensor_poses == {"lidar": "0.12 0 0.08 0 0 0", "lidar_imu": "0.12 0 0.08 0 0 0"}
    sensor_frames = {
        sensor.attrib["name"]: sensor.findtext("ignition_frame_id", default="")
        for sensor in model.findall(".//sensor")
        if sensor.attrib["name"] in {"lidar", "lidar_imu"}
    }
    assert sensor_frames == {"lidar": "lidar_link", "lidar_imu": "lidar_link"}
    assert 'self.declare_parameter("publish_odom_to_base_link_tf", True)' in fcu_source
    assert "if self.get_parameter(\"publish_odom_to_base_link_tf\").value:" in fcu_source


def test_world_matches_d_task_drawing_and_contains_both_vehicles() -> None:
    # Given: the simulator world asset.
    world = ElementTree.parse(PACKAGE_ROOT / "worlds" / "ed_uav_arena.sdf")
    xml = world.getroot()
    world_text = (PACKAGE_ROOT / "worlds" / "ed_uav_arena.sdf").read_text(encoding="utf-8")

    # When: the world dependencies and useful geometry are inspected.
    includes = [include.findtext("uri") for include in xml.findall(".//include")]
    plugin_names = {
        plugin.attrib["name"] for plugin in xml.findall(".//world/plugin")
    }

    # Then: the checked-in 4x5m D-task arena includes its route, indoor walls,
    # target car, and local quadrotor without any network model dependency.
    assert includes == ["model://d_task_car", "model://ed_quadrotor"]
    for token in (
        "d_arena_floor",
        "south_wall_2p5m",
        "north_wall_2p5m",
        "west_wall_2p5m",
        "east_wall_2p5m",
        "d_task_capsule_route",
        "home_h_marking",
        "point_a",
        "point_b",
        "point_c",
        "point_d",
    ):
        assert token in world_text
    assert "https://fuel.gazebosim.org" not in world_text
    assert "ignition::gazebo::systems::Imu" in plugin_names


def test_planar_lidar_and_downward_sensors_match_the_simulation_contract() -> None:
    model = ElementTree.parse(
        PACKAGE_ROOT / "models" / "ed_quadrotor" / "model.sdf"
    ).getroot()
    sensors = {sensor.attrib["name"]: sensor for sensor in model.findall(".//sensor")}

    lidar = sensors["lidar"]
    assert lidar.findtext("./ray/scan/vertical/samples") == "1"
    assert lidar.findtext("./ray/scan/vertical/min_angle") == "0"
    assert lidar.findtext("./ray/scan/vertical/max_angle") == "0"
    assert sensors["narrow_camera"].findtext("pose") == "0.08 0 -0.08 0 1.5708 0"
    assert sensors["wide_camera"].findtext("pose") == "-0.04 0 -0.08 0 1.5708 0"
    assert sensors["rangefinder"].findtext("pose") == "0 0 -0.08 0 1.5708 0"


def test_car_target_has_centered_fifteen_centimeter_tag36h11_zero() -> None:
    car_path = PACKAGE_ROOT / "models" / "d_task_car" / "model.sdf"
    car = ElementTree.parse(car_path).getroot()
    tag = next(
        visual
        for visual in car.findall(".//visual")
        if visual.attrib.get("name") == "center_apriltag_id_0"
    )
    assert tag.findtext("pose") == "0 0 0.163 0 0 0"
    assert tag.findtext("./geometry/plane/size") == "0.15 0.15"
    assert (
        tag.findtext("./material/pbr/albedo_map")
        == "model://apriltag_marker/materials/textures/tag36h11_0.png"
    )
    texture = (
        PACKAGE_ROOT
        / "models"
        / "apriltag_marker"
        / "materials"
        / "textures"
        / "tag36h11_0.png"
    )
    assert texture.is_file()
