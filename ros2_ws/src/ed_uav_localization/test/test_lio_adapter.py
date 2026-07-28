from __future__ import annotations

import importlib
import json
import math
import sys
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
INTERFACES_ROOT = PACKAGE_ROOT.parent / "ed_uav_interfaces"
sys.path.insert(0, str(PACKAGE_ROOT))


@dataclass
class Stamp:
    sec: int
    nanosec: int


@dataclass
class Header:
    stamp: Stamp
    frame_id: str


@dataclass
class Position:
    x: float
    y: float
    z: float


@dataclass
class Orientation:
    x: float
    y: float
    z: float
    w: float


@dataclass
class Pose:
    position: Position
    orientation: Orientation


@dataclass
class PoseWithCovariance:
    pose: Pose
    covariance: list[float]


@dataclass
class Velocity:
    x: float
    y: float
    z: float


@dataclass
class Twist:
    linear: Velocity
    angular: Velocity


@dataclass
class TwistWithCovariance:
    twist: Twist
    covariance: list[float]


@dataclass
class FakeOdometry:
    header: Header
    child_frame_id: str
    pose: PoseWithCovariance
    twist: TwistWithCovariance


def _raw_odometry() -> FakeOdometry:
    return FakeOdometry(
        header=Header(stamp=Stamp(sec=123, nanosec=456), frame_id="fast_lio_world"),
        child_frame_id="fast_lio_body",
        pose=PoseWithCovariance(
            pose=Pose(
                position=Position(x=1.25, y=-2.5, z=3.75),
                orientation=Orientation(x=0.1, y=-0.2, z=0.3, w=0.9),
            ),
            covariance=[float(index) for index in range(36)],
        ),
        twist=TwistWithCovariance(
            twist=Twist(
                linear=Velocity(x=4.0, y=5.0, z=6.0),
                angular=Velocity(x=-0.4, y=0.5, z=-0.6),
            ),
            covariance=[float(index + 36) for index in range(36)],
        ),
    )


def _diagonal_covariance(values: tuple[float, float, float, float, float, float]) -> list[float]:
    return [
        values[row] if row == column else 0.0
        for row in range(6)
        for column in range(6)
    ]


def test_normalize_odometry_when_fast_lio_message_preserves_payload() -> None:
    # Given
    raw_odometry = _raw_odometry()

    # When
    odometry = importlib.import_module("ed_uav_localization.odometry")
    normalized = odometry.normalize_odometry(
        raw_odometry,
        odometry.RigidTransform.from_xyz_rpy(
            xyz_m=(0.0, 0.0, 0.0), rpy_rad=(0.0, 0.0, 0.0)
        ),
    )

    # Then
    assert normalized is not None
    assert normalized is not raw_odometry
    assert normalized.header.stamp == raw_odometry.header.stamp
    assert normalized.pose == raw_odometry.pose
    assert normalized.header.frame_id == "odom"
    assert normalized.child_frame_id == "base_link"
    assert raw_odometry.header.frame_id == "fast_lio_world"
    assert raw_odometry.child_frame_id == "fast_lio_body"


def test_normalize_odometry_when_lidar_has_extrinsic_returns_base_pose() -> None:
    # Given
    raw_odometry = _raw_odometry()
    raw_odometry.pose.pose.position = Position(x=10.0, y=20.0, z=3.0)
    raw_odometry.pose.pose.orientation = Orientation(
        x=0.0,
        y=0.0,
        z=math.sin(math.pi / 4.0),
        w=math.cos(math.pi / 4.0),
    )

    # When
    odometry = importlib.import_module("ed_uav_localization.odometry")
    base_to_lidar = odometry.RigidTransform.from_xyz_rpy(
        xyz_m=(0.12, 0.0, 0.08), rpy_rad=(0.0, 0.0, math.pi / 2.0)
    )
    normalized = odometry.normalize_odometry(raw_odometry, base_to_lidar)

    # Then
    assert normalized is not None
    assert normalized.pose.pose.position.x == pytest.approx(9.88)
    assert normalized.pose.pose.position.y == pytest.approx(20.0)
    assert normalized.pose.pose.position.z == pytest.approx(2.92)
    assert normalized.pose.pose.orientation.x == pytest.approx(0.0)
    assert normalized.pose.pose.orientation.y == pytest.approx(0.0)
    assert normalized.pose.pose.orientation.z == pytest.approx(0.0)
    assert normalized.pose.pose.orientation.w == pytest.approx(1.0)


def test_normalize_odometry_when_lidar_extrinsic_is_nonidentity_transforms_twist_and_covariances() -> None:
    # Given
    raw_odometry = _raw_odometry()
    raw_odometry.pose.pose.position = Position(x=10.0, y=20.0, z=3.0)
    raw_odometry.pose.pose.orientation = Orientation(
        x=0.0,
        y=0.0,
        z=math.sin(math.pi / 4.0),
        w=math.cos(math.pi / 4.0),
    )
    raw_odometry.twist.twist.linear = Velocity(x=1.0, y=2.0, z=3.0)
    raw_odometry.twist.twist.angular = Velocity(x=4.0, y=5.0, z=6.0)
    raw_odometry.pose.covariance = _diagonal_covariance((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
    raw_odometry.twist.covariance = _diagonal_covariance((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
    odometry = importlib.import_module("ed_uav_localization.odometry")
    base_to_lidar = odometry.RigidTransform.from_xyz_rpy(
        xyz_m=(0.12, 0.0, 0.08), rpy_rad=(0.0, 0.0, math.pi / 2.0)
    )

    # When
    normalized = odometry.normalize_odometry(raw_odometry, base_to_lidar)

    # Then
    assert normalized is not None
    assert normalized.twist.twist.angular.x == pytest.approx(5.0)
    assert normalized.twist.twist.angular.y == pytest.approx(-4.0)
    assert normalized.twist.twist.angular.z == pytest.approx(6.0)
    assert normalized.twist.twist.linear.x == pytest.approx(2.32)
    assert normalized.twist.twist.linear.y == pytest.approx(-1.32)
    assert normalized.twist.twist.linear.z == pytest.approx(2.52)
    assert normalized.pose.covariance[0] == pytest.approx(1.032)
    assert normalized.pose.covariance[7] == pytest.approx(2.112)
    assert normalized.pose.covariance[14] == pytest.approx(3.072)
    assert normalized.pose.covariance[4] == pytest.approx(-0.4)
    assert normalized.twist.covariance[0] == pytest.approx(2.0256)
    assert normalized.twist.covariance[7] == pytest.approx(1.1184)
    assert normalized.twist.covariance[14] == pytest.approx(3.0576)
    assert normalized.twist.covariance[4] == pytest.approx(-0.32)


@pytest.mark.parametrize("field", ["pose", "twist"])
def test_normalize_odometry_when_covariance_is_not_finite_or_6_by_6_rejects_message(
    field: str,
) -> None:
    # Given
    raw_odometry = _raw_odometry()
    covariance = raw_odometry.pose.covariance if field == "pose" else raw_odometry.twist.covariance
    covariance[:] = [0.0] * 35
    base_to_lidar = importlib.import_module(
        "ed_uav_localization.odometry"
    ).RigidTransform.from_xyz_rpy(
        xyz_m=(0.0, 0.0, 0.0), rpy_rad=(0.0, 0.0, 0.0)
    )

    # When
    normalized = importlib.import_module("ed_uav_localization.odometry").normalize_odometry(
        raw_odometry, base_to_lidar
    )

    # Then
    assert normalized is None


@pytest.mark.parametrize("field", ["pose", "twist"])
def test_normalize_odometry_when_covariance_is_nonfinite_rejects_message(
    field: str,
) -> None:
    # Given
    raw_odometry = _raw_odometry()
    covariance = raw_odometry.pose.covariance if field == "pose" else raw_odometry.twist.covariance
    covariance[0] = float("nan")
    base_to_lidar = importlib.import_module(
        "ed_uav_localization.odometry"
    ).RigidTransform.from_xyz_rpy(
        xyz_m=(0.0, 0.0, 0.0), rpy_rad=(0.0, 0.0, 0.0)
    )

    # When
    normalized = importlib.import_module("ed_uav_localization.odometry").normalize_odometry(
        raw_odometry, base_to_lidar
    )

    # Then
    assert normalized is None


@pytest.mark.parametrize("invalid_value", [float("nan"), float("inf")])
def test_normalize_odometry_when_pose_is_nonfinite_rejects_message(
    invalid_value: float,
) -> None:
    # Given
    raw_odometry = _raw_odometry()
    raw_odometry.pose.pose.orientation.w = invalid_value

    # When
    odometry = importlib.import_module("ed_uav_localization.odometry")
    normalized = odometry.normalize_odometry(
        raw_odometry,
        odometry.RigidTransform.from_xyz_rpy(
            xyz_m=(0.0, 0.0, 0.0), rpy_rad=(0.0, 0.0, 0.0)
        ),
    )

    # Then
    assert normalized is None


def test_odom_to_base_transform_when_selected_odom_uses_same_stamp_and_pose() -> None:
    # Given
    selected_odometry = _raw_odometry()

    # When
    odometry = importlib.import_module("ed_uav_localization.odometry")
    transform = odometry.odom_to_base_transform(selected_odometry)

    # Then
    assert transform.stamp == selected_odometry.header.stamp
    assert transform.parent_frame == "odom"
    assert transform.child_frame == "base_link"
    assert transform.translation_x == selected_odometry.pose.pose.position.x
    assert transform.translation_y == selected_odometry.pose.pose.position.y
    assert transform.translation_z == selected_odometry.pose.pose.position.z
    assert transform.rotation_x == selected_odometry.pose.pose.orientation.x
    assert transform.rotation_y == selected_odometry.pose.pose.orientation.y
    assert transform.rotation_z == selected_odometry.pose.pose.orientation.z
    assert transform.rotation_w == selected_odometry.pose.pose.orientation.w


def test_lio_adapter_when_wired_exposes_topics_and_no_tf_authority() -> None:
    # Given
    adapter_path = PACKAGE_ROOT / "ed_uav_localization" / "lio_adapter.py"

    # When
    adapter_text = adapter_path.read_text(encoding="utf-8")

    # Then
    assert 'self.declare_parameter("input_topic", "/fast_lio/odometry")' in adapter_text
    assert (
        'self.declare_parameter("output_topic", "/localization/lio/odom")'
        in adapter_text
    )
    assert 'self.declare_parameter("calibration_file", "")' in adapter_text
    for topic in (
        "/fast_lio/cloud_registered",
        "/fast_lio/laser_map",
        "/fast_lio/path",
        "/localization/lio/cloud_registered",
        "/localization/lio/map",
        "/localization/lio/path",
    ):
        assert topic in adapter_text
    assert "from ed_uav_description.calibration import" in adapter_text
    assert "load_calibration" in adapter_text
    assert "load_calibration(Path(calibration_file))" in adapter_text
    assert 'transform_for("lidar_link")' in adapter_text
    assert "normalize_odometry(msg, self._base_to_lidar)" in adapter_text
    assert "self._odom_pub.publish(normalized)" in adapter_text
    assert "TransformBroadcaster" not in adapter_text
    assert "StaticTransformBroadcaster" not in adapter_text
    assert "sendTransform" not in adapter_text


def test_package_when_adapter_is_installed_exposes_ros_dependencies() -> None:
    # Given
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    package_xml = ElementTree.parse(PACKAGE_ROOT / "package.xml")

    # When
    dependencies = {
        dependency.text
        for dependency in package_xml.findall("exec_depend")
        if dependency.text is not None
    }

    # Then
    assert "lio_adapter = ed_uav_localization.lio_adapter:main" in setup_text
    assert {"ed_uav_description", "rclpy", "nav_msgs"} <= dependencies


def test_source_supervisor_when_publishing_fused_odom_sends_matching_transform() -> None:
    # Given
    source_path = PACKAGE_ROOT / "ed_uav_localization" / "source_supervisor.py"

    # When
    source_text = source_path.read_text(encoding="utf-8")

    # Then
    assert "from tf2_ros import" in source_text
    assert "TransformBroadcaster" in source_text
    assert "self._tf_broadcaster = TransformBroadcaster(self)" in source_text
    assert "self._publish_odom_transform(odom)" in source_text
    assert "transform = odom_to_base_transform(odom)" in source_text
    assert "self._tf_broadcaster.sendTransform(transform_msg)" in source_text


def test_manifest_when_fused_odometry_is_authoritative_has_one_odom_base_owner() -> None:
    # Given
    manifest_path = INTERFACES_ROOT / "contracts" / "ros2_contract_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    # When
    odom_topics = [
        topic for topic in manifest["topics"] if topic["name"] == "/localization/odom"
    ]
    odom_to_base_edges = [
        edge
        for edge in manifest["tf_edges"]
        if edge["parent"] == "odom" and edge["child"] == "base_link"
    ]

    # Then
    assert odom_topics == [
        {
            "name": "/localization/odom",
            "type": "nav_msgs/msg/Odometry",
            "owner": "ed_uav_localization.source_supervisor",
            "qos": "state_reliable",
            "units": "SI: m, rad",
            "frame": "odom",
            "clock": "source_acquisition_ros_time",
            "freshness": "0.15 s",
        }
    ]
    assert odom_to_base_edges == [
        {
            "parent": "odom",
            "child": "base_link",
            "publisher": "ed_uav_localization.source_supervisor",
        }
    ]
