"""Acceptance tests for versioned D-task ROS and deployment contracts."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]
D_TASK_CHECKER = PACKAGE_ROOT / "tools" / "check_d_task_config.py"
ROS_CHECKER = PACKAGE_ROOT / "tools" / "check_contract.py"
CONFIG_ROOT = PACKAGE_ROOT / "contracts" / "d_task"
ROS_FIXTURE = PACKAGE_ROOT / "test" / "fixtures" / "valid_contract.json"
ESP32_FIXTURE = PACKAGE_ROOT / "test" / "fixtures" / "esp32_frames_valid.json"
SCHEMA_ROOT = CONFIG_ROOT / "schemas"
MANIFEST = PACKAGE_ROOT / "contracts" / "ros2_contract_manifest.json"


def run_d_task_checker(kind: str, path: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(D_TASK_CHECKER), kind, str(path)],
        capture_output=True,
        text=True,
        check=False,
    )


@pytest.mark.parametrize(
    ("kind", "name"),
    [
        ("mission", "mission_profile.example.yaml"),
        ("target", "target_revision.example.yaml"),
        ("deployment", "deployment_preset.example.yaml"),
        ("esp32-frames", "esp32_frames.example.json"),
    ],
)
def test_accepts_valid_versioned_contract_examples(kind: str, name: str) -> None:
    # Given: a committed credential-free example at an external file boundary.
    path = CONFIG_ROOT / "examples" / name

    # When: the corresponding strict typed schema loader parses it.
    result = run_d_task_checker(kind, path)

    # Then: parsing succeeds with an explicit, kind-specific success receipt.
    assert result.returncode == 0, result.stderr
    assert result.stdout == f"D-TASK CONFIG: GREEN: {kind}\n"


@pytest.mark.parametrize(
    "name",
    [
        "mission_profile.schema.json",
        "target_revision.schema.json",
        "deployment_preset.schema.json",
        "esp32_frames.schema.json",
    ],
)
def test_committed_json_schemas_are_strict_draft_2020_12(name: str) -> None:
    # Given: a committed schema used by the typed configuration boundary.
    schema = json.loads((SCHEMA_ROOT / name).read_text(encoding="utf-8"))

    # When: the declared metaschema validates the schema itself.
    Draft202012Validator.check_schema(schema)

    # Then: root objects reject undeclared fields.
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert schema["additionalProperties"] is False


def test_local_field_manifest_is_ignored_and_excluded_from_install() -> None:
    # Given: the deployment boundary's source and install controls.
    ignore_rules = (REPO_ROOT / ".gitignore").read_text(encoding="utf-8")
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When / Then: real local values can enter neither Git nor package installation.
    assert "contracts/d_task/deployment_preset.local.yaml" in ignore_rules
    assert 'PATTERN "*.local.yaml" EXCLUDE' in cmake


@pytest.mark.parametrize(
    ("sequence", "acquisition_time_ms", "expected"),
    [
        (7, 1100, "duplicate telemetry sequence: 7"),
        (8, 900, "stale telemetry acquisition time: 900 <= 1000"),
        (8, 1501, "stale telemetry gap: 501 > 500 ms"),
    ],
)
def test_rejects_duplicate_or_stale_telemetry(
    tmp_path: Path,
    sequence: int,
    acquisition_time_ms: int,
    expected: str,
) -> None:
    # Given: a valid frame window followed by duplicate, regressed, or expired telemetry.
    document = json.loads(ESP32_FIXTURE.read_text(encoding="utf-8"))
    document["frames"][1]["sequence"] = sequence
    document["frames"][1]["acquisition_time_ms"] = acquisition_time_ms
    path = tmp_path / "invalid-frames.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    # When: the frame window crosses the typed boundary.
    result = run_d_task_checker("esp32-frames", path)

    # Then: temporal ambiguity is rejected instead of reported as success.
    assert result.returncode == 1
    assert expected in result.stderr
    assert result.stdout == ""


def test_rejects_descending_nonduplicate_uint32_sequence(tmp_path: Path) -> None:
    # Given: acquisition time increases while uint32 sequence descends from 8 to 7.
    document = json.loads(ESP32_FIXTURE.read_text(encoding="utf-8"))
    document["frames"][0]["sequence"] = 8
    document["frames"][1]["sequence"] = 7
    path = tmp_path / "descending-sequence.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    # When: telemetry ordering crosses the typed boundary.
    result = run_d_task_checker("esp32-frames", path)

    # Then: a backward nonduplicate serial is stale, not a uint32 wrap.
    assert result.returncode == 1
    assert "stale telemetry sequence: 7 after 8" in result.stderr
    assert result.stdout == ""


def test_accepts_forward_uint32_sequence_wrap(tmp_path: Path) -> None:
    # Given: the uint32 sequence advances from UINT32_MAX to zero.
    document = json.loads(ESP32_FIXTURE.read_text(encoding="utf-8"))
    document["frames"][0]["sequence"] = 4_294_967_295
    document["frames"][1]["sequence"] = 0
    path = tmp_path / "wrapped-sequence.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    # When: telemetry ordering crosses the typed boundary.
    result = run_d_task_checker("esp32-frames", path)

    # Then: modulo-forward progress remains valid.
    assert result.returncode == 0, result.stderr
    assert result.stdout == "D-TASK CONFIG: GREEN: esp32-frames\n"


def test_rejects_invalid_b_d_a_completion_route_order(tmp_path: Path) -> None:
    # Given: telemetry that claims D before the required B checkpoint.
    document = json.loads(ESP32_FIXTURE.read_text(encoding="utf-8"))
    document["frames"][1]["payload"]["route_stage"] = "D"
    path = tmp_path / "invalid-route.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    # When: route progression is validated across the frame window.
    result = run_d_task_checker("esp32-frames", path)

    # Then: only START -> B -> D -> A -> COMPLETE progression is accepted.
    assert result.returncode == 1
    assert "invalid route order: START -> D" in result.stderr


def test_rejects_unsupported_target_revision(tmp_path: Path) -> None:
    # Given: a target document naming an unapproved geometry revision.
    valid = (CONFIG_ROOT / "examples" / "target_revision.example.yaml").read_text(
        encoding="utf-8"
    )
    path = tmp_path / "unknown-target.yaml"
    path.write_text(valid.replace("d2026-circle-cross-v1", "legacy-cross-v0"), encoding="utf-8")

    # When: the target revision crosses the typed boundary.
    result = run_d_task_checker("target", path)

    # Then: unsupported geometry cannot activate deployment.
    assert result.returncode == 1
    assert "target_revision" in result.stderr


def test_rejects_placeholder_field_deployment_values(tmp_path: Path) -> None:
    # Given: a field preset containing a documentation-only endpoint and serial.
    path = tmp_path / "field-placeholder.yaml"
    path.write_text(
        """\
version: 1
preset_kind: field
preset_id: field-mid360
mission_profile_id: d2026-payload-drop
target_revision: d2026-circle-cross-v1
mid360:
  owner: ed_uav_lidar
  serial: REPLACE_ME
  sensor_ip: 192.0.2.10
  host_ip: 192.0.2.20
  firmware: UNKNOWN
ground_station:
  owner: ground_station_esp32s3
  transport: esp_now
  peer_id: 00:00:00:00:00:00
""",
        encoding="utf-8",
    )

    # When: the local field preset is parsed.
    result = run_d_task_checker("deployment", path)

    # Then: placeholders and documentation ranges fail closed.
    assert result.returncode == 1
    assert "placeholder deployment value" in result.stderr


def test_rejects_all_zero_esp_peer_as_placeholder(tmp_path: Path) -> None:
    # Given: a structurally valid field preset whose only placeholder is its ESP peer.
    path = tmp_path / "zero-peer.yaml"
    path.write_text(
        """\
version: 1
preset_kind: field
preset_id: field-mid360
mission_profile_id: d2026-payload-drop
target_revision: d2026-circle-cross-v1
mid360:
  owner: ed_uav_lidar
  serial: TEST-SERIAL-001
  sensor_ip: 10.20.0.2
  host_ip: 10.20.0.3
  firmware: 1.2.3-test
ground_station:
  owner: ground_station_esp32s3
  transport: esp_now
  peer_id: 00:00:00:00:00:00
""",
        encoding="utf-8",
    )

    # When: the field preset crosses the typed boundary.
    result = run_d_task_checker("deployment", path)

    # Then: the zero peer fails closed as a placeholder.
    assert result.returncode == 1
    assert "placeholder deployment value: zero ESP peer" in result.stderr
    assert result.stdout == ""


def test_installed_checker_resolves_schemas_from_package_share(tmp_path: Path) -> None:
    # Given: an installed layout with scripts under lib and schemas only under share.
    prefix = tmp_path / "install" / "ed_uav_interfaces"
    installed_lib = prefix / "lib" / "ed_uav_interfaces"
    installed_schemas = prefix / "share" / "ed_uav_interfaces" / "contracts" / "d_task" / "schemas"
    installed_lib.mkdir(parents=True)
    shutil.copy2(D_TASK_CHECKER, installed_lib)
    shutil.copy2(PACKAGE_ROOT / "tools" / "d_task_models.py", installed_lib)
    shutil.copytree(SCHEMA_ROOT, installed_schemas)

    # When: the installed checker parses a mission while no source-layout schema exists.
    result = subprocess.run(
        [
            sys.executable,
            str(installed_lib / "check_d_task_config.py"),
            "mission",
            str(CONFIG_ROOT / "examples" / "mission_profile.example.yaml"),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    # Then: it resolves package-share schemas and retains the normal CLI surface.
    assert result.returncode == 0, result.stderr
    assert result.stdout == "D-TASK CONFIG: GREEN: mission\n"


def test_adjacent_ros_contract_checker_regression(tmp_path: Path) -> None:
    # Given: the established ROS contract with one unbounded interface added.
    contract = json.loads(ROS_FIXTURE.read_text(encoding="utf-8"))
    contract["interfaces"] = [{"path": "msg/Bad.msg", "definition": "string reason"}]
    path = tmp_path / "unbounded.json"
    path.write_text(json.dumps(contract), encoding="utf-8")

    # When: the pre-existing checker validates the adjacent contract surface.
    result = subprocess.run(
        [sys.executable, str(ROS_CHECKER), str(path)],
        capture_output=True,
        text=True,
        check=False,
    )

    # Then: its established rejection remains unchanged.
    assert result.returncode == 1
    assert "unbounded dynamic text or array: msg/Bad.msg" in result.stderr


def test_target_observation_contract_exposes_typed_quality_and_rejection() -> None:
    # Given
    definition = (PACKAGE_ROOT / "msg" / "TargetObservation.msg").read_text(
        encoding="utf-8"
    )
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    # When
    frozen = next(
        item["definition"]
        for item in manifest["interfaces"]
        if item["path"] == "msg/TargetObservation.msg"
    )

    # Then
    for field in (
        "bool valid",
        "uint8 status",
        "uint16 candidate_count",
        "float32 reprojection_rms_px",
        "float32 quality",
        "string<=96 rejection_reason",
    ):
        assert field in definition
        assert field in frozen


def test_vehicle_contract_exposes_si_heading_context_on_canonical_topic() -> None:
    # Given
    definition = (PACKAGE_ROOT / "msg" / "VehicleTelemetry.msg").read_text(
        encoding="utf-8"
    )
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    # When
    topic = next(
        item for item in manifest["topics"] if item["type"].endswith("VehicleTelemetry")
    )

    # Then
    assert "float32 heading_rad" in definition
    assert "float32 yaw_rate_rad_s" in definition
    assert topic["name"] == "/d_task/vehicle/telemetry"
    assert topic["owner"] == "ed_uav_vehicle_bridge"
    assert "rad" in topic["units"]
    assert topic["frame"] == "vehicle_start"
    assert any(
        item["node"] == "ed_uav_vehicle_bridge"
        for item in manifest["lifecycle"]
    )


def test_esp32_schema_requires_heading_and_yaw_rate() -> None:
    # Given
    schema = json.loads(
        (SCHEMA_ROOT / "esp32_frames.schema.json").read_text(encoding="utf-8")
    )

    # When
    payload = schema["$defs"]["vehicleTelemetryPayload"]

    # Then
    assert "heading_rad" in payload["required"]
    assert "yaw_rate_rad_s" in payload["required"]
    assert payload["properties"]["heading_rad"]["minimum"] < 0.0
    assert payload["properties"]["yaw_rate_rad_s"]["minimum"] < 0.0


def test_checker_rejects_inconsistent_turn_and_yaw_rate(tmp_path: Path) -> None:
    # Given
    document = json.loads(ESP32_FIXTURE.read_text(encoding="utf-8"))
    document["frames"][0]["payload"]["yaw_rate_rad_s"] = 1.0
    path = tmp_path / "invalid-turn-context.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    # When
    result = run_d_task_checker("esp32-frames", path)

    # Then
    assert result.returncode == 1
    assert "invalid straight turn yaw rate" in result.stderr
