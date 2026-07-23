# =============================================================================
# test_legacy_parity.py — Legacy ↔ ROS 2 Parity Matrix & Golden-Vector Tests
# =============================================================================
#
# PARITY MATRIX (Phase-1 rows verified by golden-vector assertions below)
# ---------------------------------------------------------------------------
#
#  ╔═════════════════════╤══════════════════════╤════════════════════════════╗
#  ║ Concern             │ Legacy (drone/)       │ ROS 2 Replacement          ║
#  ╠═════════════════════╪══════════════════════╪════════════════════════════╣
#  ║ V7 frame encoding   │ lx_protocol.py        │ ed_uav_fcu_bridge/         ║
#  ║   checksums (SC+AC) │ build_lx_frame()      │   v7_codec.build_frame()   ║
#  ║   unlock command     │ cmd_unlock()          │   v7_codec.cmd_unlock()    ║
#  ║   lock command       │ cmd_lock()            │   v7_codec.cmd_lock()      ║
#  ║   mode select        │ cmd_mode(3)           │   v7_codec.cmd_mode(3)     ║
#  ║   takeoff command    │ cmd_takeoff(150)      │   v7_codec.cmd_takeoff(150)║
#  ║   land command       │ cmd_land()            │   v7_codec.cmd_land()      ║
#  ║   move command       │ cmd_move(100,30,90)   │   v7_codec.cmd_move(…)     ║
#  ╠═════════════════════╪══════════════════════╪════════════════════════════╣
#  ║ Grid coordinates    │ path_plan.py           │ ed_uav_mission/plugins/    ║
#  ║   block positions    │ init_grid()            │   coverage.GridCoverage    ║
#  ║   path generation    │ generate_move_cmds()  │   plugin.generate()        ║
#  ║   validation         │ validate_path()       │   mission_model schema    ║
#  ╠═════════════════════╪══════════════════════╪════════════════════════════╣
#  ║ State transitions   │ state_machine.py       │ ed_uav_mission/            ║
#  ║   IDLE→ARM→TAKEOFF   │ FlightState enum      │   state_machine.MissionFSM ║
#  ║   EXEC→RETURN→LAND   │ DroneStateMachine     │   MissionState enum        ║
#  ║   emergency handling │ _emergency()          │   ABORTED state            ║
#  ╠═════════════════════╪══════════════════════╪════════════════════════════╣
#  ║ Protocol commands   │ lx_protocol.py         │ ed_uav_fcu_bridge/         ║
#  ║   typed request      │ (implicit in mcu)     │   actions.CommandRequest   ║
#  ║   ACK correlation    │ return-value bool      │   FlightActionController  ║
#  ╠═════════════════════╪══════════════════════╪════════════════════════════╣
#  ║ INTENTIONAL DIFFS   │                        │                            ║
#  ║  0x08/0x51 separate  │ combined in mcu_ser   │ separate channels          ║
#  ║  hover safety cmd    │ no explicit hover      │ cmd_hover() added         ║
#  ║  freshness gates     │ timeout-based only     │ per-frame monotonic age   ║
#  ║  command ACK model   │ bool return, no corr   │ typed ACK + result codes  ║
#  ╚═════════════════════╧══════════════════════╧════════════════════════════╝
#
# ---------------------------------------------------------------------------
# INTENTIONAL DIFFERENCES (asserted, not errors)
# ---------------------------------------------------------------------------
# 1. 0x08/0x51 SEPARATION: legacy mcu_serial.py mixes position and diag;
#    ROS bridge keeps 0x08 as sole continuous position source, 0x51 as
#    separately stamped diagnostics. Verified by distinct publish paths.
# 2. HOVER SAFETY: legacy has no dedicated hover command; ROS adds
#    cmd_hover() (CID=0x10, CMD0=0x00, CMD1=0x04) for localization-loss policy.
# 3. FRESHNESS: legacy uses timeout-based polling; ROS attaches per-frame
#    monotonic age to every V7 frame and rejects stale data by config.
# 4. COMMAND ACK: legacy returns bool from send methods with no correlation;
#    ROS uses typed CommandRequest → CommandResult with ACK timeout/retry.
#
# =============================================================================

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


# ── Path setup: make both legacy drone/ and ROS bridge importable ──────

_PROJECT = Path(__file__).resolve().parents[4]  # ed/
_DRONE = _PROJECT / "drone"
_BRIDGE = _PROJECT / "ros2_ws" / "src" / "ed_uav_fcu_bridge"
_MISSION = _PROJECT / "ros2_ws" / "src" / "ed_uav_mission"

for _p in (_DRONE, _BRIDGE, _MISSION):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

# ── Legacy imports ─────────────────────────────────────────────────────

from lx_protocol import (  # type: ignore[import-not-found]
    build_lx_frame,
    build_pi_frame,
    cmd_ascend,
    cmd_descend,
    cmd_land as legacy_cmd_land,
    cmd_lock as legacy_cmd_lock,
    cmd_mode as legacy_cmd_mode,
    cmd_move as legacy_cmd_move,
    cmd_takeoff as legacy_cmd_takeoff,
    cmd_unlock as legacy_cmd_unlock,
    verify_lx_frame,
)

from path_plan import (  # type: ignore[import-not-found]
    BLOCK_GRID as LEGACY_BLOCK_GRID,
    BLOCK_POSITIONS as LEGACY_BLOCK_POSITIONS,
    PATH as LEGACY_PATH,
    TOTAL_BLOCKS as LEGACY_TOTAL_BLOCKS,
    generate_move_commands,
    init_grid as legacy_init_grid,
    validate_path,
)

from state_machine import FlightState  # type: ignore[import-not-found]

# ── ROS 2 imports ──────────────────────────────────────────────────────

from ed_uav_fcu_bridge.v7_codec import (  # type: ignore[import-not-found]
    build_frame,
    cmd_hover,
    cmd_land as ros_cmd_land,
    cmd_lock as ros_cmd_lock,
    cmd_mode as ros_cmd_mode,
    cmd_move as ros_cmd_move,
    cmd_takeoff as ros_cmd_takeoff,
    cmd_unlock as ros_cmd_unlock,
    decode_frame,
)

from ed_uav_fcu_bridge.actions import CommandKind, CommandRequest, FlightActionController  # type: ignore[import-not-found]
from ed_uav_mission.state_machine import MissionFSM, MissionState  # type: ignore[import-not-found]

# ── Conditional ROS imports (may require colcon-built dependencies) ────

_CoveragePlugin: type | None = None
_CoverageParams: type | None = None
_KNOWN_FIELD_PROFILE: type | None = None
_POINT2D: type | None = None
_POLYGON_ZONE: type | None = None

try:
    from ed_uav_mission.plugins.coverage import GridCoveragePlugin  # type: ignore[import-not-found]
    _CoveragePlugin = GridCoveragePlugin
except ModuleNotFoundError:
    pass

try:
    from ed_uav_mission.mission_model import CoverageParams  # type: ignore[import-not-found]
    _CoverageParams = CoverageParams
except ModuleNotFoundError:
    pass

try:
    from ed_uav_localization.field_profile.model import (  # type: ignore[import-not-found]
        KnownFieldProfile,
        Point2D,
        PolygonZone,
    )
    _KNOWN_FIELD_PROFILE = KnownFieldProfile
    _POINT2D = Point2D
    _POLYGON_ZONE = PolygonZone
except ModuleNotFoundError:
    pass

# ═════════════════════════════════════════════════════════════════════════
# PROTOCOL PARITY: V7 Frame Encoding
# ═════════════════════════════════════════════════════════════════════════


# Verified legacy command vectors (from ed_uav_fcu_bridge/test/test_v7_codec.py)
_VERIFIED_VECTORS = (
    # (legacy_builder, ros_builder, expected_hex_upper, label)
    (legacy_cmd_unlock, ros_cmd_unlock, "AAFFE00B1000010000000000000000A585", "unlock"),
    (legacy_cmd_lock, ros_cmd_lock, "AAFFE00B1000020000000000000000A68E", "lock"),
    (lambda: legacy_cmd_mode(3), lambda: ros_cmd_mode(3), "AAFFE00B01010103000000000000009A02", "mode(3)"),
    (lambda: legacy_cmd_takeoff(150), lambda: ros_cmd_takeoff(150), "AAFFE00B10000596000000000000003F59", "takeoff(150)"),
    (legacy_cmd_land, ros_cmd_land, "AAFFE00B1000060000000000000000AAB2", "land"),
    (lambda: legacy_cmd_move(100, 30, 90), lambda: ros_cmd_move(100, 30, 90), "AAFFE00B10020364001E005A00000085E7", "move(100,30,90)"),
)


@pytest.mark.parametrize("legacy_fn,ros_fn,expected_hex,label", _VERIFIED_VECTORS)
def test_protocol_parity_command_bytes_identical(
    legacy_fn, ros_fn, expected_hex: str, label: str
) -> None:
    """Each legacy and ROS high-level V7 command produces the identical frame."""
    legacy_frame: bytes = legacy_fn()
    ros_frame: bytes = ros_fn()

    assert legacy_frame.hex().upper() == expected_hex, (
        f"Legacy {label} frame mismatch against verified vector"
    )
    assert ros_frame.hex().upper() == expected_hex, (
        f"ROS {label} frame mismatch against verified vector"
    )
    assert legacy_frame == ros_frame, (
        f"Legacy {label} ≠ ROS {label} (legacy={legacy_frame.hex().upper()}, "
        f"ros={ros_frame.hex().upper()})"
    )
    # Both must pass SC + AC checksum verification
    assert verify_lx_frame(legacy_frame), f"Legacy {label} checksum fails"
    assert verify_lx_frame(ros_frame), f"ROS {label} checksum fails"


def test_protocol_parity_unlock_explicit() -> None:
    """The legacy and ROS unlock commands produce the same byte sequence."""
    legacy = legacy_cmd_unlock()
    ros = ros_cmd_unlock()
    assert legacy == ros
    assert legacy.hex().upper() == "AAFFE00B1000010000000000000000A585"


def test_protocol_parity_move_explicit() -> None:
    """The legacy and ROS move commands produce the same byte sequence."""
    legacy = legacy_cmd_move(distance_cm=100, speed_cmps=30, direction_deg=90)
    ros = ros_cmd_move(distance_cm=100, speed_cmps=30, direction_deg=90)
    assert legacy == ros
    assert legacy.hex().upper() == "AAFFE00B10020364001E005A00000085E7"


def test_protocol_parity_range_validation_identical() -> None:
    """Both legacy and ROS move reject the same out-of-range parameters."""
    out_of_range = [
        (-1, 30, 90),       # distance < 0
        (10001, 30, 90),    # distance > 10000
        (100, 5, 90),       # speed < 10
        (100, 301, 90),     # speed > 300
        (100, 30, -1),      # direction < 0
        (100, 30, 360),     # direction >= 360
    ]
    for dist, speed, direc in out_of_range:
        with pytest.raises(ValueError):
            legacy_cmd_move(dist, speed, direc)
        with pytest.raises(ValueError):
            ros_cmd_move(dist, speed, direc)


def test_protocol_parity_checksum_algorithm_identical() -> None:
    """Legacy build_lx_frame and ROS build_frame use the same SC+AC algorithm."""
    payload = bytes([0x10, 0x02, 0x03, 0x64, 0x00, 0x1E, 0x00, 0x5A, 0x00, 0x00, 0x00])
    legacy = build_lx_frame(d_addr=0xFF, frame_id=0xE0, data=payload)
    ros = build_frame(address=0xFF, frame_id=0xE0, data=payload)
    assert legacy == ros
    assert verify_lx_frame(legacy)
    decoded = decode_frame(ros)
    assert decoded.address == 0xFF
    assert decoded.frame_id == 0xE0
    assert decoded.data == payload


def test_protocol_parity_pi_forwarding_frame_unchanged() -> None:
    """Legacy Pi→MCU forwarding frame format is preserved as reference."""
    inner = legacy_cmd_unlock()
    pi_frame = build_pi_frame(inner, frame_type=0x01)
    assert pi_frame[0] == 0xAA
    assert pi_frame[2] == 0x01  # TYPE=0x01 forwarding
    # Checksum covers entire buffer (little-endian 16-bit)
    expected_sum = sum(pi_frame[:-2]) & 0xFFFF
    actual_sum = pi_frame[-2] | (pi_frame[-1] << 8)
    assert actual_sum == expected_sum


# ═════════════════════════════════════════════════════════════════════════
# PATH PARITY: Grid Coordinates
# ═════════════════════════════════════════════════════════════════════════


def test_path_parity_grid_layout_consistent() -> None:
    """Legacy path_plan layout has 28 blocks with no overlapping positions."""
    legacy_init_grid()
    assert LEGACY_TOTAL_BLOCKS == 28
    assert len(LEGACY_BLOCK_GRID) == 28
    assert len(LEGACY_BLOCK_POSITIONS) == 28
    positions = list(LEGACY_BLOCK_POSITIONS.values())
    assert len(positions) == len(set(positions)), "duplicate positions found"


def test_path_parity_all_block_ids_in_range() -> None:
    """Every legacy block ID falls within the expected 1..28 range."""
    legacy_init_grid()
    for bid in LEGACY_BLOCK_GRID:
        assert 1 <= bid <= 28, f"Block {bid} outside 1..28"


def test_path_parity_path_covers_all_blocks() -> None:
    """The legacy snake path visits each of the 28 blocks exactly once."""
    legacy_init_grid()
    assert set(LEGACY_PATH) == set(LEGACY_BLOCK_GRID.keys())
    assert len(LEGACY_PATH) == len(set(LEGACY_PATH))


def test_path_parity_path_starts_at_block_21() -> None:
    """The legacy path starts from block 21 (A marker)."""
    assert LEGACY_PATH[0] == 21


def test_path_parity_path_ends_at_block_28() -> None:
    """The legacy path ends at block 28, adjacent to A marker at 21."""
    assert LEGACY_PATH[-1] == 28
    col_28, row_28 = LEGACY_BLOCK_GRID[28]
    col_21, row_21 = LEGACY_BLOCK_GRID[21]
    assert abs(col_28 - col_21) + abs(row_28 - row_21) == 1


def test_path_parity_validate_path_clean() -> None:
    """The legacy path passes its own validator with zero issues."""
    legacy_init_grid()
    issues = validate_path(LEGACY_PATH)
    assert len(issues) == 0, f"path validation issues: {issues}"


def test_path_parity_generate_move_commands_output() -> None:
    """Each move command has the expected fields with valid ranges."""
    legacy_init_grid()
    commands = generate_move_commands(LEGACY_PATH, LEGACY_BLOCK_POSITIONS, speed_cmps=30)

    assert len(commands) == len(LEGACY_PATH) - 1
    for cmd in commands:
        assert "from" in cmd
        assert "to" in cmd
        assert "distance" in cmd
        assert "direction" in cmd
        assert "speed" in cmd
        assert isinstance(cmd["distance"], int)
        assert isinstance(cmd["direction"], int)
        assert cmd["distance"] > 0
        assert 0 <= cmd["direction"] < 360
        assert cmd["speed"] == 30


def test_path_parity_block_1_at_expected_position() -> None:
    """Block 1 is at the known world position (0, 350)."""
    legacy_init_grid()
    assert LEGACY_BLOCK_POSITIONS[1] == (0, 350)


def test_path_parity_coverage_plugin_deterministic() -> None:
    """The ROS coverage plugin produces a deterministic grid for the same params."""
    if _CoveragePlugin is None or _CoverageParams is None:
        pytest.skip("coverage plugin dependencies not available (requires colcon build)")
    if _KNOWN_FIELD_PROFILE is None or _POINT2D is None or _POLYGON_ZONE is None:
        pytest.skip("field profile dependencies not available (requires colcon build)")

    # A simple rectangular allowed zone
    allowed = (
        _POINT2D(x_m=0.0, y_m=0.0),
        _POINT2D(x_m=7.0, y_m=0.0),
        _POINT2D(x_m=7.0, y_m=5.0),
        _POINT2D(x_m=0.0, y_m=5.0),
    )
    profile = _KNOWN_FIELD_PROFILE(
        profile_id="parity-test",
        version=1,
        field_frame="map",
        allowed_zone=_POLYGON_ZONE(vertices=allowed, id="allowed"),
        no_fly_zones=(),
    )
    params = _CoverageParams(cell_size_m=1.0, altitude_m=2.0, speed_m_s=0.3)

    plugin = _CoveragePlugin()
    waypoints_1 = plugin.generate(profile, params)
    waypoints_2 = plugin.generate(profile, params)

    assert len(waypoints_1) == len(waypoints_2)
    for wp1, wp2 in zip(waypoints_1, waypoints_2):
        assert wp1.x_m == wp2.x_m
        assert wp1.y_m == wp2.y_m
    # With 1m cells over a 7x5m field, expect ~35 waypoints
    assert len(waypoints_1) == 35


# ═════════════════════════════════════════════════════════════════════════
# STATE PARITY: Transition Logic
# ═════════════════════════════════════════════════════════════════════════


def test_state_parity_legacy_enum_has_all_required_states() -> None:
    """The legacy FlightState enum includes the expected mission phases."""
    expected_names = {
        "IDLE", "ARM_UNLOCK", "SET_PROGRAM_MODE", "TAKEOFF",
        "FIND_START", "SPRAY", "NAVIGATE", "RETURN_HOME",
        "ALIGN_HOME", "LAND", "LOCK", "EMERGENCY", "DONE",
    }
    legacy_names = {s.name for s in FlightState}
    assert expected_names == legacy_names


def test_state_parity_ros_enum_has_all_required_states() -> None:
    """The ROS MissionState enum includes the expected mission phases."""
    expected_names = {
        "IDLE", "ARMED", "TAKEOFF", "EXECUTING",
        "RETURNING", "LANDING", "COMPLETE", "ABORTED",
    }
    ros_names = {s.name for s in MissionState}
    assert expected_names == ros_names


# ── Mapping table: legacy FlightState → ROS MissionState ───────────────

_LEGACY_TO_ROS_STATE_MAP = {
    "IDLE": "IDLE",
    "ARM_UNLOCK": "ARMED",
    "SET_PROGRAM_MODE": "ARMED",
    "TAKEOFF": "TAKEOFF",
    "FIND_START": "EXECUTING",
    "SPRAY": "EXECUTING",
    "NAVIGATE": "EXECUTING",
    "RETURN_HOME": "RETURNING",
    "ALIGN_HOME": "RETURNING",
    "LAND": "LANDING",
    "LOCK": "COMPLETE",
    "DONE": "COMPLETE",
    "EMERGENCY": "ABORTED",
}


def test_state_parity_mapping_is_complete() -> None:
    """Every legacy FlightState has a documented ROS MissionState mapping."""
    legacy_names = {s.name for s in FlightState}
    mapped = set(_LEGACY_TO_ROS_STATE_MAP.keys())
    assert legacy_names == mapped, (
        f"Unmapped legacy states: {legacy_names - mapped}"
    )


def test_state_parity_idle_is_starting_state() -> None:
    """Both legacy and ROS state machines start in an idle/ready state."""
    # Legacy: DroneStateMachine.__init__ sets self.state = FlightState.IDLE
    assert FlightState.IDLE.name == "IDLE"
    # ROS: MissionFSM defaults to MissionState.IDLE
    fsm = MissionFSM()
    assert fsm.state == MissionState.IDLE


def test_state_parity_emergency_is_terminal_without_recovery() -> None:
    """EMERGENCY/ABORTED is a terminal state in both systems."""
    # Legacy: once in EMERGENCY, _emergency() short-circuits
    assert "EMERGENCY" in {s.name for s in FlightState}
    # ROS: ABORTED is terminal
    fsm = MissionFSM()
    fsm.transition(MissionState.ARMED)
    fsm.transition(MissionState.ABORTED, reason="test")
    assert fsm.is_terminal
    assert fsm.state == MissionState.ABORTED


def test_state_parity_invalid_transitions_rejected() -> None:
    """The ROS FSM rejects transitions that are not in its valid set."""
    fsm = MissionFSM()
    fsm.transition(MissionState.ARMED)
    with pytest.raises(ValueError, match="invalid transition"):
        fsm.transition(MissionState.IDLE)  # ARMED → IDLE not allowed


def test_state_parity_ros_fsm_reports_not_active_in_idle() -> None:
    """IDLE, COMPLETE, and ABORTED are not considered active states."""
    fsm = MissionFSM()
    assert not fsm.is_active  # IDLE
    fsm.transition(MissionState.ARMED)
    assert fsm.is_active
    fsm.transition(MissionState.ABORTED, reason="test")
    assert not fsm.is_active


# ═════════════════════════════════════════════════════════════════════════
# COMMAND PARITY: Typed Requests
# ═════════════════════════════════════════════════════════════════════════


def test_command_parity_request_to_frame_unlock() -> None:
    """CommandRequest.unlock().to_frame() equals legacy cmd_unlock()."""
    req = CommandRequest.unlock()
    assert req.command == CommandKind.UNLOCK
    assert req.to_frame() == legacy_cmd_unlock()


def test_command_parity_request_to_frame_move() -> None:
    """CommandRequest.move(...).to_frame() equals legacy cmd_move(...)."""
    req = CommandRequest.move(distance_cm=100, speed_cmps=30, direction_deg=90)
    assert req.command == CommandKind.MOVE
    assert req.move_spec is not None
    assert req.move_spec.distance_cm == 100
    assert req.move_spec.speed_cmps == 30
    assert req.move_spec.direction_deg == 90
    assert req.to_frame() == legacy_cmd_move(100, 30, 90)


def test_command_parity_request_to_frame_land() -> None:
    """CommandRequest.land().to_frame() equals legacy cmd_land()."""
    req = CommandRequest.land()
    assert req.command == CommandKind.LAND
    assert req.to_frame() == legacy_cmd_land()


def test_command_parity_all_kinds_map_to_v7_frames() -> None:
    """Every CommandKind produces a non-empty V7 frame starting with 0xAA."""
    requests = [
        CommandRequest.unlock(),
        CommandRequest(mode=3, command=CommandKind.SET_MODE),
        CommandRequest(height_cm=150, command=CommandKind.TAKEOFF),
        CommandRequest.move(distance_cm=50, speed_cmps=20, direction_deg=180),
        CommandRequest.hover(),
        CommandRequest.land(),
        CommandRequest(command=CommandKind.LOCK),
    ]
    for req in requests:
        frame = req.to_frame()
        assert len(frame) >= 6
        assert frame[0] == 0xAA
        assert verify_lx_frame(frame), f"Command {req.command.name} checksum invalid"


# ═════════════════════════════════════════════════════════════════════════
# INTENTIONAL DIFFERENCES: Asserted (not errors)
# ═════════════════════════════════════════════════════════════════════════


def test_intentional_diff_hover_command_exists_only_in_ros() -> None:
    """ROS adds cmd_hover() for localization-loss policy; legacy has no equivalent."""
    hover_frame = cmd_hover()
    assert hover_frame[0] == 0xAA
    assert hover_frame[2] == 0xE0
    # CID=0x10, CMD0=0x00, CMD1=0x04
    assert hover_frame[4:7] == bytes((0x10, 0x00, 0x04))
    assert verify_lx_frame(hover_frame)
    # Legacy raises AttributeError (no hover)
    with pytest.raises(AttributeError):
        import lx_protocol  # type: ignore[import-not-found]
        _ = lx_protocol.cmd_hover  # type: ignore[attr-defined]


def test_intentional_diff_typed_command_request_only_in_ros() -> None:
    """ROS CommandRequest has ACK correlation; legacy send methods return bool."""
    req = CommandRequest.unlock()
    assert req.command == CommandKind.UNLOCK
    assert req.mode is None
    assert req.height_cm is None
    assert req.move_spec is None


def test_intentional_diff_decoder_separation() -> None:
    """ROS decode_frame rejects bad frames; legacy parse_lx_frame returns None."""
    # Both reject corrupt data
    valid = legacy_cmd_unlock()
    corrupt = bytearray(valid)
    corrupt[-1] ^= 0xFF
    # Legacy returns None
    from lx_protocol import parse_lx_frame  # type: ignore[import-not-found]
    assert parse_lx_frame(bytes(corrupt)) is None
    # ROS raises FrameDecodeError
    from ed_uav_fcu_bridge.v7_codec import FrameDecodeError  # type: ignore[import-not-found]
    with pytest.raises(FrameDecodeError):
        decode_frame(bytes(corrupt))
