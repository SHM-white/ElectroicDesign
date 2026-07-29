from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT / "ed_uav_vehicle_bridge"


def test_bridge_source_has_no_fcu_command_or_serial_authority() -> None:
    # Given: all production Python source in the dedicated vehicle bridge package.
    source = "\n".join(
        path.read_text(encoding="utf-8") for path in sorted(SOURCE_ROOT.glob("*.py"))
    )
    normalized = source.casefold()

    # When/Then: no FCU action type, action path, serial API, or device path exists.
    forbidden = (
        "flight" + "command",
        "/fcu/" + "flight_command",
        "import " + "serial",
        "serial." + "serial(",
        "/dev/" + "tty",
    )
    assert all(token not in normalized for token in forbidden)


def test_package_does_not_depend_on_fcu_bridge() -> None:
    manifest = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    assert "ed_uav_" + "fcu_bridge" not in manifest
