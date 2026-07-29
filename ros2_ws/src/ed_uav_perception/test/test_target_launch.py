"""Installation contract for the perception-owned launch surface."""

from pathlib import Path


def test_target_observation_launch_is_installed() -> None:
    # Given
    package_root = Path(__file__).resolve().parents[1]
    setup_text = (package_root / "setup.py").read_text(encoding="utf-8")
    launch_file = package_root / "launch" / "target_observation.launch.py"

    # When / Then
    assert launch_file.is_file()
    assert 'glob("launch/*.launch.py")' in setup_text
