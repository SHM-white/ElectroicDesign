"""Interactive calibration CLI boundary tests."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "tools" / "calibration"))

from calibrate_chessboard import _choose_device
from ed_uav_camera.calibration_models import CalibrationBootstrapError
from ed_uav_camera.device_discovery import StableVideoDevice


def test_device_selection_reports_unavailable_standard_input(monkeypatch) -> None:
    # Given: one discovered device but no interactive standard input.
    device = StableVideoDevice("CAMERA-1", "/dev/v4l/by-id/camera", "/dev/video0")
    monkeypatch.setattr("builtins.input", lambda _prompt: (_ for _ in ()).throw(EOFError()))

    # When/Then: the CLI returns a stable domain error rather than leaking a traceback.
    with pytest.raises(CalibrationBootstrapError, match="camera selection requires interactive input"):
        _choose_device((device,))
