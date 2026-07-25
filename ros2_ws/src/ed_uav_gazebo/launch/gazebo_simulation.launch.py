"""Compatibility entry point for the persistent Gazebo simulator runners."""

import importlib.util
from pathlib import Path


_IMPLEMENTATION = Path(__file__).with_name("sim.launch.py")
_SPEC = importlib.util.spec_from_file_location("ed_uav_gazebo_sim_launch", _IMPLEMENTATION)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError(f"cannot load simulator launch implementation: {_IMPLEMENTATION}")
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
generate_launch_description = _MODULE.generate_launch_description

__all__ = ("generate_launch_description",)
