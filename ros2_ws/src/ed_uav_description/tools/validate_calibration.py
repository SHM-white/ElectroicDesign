#!/usr/bin/env python3
"""Run the source-tree calibration gate without a ROS installation."""

from __future__ import annotations

import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ed_uav_description.calibration import main


raise SystemExit(main())
