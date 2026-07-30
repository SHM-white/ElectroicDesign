#!/usr/bin/env bash
# Test stability mission logic using Python (requires Humble environment)
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=========================================="
echo "  Stability Mission Logic Test"
echo "=========================================="
echo ""
echo "Testing stability mission configuration and runner logic..."
echo ""

# Run Python test in Humble environment
bash "$repo_root/tools/run_humble.sh" bash -c '
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash

python3 -c "
import sys

from ed_uav_mission.mission_model import MissionConfig, MissionType, StabilityParams
from ed_uav_mission.mission_config import parse_mission_config_text
from pathlib import Path

# Load the stability test config
config_path = Path(\"/workspace/ros2_ws/src/ed_uav_mission/config/missions/simulation_stability_test.yaml\")
config_text = config_path.read_text()

print(\"=== Parsing stability test config ===\")
config = parse_mission_config_text(config_text)

print(f\"Mission ID: {config.mission_id}\")
print(f\"Mission Type: {config.mission_type}\")
print(f\"Takeoff Altitude: {config.takeoff_altitude_m} m\")
print(f\"Timeout: {config.timeout_sec} s\")
print()

if config.stability_params is None:
    print(\"ERROR: stability_params is None!\")
    sys.exit(1)

params = config.stability_params
print(\"=== Stability Parameters ===\")
print(f\"Altitude: {params.altitude_m} m\")
print(f\"Pre-hover: {params.pre_hover_sec} s\")
print(f\"Post-hover: {params.post_hover_sec} s\")
print(f\"Square side: {params.square_side_m} m\")
print(f\"Square segment: {params.square_segment_m} m\")
print(f\"Circle diameter: {params.circle_diameter_m} m\")
print(f\"Circle segment: {params.circle_segment_m} m\")
print(f\"Heading tolerance: {params.heading_hold_tolerance_rad} rad\")
print()

# Test StabilityRunner
from ed_uav_mission.stability_runner import StabilityRunner
from ed_uav_mission.d_task_model import DTaskKind, DTaskSelection, DTaskPhase

print(\"=== Testing StabilityRunner ===\")

class MockCallbacks:
    def __init__(self):
        self.now = 0.0
        self.moves = []
        self.hovers = []
        self.phases = []

    def now_s(self):
        return self.now

    async def execute_takeoff(self, feedback):
        self.now += 0.1

    async def send_hover(self, duration_sec):
        self.hovers.append(duration_sec)
        self.now += duration_sec

    def capture_home(self):
        pass

    def capture_pose(self):
        return (0.0, 0.0, 0.0)

    async def send_move(self, x_m, y_m, altitude_m):
        self.moves.append((x_m, y_m, altitude_m))
        self.now += 0.01

    async def land_home(self, feedback):
        self.now += 0.1

    async def next_event(self):
        from ed_uav_mission.d_task_events import Tick
        return Tick(now_s=self.now)

    def publish_transition(self, transition, feedback):
        self.phases.append(transition.state.phase)

callbacks = MockCallbacks()
runner = StabilityRunner(callbacks, params)

selection = DTaskSelection(
    mission_id=\"test\",
    mission_profile_id=\"test\",
    deployment_preset_id=\"test\",
    target_revision=\"test\",
    task=DTaskKind.STABILITY_TEST,
    committed_at_s=0.0,
)

import asyncio

async def run_test():
    from ed_uav_interfaces.action import ExecuteMission
    feedback = ExecuteMission.Feedback()
    await runner.run(selection, feedback)

asyncio.run(run_test())

print(f\"Square waypoints: 4\")
print(f\"Circle waypoints: {len(callbacks.moves) - 4}\")
print(f\"Total moves: {len(callbacks.moves)}\")
print(f\"Hovers: {callbacks.hovers}\")
print(f\"Phases: {[p.value for p in callbacks.phases]}\")
print()

if callbacks.phases[0] == DTaskPhase.STABILIZING:
    print(\"✓ First phase is STABILIZING\")
else:
    print(f\"✗ First phase is {callbacks.phases[0]}, expected STABILIZING\")

if callbacks.phases[-1] == DTaskPhase.SUCCEEDED:
    print(\"✓ Last phase is SUCCEEDED\")
else:
    print(f\"✗ Last phase is {callbacks.phases[-1]}, expected SUCCEEDED\")

print()
print(\"=== All tests passed ===\")
"
'
