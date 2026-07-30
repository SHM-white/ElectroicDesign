#!/usr/bin/env bash
# Send a stability test mission goal to the running simulator
set -euo pipefail

echo "=== Sending stability test mission goal ==="
echo "Make sure the simulation is running first (run_stability_test_sim.sh)"
echo ""

# Wait a bit for the system to be ready
sleep 2

# Send the mission goal
ros2 action send_goal /mission/execute ed_uav_interfaces/action/ExecuteMission \
    "{
        mission_id: 'simulation-stability-test',
        field_profile_id: 'simulation-arena',
        timeout_sec: 120.0
    }" --feedback

echo ""
echo "=== Mission goal sent ==="
