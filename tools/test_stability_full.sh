#!/usr/bin/env bash
# One-click script to launch stability test simulation and send the mission goal
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=========================================="
echo "  Stability Test Simulation Runner"
echo "=========================================="
echo ""
echo "This script will:"
echo "1. Build the workspace"
echo "2. Launch Gazebo simulation with stability test mission"
echo "3. Wait for system to be ready"
echo "4. Send the stability test mission goal"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch simulation in background
"$repo_root/tools/run_stability_test_sim.sh" &
sim_pid=$!

# Wait for simulation to start
echo "Waiting for simulation to start..."
sleep 15

# Check if simulation is still running
if ! kill -0 "$sim_pid" 2>/dev/null; then
    echo "ERROR: Simulation failed to start"
    exit 1
fi

echo ""
echo "=== Simulation started, sending mission goal ==="
echo ""

# Send the mission goal
"$repo_root/tools/send_stability_goal.sh"

# Wait for simulation to finish
echo ""
echo "=== Waiting for simulation to complete ==="
wait "$sim_pid" || true

echo ""
echo "=== Simulation complete ==="
