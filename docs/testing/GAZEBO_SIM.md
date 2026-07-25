# Gazebo Fortress Simulation

This is a simulation-only path for validating ROS graph wiring, sensor
transport, localization status, mission orchestration, and the simulator FCU
action lifecycle. It is not a V7 firmware, HIL, hardware-sensor, or flight
safety test. The field and mission files are synthetic and blocked from
competition activation. It is never a substitute for serial hardware.

## Interactive GUI

From WSLg with `DISPLAY=:0`, `WAYLAND_DISPLAY`, `XDG_RUNTIME_DIR`, and
`/mnt/wslg` available:

```bash
./tools/run_gazebo_sim.sh
```

This opens Gazebo Fortress and RViz and stays attached until `Ctrl+C`.

## Bounded Smoke

```bash
./tools/run_gazebo_smoke.sh
```

The smoke runner starts the headless simulator, checks `/clock`, enables the
simulator controller, verifies ground-truth odometry, and cleans up the
process group. Neither path opens serial hardware; the simulator owns
`/fcu/flight_command` and reports `FcuState.SOURCE_SIMULATOR`.

The simulator uses Gazebo Fortress native multicopter control and motor-model
systems. It does not claim V7 protocol, HIL timing, real sensor fidelity, or
flight readiness.
