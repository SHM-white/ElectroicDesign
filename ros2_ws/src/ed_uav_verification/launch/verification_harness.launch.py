"""Launch the finite wall-time ED UAV verification publisher."""

import signal

from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.events.process import ProcessIO, SignalProcess
from launch.substitutions import LaunchConfiguration

WALL_TIME_ERROR = (
    "live deterministic publisher has no /clock and requires use_sim_time=false"
)
FINITE_PROCESS_WRAPPER = (
    "trap 'kill -TERM -- -\"$child\" 2>/dev/null; "
    "wait \"$child\" 2>/dev/null; exit 0' TERM; "
    'setsid "$@" & child=$!; wait "$child"'
)


def _build_actions(context: LaunchContext) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    if use_sim_time:
        raise RuntimeError(WALL_TIME_ERROR)

    seed = LaunchConfiguration("seed")
    duration_seconds = LaunchConfiguration("duration_seconds")
    rate_hz = LaunchConfiguration("rate_hz")
    publisher = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            FINITE_PROCESS_WRAPPER,
            "--",
            "ros2",
            "run",
            "ed_uav_verification",
            "ed-uav-verify-ros",
            "--seed",
            seed,
            "--duration-seconds",
            duration_seconds,
            "--rate-hz",
            rate_hz,
        ],
        output="screen",
    )

    def stop_on_completion(event: ProcessIO) -> list[Action]:
        if b"ROS SCENARIO: GREEN virtual replay completed" not in event.text:
            return []
        return [
            EmitEvent(
                event=SignalProcess(
                    process_matcher=lambda candidate: candidate is publisher,
                    signal_number=signal.SIGTERM,
                )
            )
        ]

    return [
        publisher,
        RegisterEventHandler(
            OnProcessIO(
                target_action=publisher,
                on_stdout=stop_on_completion,
                on_stderr=stop_on_completion,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=publisher,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="finite verification publisher completed")
                    )
                ],
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Expose the canonical seeded wall-time 60-second/20Hz launch surface."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("seed", default_value="7"),
            DeclareLaunchArgument("duration_seconds", default_value="60"),
            DeclareLaunchArgument("rate_hz", default_value="20"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_build_actions),
        ]
    )
