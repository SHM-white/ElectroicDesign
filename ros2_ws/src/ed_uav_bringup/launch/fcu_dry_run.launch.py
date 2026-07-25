"""Bounded FCU dry-run using only a fake PTY; no flight hardware is opened."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ed_uav_description.calibration import ExpectedSerials, load_calibration, validate_for_profile
from ed_uav_description.model import render_robot_description

PROFILE = "offline"
LIFECYCLE_ORDER = ("calibration_gate", "fake_fcu_ready", "fcu_bridge", "bounded_shutdown")
READY_PREFIX = b"FAKE FCU READY:"


def _build_actions(context: LaunchContext) -> list[Action]:
    calibration_path = Path(LaunchConfiguration("calibration_file").perform(context))
    calibration = load_calibration(calibration_path)
    validate_for_profile(
        calibration,
        PROFILE,
        ExpectedSerials(
            LaunchConfiguration("camera_narrow_serial").perform(context),
            LaunchConfiguration("camera_wide_serial").perform(context),
            LaunchConfiguration("lidar_serial").perform(context),
        ),
    )
    description_share = Path(get_package_share_directory("ed_uav_description"))
    robot_description = render_robot_description(calibration, description_share / "urdf" / "ed_uav.urdf.xacro")
    duration_seconds = float(LaunchConfiguration("duration_seconds").perform(context))
    pty_device = LaunchConfiguration("pty_device").perform(context)

    fake_fcu = Node(
        package="ed_uav_verification",
        executable="ed-uav-fake-fcu",
        name="ed_uav_fake_fcu",
        output="screen",
        arguments=[
            "--pty-device",
            LaunchConfiguration("pty_device"),
            "--duration-seconds",
            LaunchConfiguration("duration_seconds"),
            "--rate-hz",
            LaunchConfiguration("rate_hz"),
            "--seed",
            LaunchConfiguration("seed"),
        ],
    )
    bridge = Node(
        package="ed_uav_fcu_bridge",
        executable="ed_uav_fcu_bridge",
        name="ed_uav_fcu_bridge",
        output="screen",
        parameters=[
            {
                "serial_port": pty_device,
                "enable_experimental_0x32_0x33": False,
            }
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return [
        RegisterEventHandler(
            OnProcessIO(
                target_action=fake_fcu,
                on_stdout=lambda event: [bridge] if event.text.startswith(READY_PREFIX) else [],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=fake_fcu,
                on_exit=[EmitEvent(event=Shutdown(reason="bounded fake FCU exited"))],
            )
        ),
        fake_fcu,
        robot_state_publisher,
        TimerAction(
            period=duration_seconds + 1.0,
            actions=[EmitEvent(event=Shutdown(reason="FCU dry-run duration elapsed"))],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    description_share = Path(get_package_share_directory("ed_uav_description"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pty_device",
                default_value="/tmp/ed-uav-fake-fcu-pty",
                description="Caller-visible symlink to the fake Linux PTY; never a hardware serial device.",
            ),
            DeclareLaunchArgument("duration_seconds", default_value="10"),
            DeclareLaunchArgument("rate_hz", default_value="20"),
            DeclareLaunchArgument("seed", default_value="7"),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(description_share / "config" / "example_uncalibrated.yaml"),
            ),
            DeclareLaunchArgument("camera_narrow_serial", default_value="UNSET"),
            DeclareLaunchArgument("camera_wide_serial", default_value="UNSET"),
            DeclareLaunchArgument("lidar_serial", default_value="UNSET"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "authority_token",
                default_value="ed-uav-fcu-dry-run",
                description="Single control-authority token for this profile.",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
