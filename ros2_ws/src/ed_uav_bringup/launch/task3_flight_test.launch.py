"""Task3 flight-test launch — one-command bringup for stability-test mission.

Composes the live-flight chain with FCU bridge, mission executor, vehicle bridge,
localization, lidar (mid360), camera (dual_uvc), and AprilTag target observation.
Requires enforced SROS2, calibrated sensors, and explicit runtime inputs.
No simulation mode, no RViz, no programmable competition commands.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Target revision for AprilTag detection
_TARGET_REVISION = "d2026-apriltag-v1"

# Lidar transport mode
_LIDAR_TRANSPORT = "mid360"

# Calibration status required for flight
_CALIBRATION_STATUS = "CALIBRATED"

# Launch file names
_LIDAR_LAUNCH = "lidar.launch.py"
_FAST_LIO_LAUNCH = "fast_lio.launch.py"
_DUAL_UVC_LAUNCH = "dual_uvc.launch.py"
_TARGET_OBSERVATION_LAUNCH = "target_observation.launch.py"


def _build_actions(context):
    """Build the complete Task3 flight-test node graph."""
    mission_share = Path(get_package_share_directory("ed_uav_mission"))
    camera_share = Path(get_package_share_directory("ed_uav_camera"))
    perception_share = Path(get_package_share_directory("ed_uav_perception"))
    lidar_share = Path(get_package_share_directory("ed_uav_lidar"))

    fcu_serial_port = LaunchConfiguration("fcu_serial_port").perform(context)
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    fast_lio_launch_path = LaunchConfiguration("fast_lio_launch_path").perform(context)
    mission_config_path = LaunchConfiguration("mission_config_path").perform(context)
    field_profile_path = LaunchConfiguration("field_profile_path").perform(context)
    hmac_key_file = LaunchConfiguration("hmac_key_file").perform(context)
    task3_identity = LaunchConfiguration("task3_identity").perform(context)
    camera_runtime_plan = LaunchConfiguration("camera_runtime_plan").perform(context)
    ros_security_keystore = LaunchConfiguration("ros_security_keystore").perform(context)
    ros_security_enable = LaunchConfiguration("ros_security_enable").perform(context)
    ros_security_strategy = LaunchConfiguration("ros_security_strategy").perform(context)
    mid360_driver_config_path = LaunchConfiguration("mid360_driver_config_path").perform(context)

    actions = [
        SetEnvironmentVariable("ROS_SECURITY_ENABLE", ros_security_enable),
        SetEnvironmentVariable("ROS_SECURITY_STRATEGY", ros_security_strategy),
        SetEnvironmentVariable("ROS_SECURITY_KEYSTORE", ros_security_keystore),
    ]

    # 1. FCU bridge
    actions.append(
        Node(
            package="ed_uav_fcu_bridge",
            executable="ed_uav_fcu_bridge",
            name="ed_uav_fcu_bridge",
            output="screen",
            arguments=["--ros-args", "--enclave", "/ed_uav_fcu_bridge"],
            parameters=[
                {
                    "serial_port": fcu_serial_port,
                    "baudrate": 500000,
                    "enable_flight_commands": True,
                    "enable_realtime_control": True,
                    "enable_programmable_commands": False,
                }
            ],
        )
    )

    # 2. Vehicle bridge
    actions.append(
        Node(
            package="ed_uav_vehicle_bridge",
            executable="vehicle_bridge",
            name="vehicle_bridge",
            output="screen",
            parameters=[
                {
                    "hmac_key_file": hmac_key_file,
                    "task3_flight_test_mode": True,
                    "task3_mission_id": task3_identity,
                    "task3_field_profile_id": field_profile_path,
                }
            ],
        )
    )

    # 3. Mission executor
    actions.append(
        Node(
            package="ed_uav_mission",
            executable="mission_executor",
            name="mission_executor",
            output="screen",
            arguments=["--ros-args", "--enclave", "/ed_uav_mission_executor"],
            parameters=[
                {
                    "profile_path": field_profile_path,
                    "mission_config_path": mission_config_path,
                    "calibration_file": calibration_file,
                    "simulation_only": False,
                    "payload_config_path": str(mission_share / "config" / "payload_adapter.yaml"),
                    "programmable_capability_report": "",
                    "fcu_device_identity": "",
                }
            ],
            remappings=[
                ("/vehicle/telemetry", "/d_task/vehicle/telemetry"),
                ("/mission/status", "/d_task/mission_status"),
                ("/mission/select_d_task", "/d_task/pre_arm/select_mission"),
            ],
        )
    )

    # 4. Localization
    actions.append(
        Node(
            package="ed_uav_localization",
            executable="field_anchor",
            name="field_anchor",
            output="screen",
            parameters=[{"profile_path": field_profile_path}],
        )
    )
    actions.append(
        Node(
            package="ed_uav_localization",
            executable="source_supervisor",
            name="source_supervisor",
            output="screen",
        )
    )
    extrinsics = lidar_share / "config" / "fields" / "field_extrinsics.yaml"
    actions.append(
        Node(
            package="ed_uav_localization",
            executable="lio_adapter",
            name="lio_adapter",
            output="screen",
            parameters=[{"calibration_file": str(extrinsics)}],
        )
    )

    # 5. Lidar — MID-360
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lidar_share / "launch" / _LIDAR_LAUNCH)),
            launch_arguments={
                "lidar_enabled": "true",
                "transport": _LIDAR_TRANSPORT,
                "driver_config_path": mid360_driver_config_path,
            }.items(),
        )
    )

    # 6. FAST-LIO
    fast_lio = Path(fast_lio_launch_path)
    if not fast_lio.is_absolute():
        fast_lio = lidar_share / "config" / "fields" / fast_lio.name
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(fast_lio)),
        )
    )

    # 7. Camera — dual UVC
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(camera_share / "launch" / _DUAL_UVC_LAUNCH)),
            launch_arguments={"camera_plan": camera_runtime_plan}.items(),
        )
    )

    # 8. Perception — AprilTag target observation
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(perception_share / "launch" / _TARGET_OBSERVATION_LAUNCH)),
            launch_arguments={
                "target_revision": _TARGET_REVISION,
                "vehicle_topic": "/d_task/vehicle/telemetry",
            }.items(),
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Expose the Task3 flight-test launch surface."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("mission_config_path", description="Path to Task3 mission YAML"),
            DeclareLaunchArgument("field_profile_path", description="Path to CALIBRATED field profile"),
            DeclareLaunchArgument("calibration_file", description="Path to CALIBRATED sensor calibration"),
            DeclareLaunchArgument("camera_runtime_plan", description="Path to camera runtime plan JSON"),
            DeclareLaunchArgument("fcu_serial_port", description="FCU serial device path"),
            DeclareLaunchArgument("hmac_key_file", description="HMAC key hex file for UDP auth"),
            DeclareLaunchArgument("mid360_driver_config_path", description="MID-360 driver JSON config"),
            DeclareLaunchArgument("fast_lio_launch_path", description="FAST-LIO launch file path"),
            DeclareLaunchArgument("task3_identity", description="Task3 mission identity string"),
            DeclareLaunchArgument("ros_security_enable", default_value="true"),
            DeclareLaunchArgument("ros_security_strategy", default_value="Enforce"),
            DeclareLaunchArgument("ros_security_keystore", description="SROS2 keystore directory"),
            OpaqueFunction(function=_build_actions),
        ]
    )
