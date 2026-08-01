"""Task3 flight-test launch — one-command bringup for stability-test mission.

Composes the live-flight chain with FCU bridge, mission executor, vehicle bridge,
localization, lidar (mid360), camera (dual_uvc), and AprilTag target observation.
Requires enforced SROS2, calibrated sensors, and explicit runtime inputs.
No simulation mode, no RViz, no programmable competition commands.

``dry_run:=true`` starts every module except the flight controller — vehicle
bridge (ground station), lidar odometry chain, dual cameras, AprilTag tracking,
H7 GPIO (electromagnet/laser) and mission display — for offline chain self-test.
Lidar chain is skipped when the MID-360 is unreachable, mirroring field_test.sh.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import yaml
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
_TASK3_MISSION_PROFILE_ID = "task3-stability"
_TASK3_DEPLOYMENT_PRESET_ID = "field-2026"

# Lidar transport mode
_LIDAR_TRANSPORT = "mid360"

# Calibration status required for flight
_CALIBRATION_STATUS = "CALIBRATED"

# Launch file names
_LIDAR_LAUNCH = "lidar.launch.py"
_FAST_LIO_LAUNCH = "fast_lio.launch.py"
_DUAL_UVC_LAUNCH = "dual_uvc.launch.py"
_TARGET_OBSERVATION_LAUNCH = "target_observation.launch.py"

# UDP endpoints — same three-party layout as full_competition.launch.py
_NUC_IP = "192.168.20.1"
_CAR_IP = "192.168.20.2"
_HMI_IP = "192.168.20.3"
_CAR_SENDER_ID = 0x43415231  # "CAR1"
_HMI_SENDER_ID = 0x484D4931  # "HMI1"
_BRIDGE_SENDER_ID = 0x524F5331  # "ROS1"


def _lidar_reachable(ip: str) -> bool:
    """Ping the MID-360; mirrors the reachability gate in field_test.sh."""
    try:
        return (
            subprocess.run(
                ["ping", "-c", "1", "-W", "1", ip],
                capture_output=True,
                timeout=3,
                check=False,
            ).returncode
            == 0
        )
    except (OSError, subprocess.SubprocessError):
        return False


def _resolve_serial(context, requested_fcu: str, requested_h7: str):
    """串口自动检测: 复用 full_competition.launch.py 的 detect_or_fallback。

    通过探测指令识别 H7 GPIO 板 (0xBB 响应) 与凌霄飞控 (V7 遥测帧),
    无需硬编码设备路径; dry-run (无飞控) 直接回退默认值。
    """
    if not Path(requested_fcu).exists() or not Path(requested_h7).exists():
        try:
            from ed_uav_fcu_bridge.serial_detect import detect_or_fallback

            detected = detect_or_fallback(
                default_h7=requested_h7,
                default_fcu=requested_fcu,
                probe_timeout=1.0,
            )
            print(
                f"[task3] 串口自动检测: fcu={detected['fcu']} "
                f"h7_gpio={detected['h7_gpio']}"
            )
            return detected["fcu"], detected["h7_gpio"]
        except Exception as error:  # noqa: BLE001 - fall back to requested paths
            print(f"[task3] 串口自动检测失败 ({error}), 使用请求路径")
    return requested_fcu, requested_h7


def _build_actions(context):
    """Build the complete Task3 flight-test node graph."""
    mission_share = Path(get_package_share_directory("ed_uav_mission"))
    camera_share = Path(get_package_share_directory("ed_uav_camera"))
    perception_share = Path(get_package_share_directory("ed_uav_perception"))
    lidar_share = Path(get_package_share_directory("ed_uav_lidar"))
    fcu_bridge_share = Path(get_package_share_directory("ed_uav_fcu_bridge"))

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
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() in ("true", "1", "yes")
    h7_serial_port = LaunchConfiguration("h7_serial_port").perform(context)
    lidar_ip = LaunchConfiguration("lidar_ip").perform(context)
    vehicle_bind_host = LaunchConfiguration("vehicle_bind_host").perform(context)

    # 串口自动检测 (仅实飞; dry-run 无飞控/H7 可跳过)
    if not dry_run:
        fcu_serial_port, h7_serial_port = _resolve_serial(
            fcu_serial_port, h7_serial_port
        )

    actions = [
        SetEnvironmentVariable("ROS_SECURITY_ENABLE", ros_security_enable),
        SetEnvironmentVariable("ROS_SECURITY_STRATEGY", ros_security_strategy),
        SetEnvironmentVariable("ROS_SECURITY_KEYSTORE", ros_security_keystore),
    ]

    # 1. FCU bridge — skipped in dry-run (no flight control)
    if not dry_run:
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

    # 2. Vehicle bridge — 需要 profile_id(而非路径),与 executor 的
    #    DTaskSelectionContract 匹配,否则小车选择请求会被拒绝
    profile_document = yaml.safe_load(Path(field_profile_path).read_text(encoding="utf-8"))
    field_profile_id = str(profile_document["profile_id"])
    actions.append(
        Node(
            package="ed_uav_vehicle_bridge",
            executable="vehicle_bridge",
            name="vehicle_bridge",
            output="screen",
            parameters=[
                {
                    "bind_host": vehicle_bind_host,
                    "bind_port": 42000,
                    "car_peer_host": _CAR_IP,
                    "car_peer_port": 42001,
                    "hmi_peer_host": _HMI_IP,
                    "hmi_peer_port": 42002,
                    "car_sender_id": _CAR_SENDER_ID,
                    "hmi_sender_id": _HMI_SENDER_ID,
                    "bridge_sender_id": _BRIDGE_SENDER_ID,
                    "hmac_key_file": hmac_key_file,
                    "mission_timeout_seconds": 90.0,
                    "telemetry_stale_seconds": 0.75,
                    "task3_flight_test_mode": True,
                    # launch 参数是字符串 ("true"/"false"), 必须转成 BOOL,
                    # 否则 vehicle_bridge 的 declare_parameter(BOOL) 会抛
                    # InvalidParameterTypeException
                    "task3_immediate_start": (
                        LaunchConfiguration("task3_immediate_start").perform(context).lower()
                        in ("true", "1", "yes")
                    ),
                    "task3_mission_id": task3_identity,
                    "task3_field_profile_id": field_profile_id,
                    "task3_mission_profile_id": _TASK3_MISSION_PROFILE_ID,
                    "task3_deployment_preset_id": _TASK3_DEPLOYMENT_PRESET_ID,
                    "task3_target_revision": _TARGET_REVISION,
                }
            ],
        )
    )

    # 2.5 Dry-run vehicle telemetry simulator — no real car on the bench, so
    #     feed the perception/display chain synthetic telemetry
    if dry_run:
        actions.append(
            Node(
                package="ed_uav_vehicle_bridge",
                executable="dry_run_telemetry",
                name="dry_run_telemetry",
                output="screen",
            )
        )

    # 3. H7 GPIO bridge (electromagnet/laser) — skipped in dry-run when the
    #    board is not attached (node exits on serial open failure by design)
    h7_device_present = Path(h7_serial_port).exists() if dry_run else True
    if not h7_device_present:
        print(f"[dry-run] {h7_serial_port} not present — H7 GPIO (electromagnet/laser) skipped")
    if h7_device_present:
        actions.append(
            Node(
                package="ed_uav_fcu_bridge",
                executable="ed_uav_h7_gpio_bridge",
                name="ed_uav_h7_gpio_bridge",
                output="screen",
                parameters=[
                    {
                        "serial_port": h7_serial_port,
                        "baudrate": 115200,
                    }
                ],
            )
        )

    # 4. Mission executor
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
                    "payload_actuator": "h7" if not dry_run else "fake",
                    "programmable_capability_report": "",
                    "fcu_device_identity": "",
                    "task3_mission_profile_id": _TASK3_MISSION_PROFILE_ID,
                    "task3_deployment_preset_id": _TASK3_DEPLOYMENT_PRESET_ID,
                    "task3_target_revision": _TARGET_REVISION,
                }
            ],
            remappings=[
                ("/vehicle/telemetry", "/d_task/vehicle/telemetry"),
                ("/mission/status", "/d_task/mission_status"),
                ("/mission/select_d_task", "/d_task/pre_arm/select_mission"),
            ],
        )
    )

    # 5. Localization
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

    # 6. Lidar + FAST-LIO — gated on MID-360 reachability in dry-run
    lidar_chain_up = not dry_run or _lidar_reachable(lidar_ip)
    if not lidar_chain_up:
        print(
            f"[dry-run] MID-360 {lidar_ip} unreachable — lidar/FAST-LIO/lio_adapter skipped "
            "(same gate as field_test.sh)"
        )
    if lidar_chain_up:
        # field_extrinsics.yaml lives in src (not installed into share) — keep
        # pointing at the repo copy, same as task3.sh's other src-relative inputs
        repo_root = Path(__file__).resolve().parents[4]
        extrinsics = repo_root / "ros2_ws" / "src" / "ed_uav_lidar" / "config" / "fields" / "field_extrinsics.yaml"
        actions.append(
            Node(
                package="ed_uav_localization",
                executable="lio_adapter",
                name="lio_adapter",
                output="screen",
                parameters=[{"calibration_file": str(extrinsics)}],
            )
        )
        # MID-360 参数必须从 manifest 完整解析 (serial/sensor_ip/firmware/driver_json),
        # 否则 lidar.launch.py 报 LIDAR_FIELD_CONFIGURATION_INCOMPLETE 不启动驱动 —
        # 与 field_test.sh 的 manifest_value 逻辑一致
        lidar_manifest = json.loads(Path(mid360_driver_config_path).read_text(encoding="utf-8"))
        lidar_serial = str(lidar_manifest["serial_number"])
        lidar_sensor_ip = str(lidar_manifest["lidar_ip"])
        lidar_host_ip = str(lidar_manifest["host_ip"])
        lidar_firmware = str(lidar_manifest["firmware"])
        lidar_driver_json = Path(lidar_manifest["driver_json"])
        if not lidar_driver_json.is_absolute():
            lidar_driver_json = Path(mid360_driver_config_path).parent / lidar_driver_json
        actions.append(
            SetEnvironmentVariable("MID360_HOST_IP", lidar_host_ip)
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(lidar_share / "launch" / _LIDAR_LAUNCH)),
                launch_arguments={
                    "lidar_enabled": "true",
                    "transport": _LIDAR_TRANSPORT,
                    "serial_number": lidar_serial,
                    "sensor_ip": lidar_sensor_ip,
                    "firmware_version": lidar_firmware,
                    "time_authority": "host",
                    "driver_config_path": str(lidar_driver_json),
                }.items(),
            )
        )
        fast_lio = Path(fast_lio_launch_path)
        if not fast_lio.is_absolute():
            fast_lio = lidar_share / "config" / "fields" / fast_lio.name
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(fast_lio)),
            )
        )

    # 7. Camera — dual UVC (OpenCV direct capture: v4l2_camera cannot handle MJPG)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(camera_share / "launch" / _DUAL_UVC_LAUNCH)),
            launch_arguments={
                "camera_plan": camera_runtime_plan,
                "use_direct_capture": "true",
            }.items(),
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

    # 9. Mission display (optional — enabled by --enable-display flag)
    enable_display = LaunchConfiguration("enable_display").perform(context)
    if enable_display.lower() in ("true", "1", "yes"):
        actions.append(
            Node(
                package="ed_uav_mission",
                executable="mission_display",
                name="mission_display",
                output="screen",
                arguments=["--ros-args", "--enclave", "/ed_uav_mission_display"],
                parameters=[
                    {
                        "max_display_width": 960,
                        "headless_log_interval_sec": 5.0,
                    }
                ],
                remappings=[
                    ("/mission/status", "/d_task/mission_status"),
                ],
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
            DeclareLaunchArgument("enable_display", default_value="false", description="Enable mission display window"),
            DeclareLaunchArgument(
                "vehicle_bind_host",
                default_value=_NUC_IP,
                description="vehicle_bridge UDP bind host (dry-run 建议 0.0.0.0)",
            ),
            DeclareLaunchArgument(
                "task3_immediate_start",
                default_value="false",
                description="地面站选择提交后立即启动任务, 不等待小车 START / AUX gate (调试用)",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="Start all modules except the flight controller (offline chain self-test)",
            ),
            DeclareLaunchArgument(
                "h7_serial_port",
                default_value="/dev/ttyUSB1",
                description="H7 GPIO (electromagnet/laser) serial device",
            ),
            DeclareLaunchArgument(
                "lidar_ip",
                default_value="192.168.1.3",
                description="MID-360 IP for dry-run reachability gate",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
