"""Launch independent V4L2 camera nodes from a capability-probed runtime plan."""

from __future__ import annotations

import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ed_uav_camera.runtime_plan import RuntimeCamera, load_runtime_plan


def _launch_cameras(context):
    plan_path = LaunchConfiguration("camera_plan").perform(context)
    catalog_path = LaunchConfiguration("profile_catalog").perform(context)
    use_fake_devices = LaunchConfiguration("use_fake_devices").perform(context)
    use_direct_capture = LaunchConfiguration("use_direct_capture").perform(context)
    disconnect_after_frames = int(
        LaunchConfiguration("fake_wide_disconnect_after_frames").perform(context)
    )
    reconnect_after_frames = int(
        LaunchConfiguration("fake_wide_reconnect_after_frames").perform(context)
    )
    if not plan_path:
        raise RuntimeError("camera_plan is required; run P25 enumeration before real-device launch")
    if use_fake_devices not in {"true", "false"}:
        raise RuntimeError("use_fake_devices must be true or false")
    if use_direct_capture not in {"true", "false"}:
        raise RuntimeError("use_direct_capture must be true or false")
    plan = load_runtime_plan(Path(plan_path), Path(catalog_path), time.time_ns())
    return [
        _camera_node(
            camera,
            use_fake_devices == "true",
            use_direct_capture == "true",
            disconnect_after_frames,
            reconnect_after_frames,
        )
        for camera in plan.cameras
    ]


def _camera_node(
    camera: RuntimeCamera,
    use_fake_devices: bool,
    use_direct_capture: bool,
    disconnect_after_frames: int,
    reconnect_after_frames: int,
) -> Node:
    namespace = f"/camera/{camera.binding.role.value}"
    if use_fake_devices:
        return Node(
            package="ed_uav_camera",
            executable="fake_image_device",
            namespace=namespace,
            name=f"{camera.binding.role.value}_fake_image_device",
            output="screen",
            parameters=[
                {
                    "width": camera.mode.width,
                    "height": camera.mode.height,
                    "frames_per_second": camera.mode.frames_per_second,
                    "frame_id": camera.frame_id,
                    "disconnect_after_frames": disconnect_after_frames
                    if camera.binding.role.value == "wide"
                    else -1,
                    "reconnect_after_frames": reconnect_after_frames
                    if camera.binding.role.value == "wide"
                    else -1,
                }
            ],
        )
    if use_direct_capture:
        # OpenCV 直读 (MJPG): 绕开 v4l2_camera 0.6.2 的 MJPG 转换崩溃
        return Node(
            package="ed_uav_camera",
            executable="direct_uvc",
            namespace=namespace,
            name=f"{camera.binding.role.value}_direct_uvc",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {
                    "video_device": camera.binding.by_id,
                    "width": camera.mode.width,
                    "height": camera.mode.height,
                    "frames_per_second": camera.mode.frames_per_second,
                    "camera_info_url": camera.calibration.camera_info_url,
                    "frame_id": camera.frame_id,
                    "publish_width": 640,
                    "publish_height": 360,
                }
            ],
        )
    return Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace=namespace,
        name=f"{camera.binding.role.value}_v4l2_camera",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "video_device": camera.binding.by_id,
                "pixel_format": camera.mode.fourcc,
                "image_size": [camera.mode.width, camera.mode.height],
                "time_per_frame": [1, camera.mode.frames_per_second],
                "camera_info_url": camera.calibration.camera_info_url,
                "camera_frame_id": camera.frame_id,
                "use_sensor_data_qos": True,
                "use_v4l2_buffer_timestamps": True,
                "hardware_id": camera.binding.serial,
            }
        ],
    )


def generate_launch_description() -> LaunchDescription:
    """Declare strict runtime-plan arguments and defer node construction to preflight."""
    default_catalog = str(
        Path(get_package_share_directory("ed_uav_camera")) / "config" / "camera_profiles.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_plan", default_value=""),
            DeclareLaunchArgument("profile_catalog", default_value=default_catalog),
            DeclareLaunchArgument("use_fake_devices", default_value="false"),
            DeclareLaunchArgument("use_direct_capture", default_value="false"),
            DeclareLaunchArgument("fake_wide_disconnect_after_frames", default_value="-1"),
            DeclareLaunchArgument("fake_wide_reconnect_after_frames", default_value="-1"),
            OpaqueFunction(function=_launch_cameras),
        ]
    )
