"""Direct OpenCV UVC capture node — bypasses v4l2_camera MJPG conversion bug.

``v4l2_camera`` 0.6.2 cannot convert MJPG frames (crashes with
"Unrecognized image encoding []"), so live cameras are read directly with
``cv2.VideoCapture`` + MJPG fourcc — the same proven path as field_test_node
direct capture — and published as bgr8 ``Image`` plus calibrated ``CameraInfo``.
"""

from __future__ import annotations

import threading
from pathlib import Path
from urllib.parse import urlparse

import cv2
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image


class DirectUvcNode(Node):

    def __init__(self) -> None:
        super().__init__("direct_uvc")
        self.declare_parameter("video_device", "/dev/video0")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("frames_per_second", 20)
        self.declare_parameter("camera_info_url", "")
        self.declare_parameter("frame_id", "camera_optical_frame")

        self._device = str(self.get_parameter("video_device").value)
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._fps = int(self.get_parameter("frames_per_second").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        info_url = str(self.get_parameter("camera_info_url").value)

        self._image_pub = self.create_publisher(Image, "image_raw", qos_profile_sensor_data)
        info_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._info_pub = self.create_publisher(CameraInfo, "camera_info", info_qos)

        self._camera_info = self._load_camera_info(info_url) if info_url else None
        if self._camera_info is None:
            self.get_logger().warn("camera_info unavailable; publishing zero matrices")

        self._capture = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not self._capture.isOpened():
            self.get_logger().error(f"cannot open camera: {self._device}")
            raise SystemExit(1)
        self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._capture.set(cv2.CAP_PROP_FPS, self._fps)

        actual_w = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f"direct capture {self._device}: {actual_w}x{actual_h}@~{self._fps}fps"
        )

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _load_camera_info(self, camera_info_url: str) -> CameraInfo | None:
        try:
            url = urlparse(camera_info_url)
            path = Path(url.path)
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as error:
            self.get_logger().warn(f"cannot load camera_info {camera_info_url}: {error}")
            return None
        message = CameraInfo()
        message.header.frame_id = self._frame_id
        message.width = int(document["image_width"])
        message.height = int(document["image_height"])
        message.distortion_model = str(document.get("distortion_model", "plumb_bob"))
        message.d = [
            float(value) for value in document["distortion_coefficients"]["data"]
        ]
        matrix = document["camera_matrix"]["data"]
        message.k = [float(value) for value in matrix]
        rect = document.get("rectification_matrix", {}).get("data", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        message.r = [float(value) for value in rect]
        projection = document.get("projection_matrix", {}).get("data")
        if projection:
            message.p = [float(value) for value in projection]
        else:
            fx, fy, cx, cy = matrix[0], matrix[4], matrix[2], matrix[5]
            message.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return message

    def _capture_loop(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            ok, frame = self._capture.read()
            if not ok or frame is None:
                continue
            stamp = self.get_clock().now().to_msg()
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = self._frame_id
            image.height, image.width = frame.shape[:2]
            image.encoding = "bgr8"
            image.step = frame.strides[0]
            image.data = frame.tobytes()
            self._image_pub.publish(image)
            info = self._camera_info
            if info is not None:
                info.header.stamp = stamp
                self._info_pub.publish(info)

    def destroy_node(self) -> None:
        self._stop.set()
        self._capture.release()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DirectUvcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
