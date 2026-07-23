"""ROS 2 detector node: subscribes to camera, publishes Detection2DArray."""

from __future__ import annotations

import time as _time
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time as RosTime
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray

from ed_uav_perception.provider_interface import DetectorProvider


def _ros_time_to_sec(stamp: RosTime) -> float:
    """Convert a builtin_interfaces/Time to seconds since epoch."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _now_sec() -> float:
    """Return current wall-clock time in seconds as float."""
    return _time.time()


class DetectorNode(Node):
    """ROS 2 node that runs narrow-camera detection at a configurable rate.

    Subscriptions
    -------------
    ``/camera/narrow/image_raw`` (sensor_msgs/Image)
    ``/camera/narrow/camera_info`` (sensor_msgs/CameraInfo)

    Publications
    ------------
    ``/perception/detections`` (vision_msgs/Detection2DArray)

    Parameters
    ----------
    rate_limit_hz : float (default 10.0)
        Maximum inference frequency in Hz.
    stale_threshold_sec : float (default 0.5)
        Images older than this are rejected.
    model_version : str
        Provider model version (read-only diagnostic).
    provider_type : str
        Provider type identifier (read-only diagnostic).
    """

    def __init__(
        self,
        *,
        provider: Optional[DetectorProvider] = None,
        node_name: str = "detector_node",
    ) -> None:
        super().__init__(node_name)

        # --- Parameters ---
        self.declare_parameter("rate_limit_hz", 10.0)
        self.declare_parameter("stale_threshold_sec", 0.5)

        # --- Provider (injected or default Mock) ---
        if provider is None:
            from ed_uav_perception.provider_interface import MockDetectorProvider

            provider = MockDetectorProvider()
        self._provider: DetectorProvider = provider

        # Declare read-only diagnostic parameters.
        self.declare_parameter("model_version", self._provider.version)
        self.declare_parameter("provider_type", self._provider.provider_type)
        self.declare_parameter("latency_ms", -1.0)
        self.declare_parameter("last_detection_stamp", "")

        # --- State ---
        self._bridge = CvBridge()
        self._camera_info: Optional[CameraInfo] = None
        self._last_inference_time: float = 0.0
        self._detection_count: int = 0

        # --- Subscriptions ---
        self._image_sub = self.create_subscription(
            Image,
            "/camera/narrow/image_raw",
            self._image_callback,
            10,
        )
        self._info_sub = self.create_subscription(
            CameraInfo,
            "/camera/narrow/camera_info",
            self._info_callback,
            10,
        )

        # --- Publisher ---
        self._detection_pub = self.create_publisher(Detection2DArray, "/perception/detections", 10)

        self.get_logger().info(
            f"DetectorNode started with provider={self._provider.provider_type} "
            f"v{self._provider.version}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _info_callback(self, msg: CameraInfo) -> None:
        """Store the latest CameraInfo for downstream geometry."""
        self._camera_info = msg

    def _image_callback(self, msg: Image) -> None:
        """Process incoming images with rate limiting and staleness checks."""
        rate_hz = self.get_parameter("rate_limit_hz").value
        stale_s = self.get_parameter("stale_threshold_sec").value

        # Reject stale images.
        img_sec = _ros_time_to_sec(msg.header.stamp)
        age = _now_sec() - img_sec
        if age > stale_s:
            self.get_logger().debug(
                f"Rejecting stale image: age={age:.3f}s > {stale_s}s",
                throttle_duration_sec=5.0,
            )
            return

        # Rate limit.
        now = _now_sec()
        interval = 1.0 / max(rate_hz, 0.1)
        if now - self._last_inference_time < interval:
            return
        self._last_inference_time = now

        # Run inference (isolated from node crash).
        t0 = _now_sec()
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            detections = self._provider.detect(cv_image)
        except Exception as exc:
            self.get_logger().error(
                f"Provider detect() raised {type(exc).__name__}: {exc}",
                throttle_duration_sec=5.0,
            )
            detections = []

        latency_ms = (_now_sec() - t0) * 1000.0

        # Populate detection headers.
        for det in detections:
            det.header = msg.header

        # Publish.
        array = Detection2DArray()
        array.header = msg.header
        array.detections = detections
        self._detection_pub.publish(array)

        # Update diagnostics.
        self._detection_count += len(detections)
        self.set_parameters([
            Parameter("latency_ms", value=latency_ms),
            Parameter(
                "last_detection_stamp",
                value=f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}",
            ),
        ])

    # ------------------------------------------------------------------
    # Public accessors (used by tests)
    # ------------------------------------------------------------------

    @property
    def provider(self) -> DetectorProvider:
        """Return the active detector provider."""
        return self._provider

    @property
    def camera_info(self) -> Optional[CameraInfo]:
        """Return the last-received CameraInfo, or None."""
        return self._camera_info

    @property
    def detection_count(self) -> int:
        """Total number of detections published (cumulative)."""
        return self._detection_count


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
