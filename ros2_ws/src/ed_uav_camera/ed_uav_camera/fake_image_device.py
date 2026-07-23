"""ROS fake image device used only by dual-camera launch tests."""

from __future__ import annotations


def main() -> None:
    """Publish synthetic images and matching camera info within one camera namespace."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    class FakeImageDevice(Node):
        """A timer-driven fake UVC source with controllable disconnect/reconnect windows."""

        def __init__(self) -> None:
            super().__init__("fake_image_device")
            self.width = int(self.declare_parameter("width", 640).value)
            self.height = int(self.declare_parameter("height", 480).value)
            self.frames_per_second = int(self.declare_parameter("frames_per_second", 10).value)
            self.frame_id = str(self.declare_parameter("frame_id", "camera_optical_frame").value)
            self.disconnect_after_frames = int(self.declare_parameter("disconnect_after_frames", -1).value)
            self.reconnect_after_frames = int(self.declare_parameter("reconnect_after_frames", -1).value)
            self.frame_count = 0
            self.image_publisher = self.create_publisher(Image, "image_raw", qos_profile_sensor_data)
            camera_info_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.camera_info_publisher = self.create_publisher(CameraInfo, "camera_info", camera_info_qos)
            self.create_timer(1.0 / self.frames_per_second, self.publish_frame)

        def publish_frame(self) -> None:
            self.frame_count += 1
            is_disconnected = (
                self.disconnect_after_frames >= 0
                and self.reconnect_after_frames >= self.disconnect_after_frames
                and self.disconnect_after_frames <= self.frame_count < self.reconnect_after_frames
            )
            if is_disconnected:
                return
            stamp = self.get_clock().now().to_msg()
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = self.frame_id
            image.height = self.height
            image.width = self.width
            image.encoding = "rgb8"
            image.step = self.width * 3
            image.data = bytes(self.height * image.step)
            camera_info = CameraInfo()
            camera_info.header = image.header
            camera_info.height = self.height
            camera_info.width = self.width
            self.image_publisher.publish(image)
            self.camera_info_publisher.publish(camera_info)

    rclpy.init()
    node = FakeImageDevice()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
