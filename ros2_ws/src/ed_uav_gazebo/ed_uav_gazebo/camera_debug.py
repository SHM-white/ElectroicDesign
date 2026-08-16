#!/usr/bin/env python3
"""Save camera frames at intervals during flight for inspection."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time


class FrameSaver(Node):
    def __init__(self):
        super().__init__('camera_debug')
        self._bridge = CvBridge()
        self._frame_count = 0
        self._save_count = 0
        self._save_dir = '/workspace/camera_frames'
        os.makedirs(self._save_dir, exist_ok=True)
        self._dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self._params = cv2.aruco.DetectorParameters_create()
        self._params.adaptiveThreshWinSizeMax = 201
        self._params.adaptiveThreshConstant = 3
        self._params.minDistanceToBorder = 0
        self._params.adaptiveThreshWinSizeStep = 4
        self.create_subscription(Image, '/camera/narrow/image_raw', self._on_image, 10)
        self.get_logger().info(f'FrameSaver: saving to {self._save_dir}')

    def _on_image(self, msg):
        self._frame_count += 1
        # Save every 10th frame (~1.5Hz)
        if self._frame_count % 10 != 0:
            return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        self._save_count += 1
        fname = f'{self._save_dir}/frame_{self._save_count:04d}.png'
        cv2.imwrite(fname, img)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        self.get_logger().info(
            f'Frame {self._save_count}: {w}x{h} '
            f'min={gray.min()} max={gray.max()} mean={gray.mean():.1f}'
        )

        corners, ids, rej = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        if ids is not None and len(ids) > 0:
            self.get_logger().info(f'  >>> DETECTED tag id={ids.flatten()} <<<')
        else:
            self.get_logger().info(f'  No tag (rejected={len(rej)})')


def main(args=None):
    rclpy.init(args=args)
    node = FrameSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
