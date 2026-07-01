"""
Synthetic test image publisher.

Publishes a moving gradient / shapes image as sensor_msgs/Image so the depth
demo can be exercised in a container with no physical camera and no simulated
camera feed. Optionally publishes a static image file instead.

Publishes:
  <image_topic>   sensor_msgs/Image   encoding=bgr8

Parameters:
  image_topic   (string) topic to publish on (default /follower/image_raw)
  image_path    (string) optional path to a static image file; if empty, a
                         synthetic animated scene is generated
  width         (int)    synthetic image width  (default 640)
  height        (int)    synthetic image height (default 480)
  fps           (float)  publish rate (default 15.0)
  frame_id      (string) header frame_id (default camera)
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class TestImagePublisher(Node):

    def __init__(self):
        super().__init__("test_image_publisher")

        self.declare_parameter("image_topic", "/follower/image_raw")
        self.declare_parameter("image_path", "")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_id", "camera")

        topic = self.get_parameter("image_topic").value
        self._path = self.get_parameter("image_path").value
        self._w = int(self.get_parameter("width").value)
        self._h = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        self._frame_id = self.get_parameter("frame_id").value

        self._static = None
        if self._path:
            img = cv2.imread(self._path, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().error(f"Could not read image: {self._path}")
            else:
                self._static = img
                self._h, self._w = img.shape[:2]
                self.get_logger().info(f"Publishing static image: {self._path}")

        self._pub = self.create_publisher(Image, topic, 5)
        self._k = 0
        self.create_timer(1.0 / max(fps, 0.1), self._tick)
        self.get_logger().info(
            f"TestImagePublisher ready — '{topic}' @ {fps} Hz ({self._w}x{self._h})"
        )

    def _synthetic_frame(self) -> np.ndarray:
        """Animated scene with depth-like structure (shapes at varied 'depths')."""
        img = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        # Background vertical gradient.
        grad = np.linspace(40, 200, self._h, dtype=np.uint8)
        img[:] = grad[:, None, None]

        t = self._k * 0.08
        cx = int(self._w * (0.5 + 0.3 * math.sin(t)))
        cy = int(self._h * 0.55)
        cv2.circle(img, (cx, cy), 70, (60, 180, 240), -1)
        cv2.rectangle(
            img, (int(self._w * 0.1), int(self._h * 0.2)),
            (int(self._w * 0.3), int(self._h * 0.5)), (200, 120, 60), -1
        )
        cv2.rectangle(
            img, (int(self._w * 0.7), int(self._h * 0.3)),
            (int(self._w * 0.9), int(self._h * 0.7)), (120, 220, 120), -1
        )
        return img

    def _tick(self):
        frame = self._static if self._static is not None else self._synthetic_frame()
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        self._pub.publish(msg)
        self._k += 1


def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
