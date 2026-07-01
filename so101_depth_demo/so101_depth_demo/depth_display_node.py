"""
Minimal depth visualisation display node.

Subscribes to the colorised depth topic and shows it in a single OpenCV window.
Useful for quick local/container verification without rqt or Foxglove.

Note: requires a display/X11 (or an X forwarding / VNC setup inside the
container). If no display is available, use rqt_image_view, Foxglove, or
``ros2 topic hz`` instead.

Subscribes:
  <image_topic>   sensor_msgs/Image   encoding=rgb8

Parameters:
  image_topic   (string) topic to display (default /camera/depth/visualization)
  window_name   (string) OpenCV window title
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthDisplayNode(Node):

    def __init__(self):
        super().__init__("depth_display_node")

        self.declare_parameter("image_topic", "/camera/depth/visualization")
        self.declare_parameter("window_name", "SO-101 | Depth Anything V2 (CPU)")

        topic = self.get_parameter("image_topic").value
        self._win = self.get_parameter("window_name").value

        self._frame = None
        self.create_subscription(Image, topic, self._image_cb, 5)
        self.create_timer(1.0 / 30.0, self._render)

        cv2.namedWindow(self._win, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"DepthDisplayNode ready — showing '{topic}'")

    def _image_cb(self, msg: Image):
        enc = msg.encoding.lower()
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
        if enc == "rgb8":
            img = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2BGR)
        self._frame = np.ascontiguousarray(img)

    def _render(self):
        if self._frame is None:
            return
        cv2.imshow(self._win, self._frame)
        cv2.waitKey(1)

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthDisplayNode()
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
