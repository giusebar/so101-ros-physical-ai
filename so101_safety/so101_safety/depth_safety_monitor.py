"""
Depth-safety monitor — perception-driven protective-stop trigger.

Consumes an already-computed normalised depth map (published by e.g. the
``depthanything`` snap / ``so101_depth_demo``'s ``depth_anything_node``, or any
other node honouring the same contract) and raises a Bool "stop" signal when a
near object fills the center region of the frame. Publishes on the topic
``so101_safety``'s ``trajectory_safety_gate`` / ``safety_pause_bridge`` listen
to, so any close object (e.g. a hand in front of the camera) halts the
follower arm.

This node does NOT run any model inference itself -- it is a pure topic
consumer (no ONNX Runtime / OpenCV dependency), by design: the model already
ran once upstream (in the perception node that publishes the depth topic), so
this keeps so101_safety lightweight and reusable with any depth source that
honours the contract below.

Subscribes:
  <depth_image_topic>  sensor_msgs/Image   encoding=32FC1  normalised [0,1]
                                            depth, higher = closer (the
                                            contract published by
                                            depth_anything_node's
                                            output_depth_topic)

Publishes:
  <stop_topic>          std_msgs/Bool       True = protective stop requested
  <debug_image_topic>   sensor_msgs/Image   (optional) mono8 depth + ROI marker

Parameters:
  depth_image_topic   (string) raw normalised depth topic (default /perception/depth)
  stop_topic          (string) Bool output (default /safety/protective_stop)
  debug_image_topic   (string) optional depth + ROI overlay (default /safety/depth_debug_image)
  roi                 (string) "x1,y1,x2,y2" normalised center ROI
  near_threshold      (float)  normalised depth [0,1] above which a pixel is near
  near_margin         (float)  margin added on top of the background reference
  min_area_ratio      (float)  fraction of ROI that must be near to trigger
  frames_to_block     (int)    consecutive near frames to assert stop
  frames_to_clear     (int)    consecutive clear frames to release stop
  monitor_hz          (float)  processing rate (throttle, decoupled from the
                                depth publisher's own rate)
  publish_debug_image (bool)   publish the overlay image
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from so101_safety.safety_monitor_common import (
    draw_roi_border,
    HysteresisDebouncer,
    parse_roi,
    roi_pixel_bounds,
)
from std_msgs.msg import Bool


class DepthSafetyMonitor(Node):

    def __init__(self):
        super().__init__("depth_safety_monitor")

        self.declare_parameter("depth_image_topic", "/perception/depth")
        self.declare_parameter("stop_topic", "/safety/protective_stop")
        self.declare_parameter("debug_image_topic", "/safety/depth_debug_image")
        self.declare_parameter("roi", "0.25,0.2,0.75,0.85")
        self.declare_parameter("near_threshold", 0.6)
        self.declare_parameter("near_margin", 0.15)
        self.declare_parameter("min_area_ratio", 0.12)
        self.declare_parameter("frames_to_block", 2)
        self.declare_parameter("frames_to_clear", 3)
        self.declare_parameter("monitor_hz", 10.0)
        self.declare_parameter("publish_debug_image", False)

        in_topic = self.get_parameter("depth_image_topic").value
        self._stop_topic = self.get_parameter("stop_topic").value
        self._roi = parse_roi(self.get_parameter("roi").value)
        self._near = float(self.get_parameter("near_threshold").value)
        self._margin = float(self.get_parameter("near_margin").value)
        self._min_area = float(self.get_parameter("min_area_ratio").value)
        n_block = int(self.get_parameter("frames_to_block").value)
        n_clear = int(self.get_parameter("frames_to_clear").value)
        hz = float(self.get_parameter("monitor_hz").value)
        self._publish_debug = bool(self.get_parameter("publish_debug_image").value)

        self._latest = None
        self._last_bg = 0.0
        self._debouncer = HysteresisDebouncer(n_block, n_clear)

        self._stop_pub = self.create_publisher(Bool, self._stop_topic, 10)
        self._dbg_pub = None
        if self._publish_debug:
            self._dbg_pub = self.create_publisher(
                Image, self.get_parameter("debug_image_topic").value, 5
            )

        self.create_subscription(Image, in_topic, self._depth_cb, qos_profile_sensor_data)
        self.create_timer(1.0 / max(hz, 0.1), self._run)
        self.get_logger().info(
            f"DepthSafetyMonitor ready — '{in_topic}' -> '{self._stop_topic}' "
            f"(near>{self._near}, area>{self._min_area})"
        )

    # ── depth image conversion ───────────────────────────────────────────────
    @staticmethod
    def _to_depth_array(msg: Image) -> np.ndarray:
        """Decode a 32FC1 sensor_msgs/Image into an HxW float32 array."""
        if msg.encoding.lower() != "32fc1":
            raise ValueError(
                f"Unsupported depth encoding: {msg.encoding} (expected 32FC1)"
            )
        buf = np.frombuffer(bytes(msg.data), dtype=np.float32)
        row_stride = msg.step // 4
        depth = buf.reshape(msg.height, row_stride)[:, : msg.width]
        return np.ascontiguousarray(depth)

    def _depth_cb(self, msg: Image):
        self._latest = msg

    # ── proximity logic ──────────────────────────────────────────────────────
    def _run(self):
        if self._latest is None:
            return
        try:
            depth = self._to_depth_array(self._latest)
        except ValueError as err:
            self.get_logger().warn(str(err), throttle_duration_sec=5.0)
            return

        h, w = depth.shape
        ix1, iy1, ix2, iy2 = roi_pixel_bounds(self._roi, w, h)
        roi = depth[iy1:iy2, ix1:ix2]

        # Background reference: median depth of the border region OUTSIDE the
        # ROI. Depth Anything gives relative (inverse) depth normalised per
        # frame, so an absolute threshold is meaningless. Instead we trigger
        # only when the center is clearly NEARER than the surrounding scene,
        # which is what happens when a hand/object approaches the camera.
        bg_mask = np.ones((h, w), dtype=bool)
        bg_mask[iy1:iy2, ix1:ix2] = False
        background = float(np.median(depth[bg_mask])) if bg_mask.any() else 0.0
        near_level = max(background + self._margin, self._near)
        area_ratio = float(np.mean(roi > near_level)) if roi.size else 0.0
        self._last_bg = background
        stop_state = self._debouncer.update(area_ratio >= self._min_area)

        self._stop_pub.publish(Bool(data=stop_state))
        if self._dbg_pub is not None:
            self._publish_debug_image(depth, area_ratio)

    def _publish_debug_image(self, depth: np.ndarray, area_ratio: float):
        # Plain-numpy mono8 overlay (no OpenCV dependency): grayscale depth
        # with the ROI boundary drawn as a bright/dark border, so this stays a
        # lightweight, dependency-free node.
        u8 = (np.clip(depth, 0.0, 1.0) * 255).astype(np.uint8)
        h, w = u8.shape
        marker = 255 if self._debouncer.state else 0
        draw_roi_border(u8, self._roi, marker)

        msg = Image()
        msg.header = self._latest.header
        msg.height, msg.width = h, w
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = w
        msg.data = u8.tobytes()
        self._dbg_pub.publish(msg)
        self.get_logger().debug(
            f"debug overlay: area_ratio={area_ratio:.2f} bg={self._last_bg:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DepthSafetyMonitor()
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
