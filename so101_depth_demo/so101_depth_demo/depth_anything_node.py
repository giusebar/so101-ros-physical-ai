"""
Depth Anything V2 (Small) inference node — CPU / ONNX Runtime
=============================================================
Subscribes to a ROS image topic, runs monocular relative-depth inference with
ONNX Runtime, and republishes both a colorised depth visualisation (for
humans) and a raw normalised depth map (for downstream consumers, e.g.
``so101_safety``'s ``depth_safety_monitor``).

This is the non-NVIDIA counterpart to the snap-twin TensorRT demo. It does NOT
import ``tensorrt`` or ``pycuda`` and does NOT open a camera device directly —
the camera is provided by the existing ``so101_bringup`` camera stack (or any
other publisher / bag / simulated camera).

Subscribes:
  <input_image_topic>   sensor_msgs/Image   (rgb8 | bgr8 | mono8)

Publishes:
  <output_image_topic>  sensor_msgs/Image   encoding=rgb8   (INFERNO colormap, for viewing)
  <output_depth_topic>  sensor_msgs/Image   encoding=32FC1  (normalised [0,1] depth,
                                                             higher = closer; for
                                                             machine consumers)

Parameters:
  model_path          (string) path to the Depth Anything V2 Small .onnx file
  input_image_topic   (string) camera image topic to subscribe to
  output_image_topic  (string) colorised depth visualisation topic to publish
  output_depth_topic  (string) raw normalised depth topic to publish (32FC1)
  model_input_size    (int)    square model input size, multiple of 14 (default 308)
  publish_width       (int)    output width  (default 518)
  publish_height      (int)    output height (default 518)
  min_period_s        (float)  minimum seconds between inferences (CPU throttle)
  intra_op_threads    (int)    ONNX Runtime intra-op thread count (0 = default)
"""

import array
import os
import time

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# ImageNet normalisation constants (float32, broadcast-ready over HWC)
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class DepthAnythingNode(Node):

    def __init__(self):
        super().__init__("depth_anything_node")

        # ── Parameters ──────────────────────────────────────────────────────
        default_model = os.path.expanduser("~/models/depth_anything_v2_small.onnx")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("input_image_topic", "/follower/image_raw")
        self.declare_parameter("output_image_topic", "/camera/depth/visualization")
        self.declare_parameter("output_depth_topic", "/perception/depth")
        # Must be a multiple of 14 (ViT patch size). 518=14x37 is the model's
        # native size; 308=14x22 runs ~2.8x faster on CPU with minor accuracy
        # loss (fine for relative-depth visualisation / proximity safety).
        self.declare_parameter("model_input_size", 308)
        self.declare_parameter("publish_width", 518)
        self.declare_parameter("publish_height", 518)
        self.declare_parameter("min_period_s", 0.0)
        self.declare_parameter("intra_op_threads", 0)

        model_path = self.get_parameter("model_path").value
        in_topic = self.get_parameter("input_image_topic").value
        out_topic = self.get_parameter("output_image_topic").value
        depth_out_topic = self.get_parameter("output_depth_topic").value
        self._in_size = int(self.get_parameter("model_input_size").value)
        self._pub_w = int(self.get_parameter("publish_width").value)
        self._pub_h = int(self.get_parameter("publish_height").value)
        self._min_period = float(self.get_parameter("min_period_s").value)
        intra_threads = int(self.get_parameter("intra_op_threads").value)

        # ── ONNX Runtime session (CPU) ──────────────────────────────────────
        self.get_logger().info(f"Loading ONNX model: {model_path}")
        sess_opts = ort.SessionOptions()
        if intra_threads > 0:
            sess_opts.intra_op_num_threads = intra_threads
        # CPUExecutionProvider is always available; this stays NVIDIA-free.
        self._session = ort.InferenceSession(
            model_path,
            sess_options=sess_opts,
            providers=["CPUExecutionProvider"],
        )
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        self.get_logger().info(
            f"ONNX session ready — providers={self._session.get_providers()} "
            f"input='{self._input_name}' output='{self._output_name}'"
        )

        # ── Pub / Sub ───────────────────────────────────────────────────────
        self._pub = self.create_publisher(Image, out_topic, 5)
        self._depth_pub = self.create_publisher(Image, depth_out_topic, 5)
        self._sub = self.create_subscription(Image, in_topic, self._image_cb, 5)

        self._frame_count = 0
        self._last_infer_t = 0.0
        self.get_logger().info(
            f"DepthAnythingNode ready — subscribing '{in_topic}', "
            f"publishing viz '{out_topic}' ({self._pub_w}x{self._pub_h}), "
            f"publishing raw depth '{depth_out_topic}' (32FC1)"
        )

    # ────────────────────────────────────────────────────────────────────────
    # ROS Image -> BGR numpy
    # ────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _image_to_bgr(msg: Image) -> np.ndarray:
        """Convert a sensor_msgs/Image into an HxWx3 BGR uint8 array."""
        enc = msg.encoding.lower()
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)

        if enc in ("rgb8", "bgr8"):
            img = buf.reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
            if enc == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return np.ascontiguousarray(img)
        if enc in ("mono8", "8uc1"):
            gray = buf.reshape(msg.height, msg.step)[:, : msg.width]
            return cv2.cvtColor(np.ascontiguousarray(gray), cv2.COLOR_GRAY2BGR)
        if enc in ("rgba8", "bgra8"):
            img = buf.reshape(msg.height, msg.step // 4, 4)[:, : msg.width, :3]
            if enc == "rgba8":
                img = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2BGR)
            else:
                img = np.ascontiguousarray(img)
            return img
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    # ────────────────────────────────────────────────────────────────────────
    # Preprocessing / inference
    # ────────────────────────────────────────────────────────────────────────

    def _preprocess(self, bgr_frame: np.ndarray) -> np.ndarray:
        """BGR frame -> NCHW float32 tensor (1, 3, S, S) ImageNet-normalised."""
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(
            rgb, (self._in_size, self._in_size), interpolation=cv2.INTER_LINEAR
        )
        x = resized.astype(np.float32) / 255.0
        x = (x - _MEAN) / _STD
        x = np.ascontiguousarray(x.transpose(2, 0, 1))[None, ...]  # NCHW
        return x

    def _infer(self, nchw_input: np.ndarray) -> np.ndarray:
        """Run one forward pass; return a 2-D (S, S) relative depth map."""
        out = self._session.run([self._output_name], {self._input_name: nchw_input})[0]
        depth = np.asarray(out, dtype=np.float32)
        return np.squeeze(depth)  # (S, S)

    # ────────────────────────────────────────────────────────────────────────
    # Subscription callback
    # ────────────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        # CPU throttle: skip frames so a slow machine doesn't build up a backlog.
        if self._min_period > 0.0:
            now = time.monotonic()
            if now - self._last_infer_t < self._min_period:
                return
            self._last_infer_t = now

        try:
            bgr = self._image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        depth = self._infer(self._preprocess(bgr))

        # Normalise to [0, 1] (higher = closer, disparity-like).
        d_min, d_max = float(depth.min()), float(depth.max())
        if d_max - d_min < 1e-6:
            depth_norm = np.zeros_like(depth)
        else:
            depth_norm = (depth - d_min) / (d_max - d_min)

        if self._pub_w != depth_norm.shape[1] or self._pub_h != depth_norm.shape[0]:
            depth_out = cv2.resize(
                depth_norm, (self._pub_w, self._pub_h), interpolation=cv2.INTER_LINEAR
            )
        else:
            depth_out = depth_norm

        stamp = msg.header.stamp if msg.header.stamp.sec else self.get_clock().now().to_msg()
        frame_id = msg.header.frame_id or "camera"

        # ── Raw depth (machine consumers, e.g. so101_safety) ────────────────
        depth_f32 = np.ascontiguousarray(depth_out, dtype=np.float32)
        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = frame_id
        depth_msg.height = self._pub_h
        depth_msg.width = self._pub_w
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = False
        depth_msg.step = self._pub_w * 4
        depth_msg.data = array.array("B", depth_f32.tobytes())
        self._depth_pub.publish(depth_msg)

        # ── Colorised visualisation (humans, rqt_image_view, ...) ───────────
        depth_u8 = (depth_out * 255).astype(np.uint8)
        color_bgr = cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)
        rgb_color = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

        vis = Image()
        vis.header.stamp = stamp
        vis.header.frame_id = frame_id
        vis.height = self._pub_h
        vis.width = self._pub_w
        vis.encoding = "rgb8"
        vis.is_bigendian = False
        vis.step = self._pub_w * 3
        vis.data = array.array("B", rgb_color.tobytes())
        self._pub.publish(vis)

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            self.get_logger().info(f"Depth inference running — frame {self._frame_count}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingNode()
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
