"""
Depth-proximity safety trigger — CPU / ONNX Runtime.

Runs Depth Anything V2 (Small) on a camera image and raises a Bool "stop"
signal when a near object fills the center region of the frame. Publishes on
the same topic the so101_teleop trajectory_safety_gate listens to, so any close
object (e.g. a hand in front of the camera) halts the simulated follower arm.

Subscribes:
  <input_image_topic>   sensor_msgs/Image   (rgb8 | bgr8 | mono8)

Publishes:
    <stop_topic>          std_msgs/Bool       True = protective stop requested
  <debug_image_topic>   sensor_msgs/Image   (optional) rgb8 depth + ROI overlay

Parameters:
  model_path          (string) Depth Anything V2 Small .onnx file
  input_image_topic   (string) camera image topic
    stop_topic          (string) Bool output (default /safety/protective_stop)
  debug_image_topic   (string) optional colorised depth + ROI overlay
  model_input_size    (int)    square model input, multiple of 14 (default 308)
  roi                 (string) "x1,y1,x2,y2" normalised center ROI
  near_threshold      (float)  normalised depth [0,1] above which a pixel is near
  min_area_ratio      (float)  fraction of ROI that must be near to trigger
  frames_to_block     (int)    consecutive near frames to assert stop
  frames_to_clear     (int)    consecutive clear frames to release stop
  inference_hz        (float)  detection rate (CPU throttle)
  publish_debug_image (bool)   publish the overlay image
  intra_op_threads    (int)    ONNX Runtime intra-op threads (0 = default)
"""

import os
from collections import deque

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class DepthProximityNode(Node):

    def __init__(self):
        super().__init__("depth_proximity_node")

        default_model = os.path.expanduser("~/models/depth_anything_v2_small.onnx")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("input_image_topic", "/static_camera/image_raw")
        self.declare_parameter("stop_topic", "/safety/protective_stop")
        self.declare_parameter("debug_image_topic", "/safety/depth_debug_image")
        # Must be a multiple of 14 (ViT patch size). 308=14x22 runs ~2.8x
        # faster on CPU than the native 518=14x37, ample for proximity safety.
        self.declare_parameter("model_input_size", 308)
        self.declare_parameter("roi", "0.25,0.2,0.75,0.85")
        self.declare_parameter("near_threshold", 0.6)
        self.declare_parameter("near_margin", 0.15)
        self.declare_parameter("min_area_ratio", 0.12)
        self.declare_parameter("frames_to_block", 2)
        self.declare_parameter("frames_to_clear", 3)
        self.declare_parameter("inference_hz", 10.0)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("intra_op_threads", 0)

        model_path = self.get_parameter("model_path").value
        in_topic = self.get_parameter("input_image_topic").value
        self._stop_topic = self.get_parameter("stop_topic").value
        self._in_size = int(self.get_parameter("model_input_size").value)
        self._roi = self._parse_roi(self.get_parameter("roi").value)
        self._near = float(self.get_parameter("near_threshold").value)
        self._margin = float(self.get_parameter("near_margin").value)
        self._min_area = float(self.get_parameter("min_area_ratio").value)
        self._n_block = int(self.get_parameter("frames_to_block").value)
        self._n_clear = int(self.get_parameter("frames_to_clear").value)
        hz = float(self.get_parameter("inference_hz").value)
        self._publish_debug = bool(self.get_parameter("publish_debug_image").value)
        intra = int(self.get_parameter("intra_op_threads").value)

        opts = ort.SessionOptions()
        if intra > 0:
            opts.intra_op_num_threads = intra
        self._session = ort.InferenceSession(
            model_path, sess_options=opts, providers=["CPUExecutionProvider"]
        )
        self._in_name = self._session.get_inputs()[0].name
        self._out_name = self._session.get_outputs()[0].name
        self.get_logger().info(
            f"ONNX session ready — providers={self._session.get_providers()}"
        )

        self._latest = None
        self._stop_state = False
        self._last_bg = 0.0
        self._hist = deque(maxlen=max(self._n_block, self._n_clear))

        self._stop_pub = self.create_publisher(Bool, self._stop_topic, 10)
        self._dbg_pub = None
        if self._publish_debug:
            self._dbg_pub = self.create_publisher(
                Image, self.get_parameter("debug_image_topic").value, 5
            )

        self.create_subscription(Image, in_topic, self._image_cb, qos_profile_sensor_data)
        self.create_timer(1.0 / max(hz, 0.1), self._run)
        self.get_logger().info(
            f"DepthProximityNode ready — '{in_topic}' -> '{self._stop_topic}' "
            f"(near>{self._near}, area>{self._min_area})"
        )

    # ── image conversion ────────────────────────────────────────────────────
    @staticmethod
    def _to_bgr(msg: Image) -> np.ndarray:
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
        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    def _image_cb(self, msg: Image):
        self._latest = msg

    # ── inference + proximity logic ──────────────────────────────────────────
    def _infer_norm_depth(self, bgr: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self._in_size, self._in_size))
        x = resized.astype(np.float32) / 255.0
        x = (x - _MEAN) / _STD
        x = np.ascontiguousarray(x.transpose(2, 0, 1))[None, ...]
        out = self._session.run([self._out_name], {self._in_name: x})[0]
        depth = np.squeeze(np.asarray(out, dtype=np.float32))
        d_min, d_max = float(depth.min()), float(depth.max())
        if d_max - d_min < 1e-6:
            return np.zeros_like(depth)
        return (depth - d_min) / (d_max - d_min)

    def _run(self):
        if self._latest is None:
            return
        try:
            bgr = self._to_bgr(self._latest)
            depth = self._infer_norm_depth(bgr)
        except Exception as err:  # noqa: BLE001
            self.get_logger().warn(
                f"Depth proximity failed: {err}", throttle_duration_sec=5.0
            )
            return

        h, w = depth.shape
        x1, y1, x2, y2 = self._roi
        ix1, iy1 = int(x1 * w), int(y1 * h)
        ix2, iy2 = int(x2 * w), int(y2 * h)
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
        self._hist.append(area_ratio >= self._min_area)
        self._update_state()

        self._stop_pub.publish(Bool(data=self._stop_state))
        if self._dbg_pub is not None:
            self._publish_debug_image(depth, area_ratio)

    def _update_state(self):
        block = list(self._hist)[-self._n_block:]
        clear = list(self._hist)[-self._n_clear:]
        if len(block) == self._n_block and all(block):
            self._stop_state = True
        elif len(clear) == self._n_clear and not any(clear):
            self._stop_state = False

    def _publish_debug_image(self, depth: np.ndarray, area_ratio: float):
        u8 = (depth * 255).astype(np.uint8)
        vis = cv2.applyColorMap(u8, cv2.COLORMAP_INFERNO)
        h, w = vis.shape[:2]
        x1, y1, x2, y2 = self._roi
        p1 = (int(x1 * w), int(y1 * h))
        p2 = (int(x2 * w), int(y2 * h))
        color = (0, 0, 255) if self._stop_state else (0, 255, 0)
        cv2.rectangle(vis, p1, p2, color, 2)
        label = (
            f"{'STOP' if self._stop_state else 'clear'}  "
            f"near={area_ratio:.2f}  bg={getattr(self, '_last_bg', 0.0):.2f}"
        )
        cv2.putText(vis, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header = self._latest.header
        msg.height, msg.width = h, w
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = rgb.tobytes()
        self._dbg_pub.publish(msg)

    @staticmethod
    def _parse_roi(value):
        vals = (
            [float(v) for v in value.split(",")]
            if isinstance(value, str)
            else [float(v) for v in value]
        )
        if len(vals) != 4 or not all(0.0 <= v <= 1.0 for v in vals):
            raise ValueError("roi must be 4 normalised values x1,y1,x2,y2 in [0,1]")
        return vals


def main(args=None):
    rclpy.init(args=args)
    node = DepthProximityNode()
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
