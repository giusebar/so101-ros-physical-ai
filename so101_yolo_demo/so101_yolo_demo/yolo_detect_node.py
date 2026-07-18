"""
YOLOv8n (nano) person/object detection inference node — CPU / ONNX Runtime
============================================================================
Subscribes to a ROS image topic, runs YOLOv8n object detection with ONNX
Runtime, and republishes both an annotated visualisation image (for humans)
and machine-readable detections (for downstream consumers, e.g.
``so101_safety``'s ``person_safety_monitor``).

This is the "swap the model, do something different" sibling of
``so101_depth_demo``'s ``depth_anything_node``: same CPU/ONNX Runtime
inference pattern and the same camera input, but recognises specific object
classes instead of estimating depth. NMS is baked into the exported ONNX
graph (``model.export(format="onnx", nms=True, ...)``), so this node only
needs to parse the final boxes -- no manual NMS/decoding.

Subscribes:
  <input_image_topic>        sensor_msgs/Image           (rgb8 | bgr8 | mono8 | rgba8 | bgra8)

Publishes:
  <output_image_topic>       sensor_msgs/Image           encoding=rgb8   (boxes + labels drawn, for viewing)
  <output_detections_topic>  vision_msgs/Detection2DArray                (for machine consumers)

Parameters:
  model_path              (string) path to the yolov8n .onnx file
  input_image_topic       (string) camera image topic to subscribe to
  output_image_topic      (string) annotated visualisation topic to publish
  output_detections_topic (string) Detection2DArray topic to publish
  input_size              (int)    square model input size (default 640)
  conf_threshold           (float)  minimum detection score to keep (default 0.4)
  class_filter             (string) comma-separated COCO class ids or names to
                                    keep (default "person"); empty = keep all
  min_period_s             (float)  minimum seconds between inferences (CPU throttle)
  intra_op_threads         (int)    ONNX Runtime intra-op thread count (0 = default)
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
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    Pose2D,
)

# Standard 80-class COCO label set, in training order (class id == index).
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]

# Deterministic per-class BGR colour for box drawing (fixed palette, no randomness).
_PALETTE = [
    (56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255),
    (49, 210, 207), (10, 249, 72), (23, 204, 146), (134, 219, 61),
    (52, 147, 26), (187, 212, 0), (168, 153, 44), (255, 194, 0),
    (147, 69, 52), (255, 115, 100), (236, 24, 0), (255, 56, 132),
    (133, 0, 82), (255, 56, 203), (200, 149, 255), (199, 55, 255),
]


class YoloDetectNode(Node):

    def __init__(self):
        super().__init__("yolo_detect_node")

        # ── Parameters ──────────────────────────────────────────────────────
        default_model = os.path.expanduser("~/models/yolov8n.onnx")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("input_image_topic", "/static_camera/image_raw")
        self.declare_parameter("output_image_topic", "/camera/detections/visualization")
        self.declare_parameter("output_detections_topic", "/perception/detections")
        self.declare_parameter("input_size", 640)
        self.declare_parameter("conf_threshold", 0.4)
        self.declare_parameter("class_filter", "person")
        self.declare_parameter("min_period_s", 0.0)
        self.declare_parameter("intra_op_threads", 0)

        model_path = self.get_parameter("model_path").value
        in_topic = self.get_parameter("input_image_topic").value
        out_topic = self.get_parameter("output_image_topic").value
        det_out_topic = self.get_parameter("output_detections_topic").value
        self._in_size = int(self.get_parameter("input_size").value)
        self._conf_threshold = float(self.get_parameter("conf_threshold").value)
        self._class_filter = self._parse_class_filter(
            self.get_parameter("class_filter").value
        )
        self._min_period = float(self.get_parameter("min_period_s").value)
        intra_threads = int(self.get_parameter("intra_op_threads").value)

        # ── ONNX Runtime session (CPU) ──────────────────────────────────────
        self.get_logger().info(f"Loading ONNX model: {model_path}")
        sess_opts = ort.SessionOptions()
        if intra_threads > 0:
            sess_opts.intra_op_num_threads = intra_threads
        # CPUExecutionProvider is always available; this stays NVIDIA-free,
        # matching so101_depth_demo's depth_anything_node.
        self._session = ort.InferenceSession(
            model_path,
            sess_options=sess_opts,
            providers=["CPUExecutionProvider"],
        )
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        self.get_logger().info(
            f"ONNX session ready — providers={self._session.get_providers()} "
            f"input='{self._input_name}' output='{self._output_name}' "
            f"class_filter={sorted(self._class_filter) if self._class_filter else 'ALL'}"
        )

        # ── Pub / Sub ───────────────────────────────────────────────────────
        self._pub = self.create_publisher(Image, out_topic, 5)
        self._det_pub = self.create_publisher(Detection2DArray, det_out_topic, 5)
        self._sub = self.create_subscription(Image, in_topic, self._image_cb, 5)

        self._frame_count = 0
        self._last_infer_t = 0.0
        self.get_logger().info(
            f"YoloDetectNode ready — subscribing '{in_topic}', "
            f"publishing viz '{out_topic}', "
            f"publishing detections '{det_out_topic}'"
        )

    # ────────────────────────────────────────────────────────────────────────
    # Parameter parsing
    # ────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _parse_class_filter(value):
        """Parse class_filter param: "" -> keep all classes (returns None).

        Otherwise a set of class ids, accepting either COCO class names
        ("person") or integer ids ("0")."""
        if value is None:
            return None
        text = str(value).strip()
        if not text:
            return None
        ids = set()
        for token in text.split(","):
            token = token.strip()
            if not token:
                continue
            if token.isdigit():
                ids.add(int(token))
            else:
                try:
                    ids.add(COCO_CLASSES.index(token.lower()))
                except ValueError:
                    raise ValueError(f"Unknown COCO class name in class_filter: '{token}'")
        return ids

    # ────────────────────────────────────────────────────────────────────────
    # ROS Image -> BGR numpy (same approach as depth_anything_node)
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
        """BGR frame -> NCHW float32 tensor (1, 3, S, S), [0,1] normalised.

        Uses a plain (non-letterboxed) resize to keep this symmetric with
        depth_anything_node's preprocessing -- trades a little accuracy on
        non-square frames for simplicity. Box coordinates are mapped back to
        the original frame size in _postprocess via independent x/y scales.
        """
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(
            rgb, (self._in_size, self._in_size), interpolation=cv2.INTER_LINEAR
        )
        x = resized.astype(np.float32) / 255.0
        x = np.ascontiguousarray(x.transpose(2, 0, 1))[None, ...]  # NCHW
        return x

    def _infer(self, nchw_input: np.ndarray) -> np.ndarray:
        """Run one forward pass; return the raw (N, 6) [x1,y1,x2,y2,conf,cls]
        detections in model-input-pixel space (NMS already applied in-graph)."""
        out = self._session.run([self._output_name], {self._input_name: nchw_input})[0]
        return np.asarray(out, dtype=np.float32).reshape(-1, 6)

    def _postprocess(self, raw, orig_w: int, orig_h: int):
        """Filter by confidence/class and rescale boxes to the original frame."""
        sx = orig_w / float(self._in_size)
        sy = orig_h / float(self._in_size)
        detections = []
        for x1, y1, x2, y2, conf, cls in raw:
            if conf < self._conf_threshold:
                continue
            cls_id = int(round(cls))
            if self._class_filter is not None and cls_id not in self._class_filter:
                continue
            detections.append(
                (x1 * sx, y1 * sy, x2 * sx, y2 * sy, float(conf), cls_id)
            )
        return detections

    # ────────────────────────────────────────────────────────────────────────
    # Publishing helpers
    # ────────────────────────────────────────────────────────────────────────

    def _publish_detections(self, detections, stamp, frame_id):
        msg = Detection2DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        for x1, y1, x2, y2, conf, cls_id in detections:
            det = Detection2D()
            det.header.stamp = stamp
            det.header.frame_id = frame_id
            label = COCO_CLASSES[cls_id] if 0 <= cls_id < len(COCO_CLASSES) else str(cls_id)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = conf
            det.results.append(hyp)

            bbox = BoundingBox2D()
            bbox.center = Pose2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = max(x2 - x1, 0.0)
            bbox.size_y = max(y2 - y1, 0.0)
            det.bbox = bbox
            det.id = label

            msg.detections.append(det)
        self._det_pub.publish(msg)

    def _draw_and_publish_viz(self, bgr_frame, detections, stamp, frame_id):
        canvas = bgr_frame.copy()
        for x1, y1, x2, y2, conf, cls_id in detections:
            color = _PALETTE[cls_id % len(_PALETTE)]
            label = COCO_CLASSES[cls_id] if 0 <= cls_id < len(COCO_CLASSES) else str(cls_id)
            p1 = (int(x1), int(y1))
            p2 = (int(x2), int(y2))
            cv2.rectangle(canvas, p1, p2, color, 2)
            text = f"{label} {conf:.2f}"
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(
                canvas, (p1[0], max(p1[1] - th - 4, 0)), (p1[0] + tw + 2, p1[1]), color, -1
            )
            cv2.putText(
                canvas, text, (p1[0] + 1, max(p1[1] - 3, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
            )

        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        vis = Image()
        vis.header.stamp = stamp
        vis.header.frame_id = frame_id
        vis.height = h
        vis.width = w
        vis.encoding = "rgb8"
        vis.is_bigendian = False
        vis.step = w * 3
        vis.data = array.array("B", np.ascontiguousarray(rgb).tobytes())
        self._pub.publish(vis)

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

        h, w = bgr.shape[:2]
        raw = self._infer(self._preprocess(bgr))
        detections = self._postprocess(raw, w, h)

        stamp = msg.header.stamp if msg.header.stamp.sec else self.get_clock().now().to_msg()
        frame_id = msg.header.frame_id or "camera"

        self._publish_detections(detections, stamp, frame_id)
        self._draw_and_publish_viz(bgr, detections, stamp, frame_id)

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"YOLO inference running — frame {self._frame_count}, "
                f"{len(detections)} detection(s) this frame"
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
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
