"""
Person-safety monitor — perception-driven protective-stop trigger (detection).

Consumes already-computed object detections (published by e.g. the
``yolodetect`` snap / ``so101_yolo_demo``'s ``yolo_detect_node``, or any other
node honouring the same ``vision_msgs/Detection2DArray`` contract) and raises
a Bool "stop" signal when a detection of a watched class (person, by default)
overlaps the center region of the frame. Publishes on the SAME
``/safety/protective_stop`` topic as ``depth_safety_monitor``, so
``trajectory_safety_gate`` / ``safety_pause_bridge`` never need to change --
only the perception method driving the stop differs.

This is the "swap the model, different trigger logic" sibling of
``depth_safety_monitor``: instead of "is something near the camera" (depth
proximity), it asks "is a *person* specifically in the workspace ROI"
(detection + class filtering). Both share the ROI-parsing and
hysteresis/debounce plumbing in ``safety_monitor_common``.

This node does NOT run any model inference itself -- it is a pure topic
consumer (no ONNX Runtime / OpenCV dependency): the model already ran once
upstream (in the perception node that publishes the detections topic).

Subscribes:
  <detections_topic>  vision_msgs/Detection2DArray  (boxes in image-pixel space)

Publishes:
  <stop_topic>          std_msgs/Bool       True = protective stop requested
  <debug_image_topic>   sensor_msgs/Image   (optional) mono8 synthetic ROI
                                             gauge -- NOT the camera image
                                             (this node has no access to the
                                             raw frame, only detections). The
                                             ROI rectangle is filled
                                             proportionally to the best
                                             watched-class overlap ratio seen
                                             this frame (0 = empty/dark, 255 =
                                             fully overlapping), and its
                                             border brightens further once the
                                             stop actually triggers -- so you
                                             can see the signal rising even
                                             before it's enough to trigger.
                                             For the actual annotated camera
                                             view with real bounding boxes,
                                             use yolo_detect_node's
                                             output_image_topic instead
                                             (/camera/detections/visualization
                                             by default).

Parameters:
  detections_topic     (string) Detection2DArray topic (default /perception/detections)
  stop_topic            (string) Bool output (default /safety/protective_stop)
  debug_image_topic     (string) optional ROI overlay (default /safety/detection_debug_image)
  image_width            (int)    frame width used to interpret normalised ROI
                                   against the detections' pixel coordinates
                                   (default 640, must match the perception
                                   node's input frame size)
  image_height           (int)    frame height (default 480)
  roi                    (string) "x1,y1,x2,y2" normalised center ROI
  watched_classes        (string) comma-separated class names/ids to trigger on
                                   (default "person")
  conf_threshold         (float)  minimum detection score to consider
  min_overlap_ratio      (float)  minimum fraction of a watched detection's
                                   box that must fall inside the ROI to count
  frames_to_block        (int)    consecutive triggered frames to assert stop
  frames_to_clear        (int)    consecutive clear frames to release stop
  monitor_hz             (float)  processing rate (throttle, decoupled from
                                   the detections publisher's own rate)
  publish_debug_image    (bool)   publish the overlay image
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
from vision_msgs.msg import Detection2DArray


class PersonSafetyMonitor(Node):

    def __init__(self):
        super().__init__("person_safety_monitor")

        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("stop_topic", "/safety/protective_stop")
        self.declare_parameter("debug_image_topic", "/safety/detection_debug_image")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("roi", "0.25,0.2,0.75,0.85")
        self.declare_parameter("watched_classes", "person")
        self.declare_parameter("conf_threshold", 0.4)
        self.declare_parameter("min_overlap_ratio", 0.2)
        self.declare_parameter("frames_to_block", 2)
        self.declare_parameter("frames_to_clear", 3)
        self.declare_parameter("monitor_hz", 10.0)
        self.declare_parameter("publish_debug_image", False)

        in_topic = self.get_parameter("detections_topic").value
        self._stop_topic = self.get_parameter("stop_topic").value
        self._img_w = int(self.get_parameter("image_width").value)
        self._img_h = int(self.get_parameter("image_height").value)
        self._roi = parse_roi(self.get_parameter("roi").value)
        self._watched = self._parse_watched_classes(
            self.get_parameter("watched_classes").value
        )
        self._conf_threshold = float(self.get_parameter("conf_threshold").value)
        self._min_overlap = float(self.get_parameter("min_overlap_ratio").value)
        n_block = int(self.get_parameter("frames_to_block").value)
        n_clear = int(self.get_parameter("frames_to_clear").value)
        hz = float(self.get_parameter("monitor_hz").value)
        self._publish_debug = bool(self.get_parameter("publish_debug_image").value)

        self._latest = None
        self._last_overlap_ratio = 0.0
        self._debouncer = HysteresisDebouncer(n_block, n_clear)

        self._stop_pub = self.create_publisher(Bool, self._stop_topic, 10)
        self._dbg_pub = None
        if self._publish_debug:
            self._dbg_pub = self.create_publisher(
                Image, self.get_parameter("debug_image_topic").value, 5
            )

        self.create_subscription(
            Detection2DArray, in_topic, self._detections_cb, qos_profile_sensor_data
        )
        self.create_timer(1.0 / max(hz, 0.1), self._run)
        self.get_logger().info(
            f"PersonSafetyMonitor ready — '{in_topic}' -> '{self._stop_topic}' "
            f"(watching={sorted(self._watched)}, conf>{self._conf_threshold}, "
            f"overlap>{self._min_overlap})"
        )

    # ── parameter parsing ────────────────────────────────────────────────────
    @staticmethod
    def _parse_watched_classes(value):
        text = str(value).strip()
        if not text:
            return set()
        return {token.strip().lower() for token in text.split(",") if token.strip()}

    def _detections_cb(self, msg: Detection2DArray):
        self._latest = msg

    # ── presence logic ───────────────────────────────────────────────────────
    def _run(self):
        if self._latest is None:
            return

        ix1, iy1, ix2, iy2 = roi_pixel_bounds(self._roi, self._img_w, self._img_h)

        triggered = False
        best_overlap = 0.0
        for det in self._latest.detections:
            if not det.results:
                continue
            top = max(det.results, key=lambda r: r.hypothesis.score)
            class_id = str(top.hypothesis.class_id).strip().lower()
            score = float(top.hypothesis.score)
            if self._watched and class_id not in self._watched:
                continue
            if score < self._conf_threshold:
                continue

            bbox = det.bbox
            bx1 = bbox.center.position.x - bbox.size_x / 2.0
            bx2 = bbox.center.position.x + bbox.size_x / 2.0
            by1 = bbox.center.position.y - bbox.size_y / 2.0
            by2 = bbox.center.position.y + bbox.size_y / 2.0

            ox1, oy1 = max(bx1, ix1), max(by1, iy1)
            ox2, oy2 = min(bx2, ix2), min(by2, iy2)
            inter_w, inter_h = max(ox2 - ox1, 0.0), max(oy2 - oy1, 0.0)
            inter_area = inter_w * inter_h
            box_area = max(bx2 - bx1, 0.0) * max(by2 - by1, 0.0)
            if box_area <= 0.0:
                continue

            overlap_ratio = inter_area / box_area
            best_overlap = max(best_overlap, overlap_ratio)
            if overlap_ratio >= self._min_overlap:
                triggered = True

        self._last_overlap_ratio = best_overlap
        stop_state = self._debouncer.update(triggered)

        self._stop_pub.publish(Bool(data=stop_state))
        if self._dbg_pub is not None:
            self._publish_debug_image()

    def _publish_debug_image(self):
        # Plain-numpy mono8 canvas (no OpenCV dependency, and no access to the
        # raw camera frame -- this node only consumes detections, not images).
        # Background is a dark grey (not pure black) so the ROI border is
        # ALWAYS visible, even when nothing is triggered -- a pure-black
        # canvas with a marker value of 0 when untriggered was
        # indistinguishable from "no signal at all" (looks all-black either
        # way). The ROI interior is filled proportionally to the best
        # overlap ratio seen this frame, so you can see the signal rising
        # as a person approaches the ROI, even before it's enough to trigger
        # the stop -- for the real annotated camera view (actual bounding
        # boxes), see yolo_detect_node's output_image_topic instead
        # (/camera/detections/visualization by default).
        u8 = np.full((self._img_h, self._img_w), 30, dtype=np.uint8)
        ix1, iy1, ix2, iy2 = roi_pixel_bounds(self._roi, self._img_w, self._img_h)
        fill = int(np.clip(self._last_overlap_ratio, 0.0, 1.0) * 255)
        u8[iy1:iy2, ix1:ix2] = fill
        marker = 255 if self._debouncer.state else 160
        draw_roi_border(u8, self._roi, marker, thickness=3)

        msg = Image()
        msg.header = self._latest.header
        msg.height, msg.width = self._img_h, self._img_w
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = self._img_w
        msg.data = u8.tobytes()
        self._dbg_pub.publish(msg)
        self.get_logger().debug(f"debug overlay: overlap_ratio={self._last_overlap_ratio:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = PersonSafetyMonitor()
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
