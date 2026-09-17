"""
OWL-ViT open-vocabulary detection node — Qualcomm Hexagon NPU / QNN
===================================================================
Subscribes to a ROS image topic, runs Qualcomm AI-Hub's OWL-ViT
(open-vocabulary object detector, ViT-B/32 CLIP backbone) with ONNX Runtime's
QNN execution provider on the Hexagon NPU, and republishes an annotated
visualisation image (for humans, with the safety ROI overlaid), machine-
readable detections, and a Bool protective-stop signal computed directly from
the (prompt-matched) detections.

Unlike the fixed-class ``so101_yolo_demo``/``yolo_detect_node`` sibling, OWL-ViT
detects whatever free-form TEXT PROMPT you give it (default ``"a hand"``): the
prompt is a live runtime input (tokenised CLIP text) fed to the model alongside
the image, so it can be swapped at runtime (via a ROS topic or ``snap set
prompt=...``) without reloading the model. This is the Qualcomm/NPU counterpart
of the Jetson-only NanoOWL variant — same camera input, same
``vision_msgs/Detection2DArray`` + ``/safety/protective_stop`` outputs, same
ROI-overlap + hysteresis trigger, different accelerator (QNN HTP instead of
TensorRT/torch2trt).

Safety-trigger logic (ROI overlap + hysteresis debounce) lives directly in this
node rather than a separate ``so101_safety`` monitor: this is the
"ai-vision-ros2" single-snap architecture, where swapping the AI backend via
``snap refresh --channel=...`` must swap the ENTIRE behaviour (including what
counts as a protective stop), not just the raw perception output. The
enforcement side (``safety_pause_bridge`` / ``trajectory_safety_gate``) is
unchanged and still just subscribes to the shared ``stop_topic`` contract.

The bundled Qualcomm AI-Hub OWL-ViT ONNX is w8a16-quantised with static shapes
(image 1x3x768x768 uint16, text 1x16 int32) and runs fully on the Hexagon NPU.
The model outputs 576 candidate boxes (uint16, pixel space) + sigmoid scores
(uint16) which this node dequantises, score-thresholds and NMSes — NMS is NOT
baked into the graph.

Subscribes:
  <input_image_topic>        sensor_msgs/Image           (rgb8 | bgr8 | mono8 | rgba8 | bgra8)
                             OR, when use_compressed=True (default),
                             <input_image_topic>/compressed  sensor_msgs/CompressedImage (JPEG/PNG)
  <prompt_topic>             std_msgs/String             (optional live prompt update)

Publishes:
  <output_image_topic>       sensor_msgs/Image           encoding=rgb8 (boxes + ROI overlay)
  <output_detections_topic>  vision_msgs/Detection2DArray
  <stop_topic>                std_msgs/Bool               True = protective stop requested

Parameters:
  model_path              (string) path to the OWL-ViT .onnx file
  input_image_topic       (string) camera image topic to subscribe to
  output_image_topic      (string) annotated visualisation topic to publish
  output_detections_topic (string) Detection2DArray topic to publish
  input_size              (int)    square model input size (default 768)
  prompt                  (string) initial text prompt (default "a hand")
  prompt_topic            (string) String topic for live prompt updates
  prompt_file             (string) file polled for live prompt updates ("" = off)
  score_threshold         (float)  minimum sigmoid score to keep (default 0.1)
  iou_threshold           (float)  NMS IoU threshold (default 0.3)
  max_detections          (int)    max detections kept after NMS (default 20)
  tokenizer_dir           (string) local CLIP tokenizer dir (vocab/merges)
  max_text_len            (int)    tokenised text length the model expects (16)
  min_period_s            (float)  minimum seconds between inferences (throttle)
  intra_op_threads        (int)    ONNX Runtime intra-op thread count (0 = default)
  execution_providers     (string list) ORT providers, in priority order
  qnn_backend_path        (string) QNN accelerator: "htp" (default), "gpu", "cpu"
  qnn_context_cache_path  (string) EPContext cache file ("" = disabled). When set
                                   and a QNN backend is used, the compiled QNN
                                   context binary is written here on the first run
                                   and reloaded on subsequent runs, turning the
                                   ~40 s graph compile into a ~1 s load. Must be a
                                   writable path (the snap points it at
                                   $SNAP_COMMON). Stale/incompatible caches (e.g.
                                   after a QNN version change) are auto-regenerated.
  log_severity_level      (int)    ORT log severity (0=Verbose .. 4=Fatal)
  input_quant_scale       (float)  pixel input quant scale (uint16); 0 = feed float
  input_quant_zero_point  (int)    pixel input quant zero-point
  box_dequant_scale       (float)  box output dequant scale; 0 = output already float
  box_dequant_zero_point  (int)    box output dequant zero-point
  score_dequant_scale     (float)  score output dequant scale; 0 = output already float
  score_dequant_zero_point(int)    score output dequant zero-point
  stop_topic              (string) Bool protective-stop output
  roi                     (string) "x1,y1,x2,y2" normalised center ROI
  min_overlap_ratio       (float)  min fraction of a detection box inside the ROI
  frames_to_block         (int)    consecutive triggered frames to assert stop
  frames_to_clear         (int)    consecutive clear frames to release stop
"""

import array
import os
import time
from collections import deque

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    Pose2D,
)

# CLIP / OWL-ViT image normalisation constants (from the OwlViT processor
# preprocessor_config.json). RGB, applied after rescaling to [0,1].
_OWL_MEAN = np.array([0.48145466, 0.4578275, 0.40821073], dtype=np.float32)
_OWL_STD = np.array([0.26862954, 0.26130258, 0.27577711], dtype=np.float32)


def _parse_roi(value):
    """Parse a normalised "x1,y1,x2,y2" ROI string (or 4-item sequence)."""
    vals = (
        [float(v) for v in value.split(",")]
        if isinstance(value, str)
        else [float(v) for v in value]
    )
    if len(vals) != 4 or not all(0.0 <= v <= 1.0 for v in vals):
        raise ValueError("roi must be 4 normalised values x1,y1,x2,y2 in [0,1]")
    return vals


def _roi_pixel_bounds(roi, width, height):
    x1, y1, x2, y2 = roi
    return int(x1 * width), int(y1 * height), int(x2 * width), int(y2 * height)


class _HysteresisDebouncer:
    """Debounces a per-frame boolean with separate assert/release windows,
    to avoid flickering the safety-stop state on single noisy frames. Local
    copy (not imported from so101_safety): each ai-vision-ros2 perception
    package is built in isolation by its own snapcraft.yaml, scoped to just
    that one ROS package, so it can't depend on a sibling package."""

    def __init__(self, frames_to_block: int, frames_to_clear: int):
        self._n_block = int(frames_to_block)
        self._n_clear = int(frames_to_clear)
        self._hist = deque(maxlen=max(self._n_block, self._n_clear))
        self._state = False

    def update(self, is_triggered: bool) -> bool:
        self._hist.append(bool(is_triggered))
        block = list(self._hist)[-self._n_block:]
        clear = list(self._hist)[-self._n_clear:]
        if len(block) == self._n_block and all(block):
            self._state = True
        elif len(clear) == self._n_clear and not any(clear):
            self._state = False
        return self._state


class OwlDetectNode(Node):

    def __init__(self):
        super().__init__("owl_detect_node")

        # ── Parameters ──────────────────────────────────────────────────────
        default_model = os.path.expanduser("~/models/owl_vit/owl_vit.onnx")
        default_tok = os.path.expanduser("~/models/owl_vit/tokenizer")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("input_image_topic", "/static_camera/image_raw")
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("output_image_topic", "/camera/detections/visualization")
        self.declare_parameter("output_detections_topic", "/perception/detections")
        self.declare_parameter("input_size", 768)
        self.declare_parameter("prompt", "a hand")
        self.declare_parameter("prompt_topic", "/perception/prompt")
        self.declare_parameter("prompt_file", "")
        self.declare_parameter("score_threshold", 0.1)
        self.declare_parameter("iou_threshold", 0.3)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("tokenizer_dir", default_tok)
        self.declare_parameter("max_text_len", 16)
        self.declare_parameter("min_period_s", 0.0)
        self.declare_parameter("intra_op_threads", 4)
        self.declare_parameter(
            "execution_providers",
            ["QNNExecutionProvider", "CPUExecutionProvider"],
        )
        self.declare_parameter("qnn_backend_path", "htp")
        self.declare_parameter("qnn_context_cache_path", "")
        self.declare_parameter("log_severity_level", 2)
        # OWL-ViT AI-Hub w8a16 quantisation params (from the model metadata.json).
        self.declare_parameter("input_quant_scale", 0.00006009246135363355)
        self.declare_parameter("input_quant_zero_point", 29825)
        self.declare_parameter("box_dequant_scale", 0.019158801063895226)
        self.declare_parameter("box_dequant_zero_point", 5775)
        self.declare_parameter("score_dequant_scale", 0.000015199416338873561)
        self.declare_parameter("score_dequant_zero_point", 0)
        self.declare_parameter("stop_topic", "/safety/protective_stop")
        self.declare_parameter("roi", "0.25,0.2,0.75,0.85")
        self.declare_parameter("min_overlap_ratio", 0.2)
        self.declare_parameter("frames_to_block", 2)
        self.declare_parameter("frames_to_clear", 3)

        model_path = self.get_parameter("model_path").value
        in_topic = self.get_parameter("input_image_topic").value
        self._use_compressed = bool(self.get_parameter("use_compressed").value)
        out_topic = self.get_parameter("output_image_topic").value
        det_out_topic = self.get_parameter("output_detections_topic").value
        self._in_size = int(self.get_parameter("input_size").value)
        prompt_topic = self.get_parameter("prompt_topic").value
        self._prompt_file = str(self.get_parameter("prompt_file").value)
        self._score_threshold = float(self.get_parameter("score_threshold").value)
        self._iou_threshold = float(self.get_parameter("iou_threshold").value)
        self._max_det = int(self.get_parameter("max_detections").value)
        tokenizer_dir = str(self.get_parameter("tokenizer_dir").value)
        self._max_text_len = int(self.get_parameter("max_text_len").value)
        self._min_period = float(self.get_parameter("min_period_s").value)
        intra_threads = int(self.get_parameter("intra_op_threads").value)
        execution_providers = list(self.get_parameter("execution_providers").value)
        qnn_backend = str(self.get_parameter("qnn_backend_path").value)
        self._qnn_ctx_cache = str(self.get_parameter("qnn_context_cache_path").value)
        self._in_quant_scale = float(self.get_parameter("input_quant_scale").value)
        self._in_quant_zp = int(self.get_parameter("input_quant_zero_point").value)
        self._box_scale = float(self.get_parameter("box_dequant_scale").value)
        self._box_zp = int(self.get_parameter("box_dequant_zero_point").value)
        self._score_scale = float(self.get_parameter("score_dequant_scale").value)
        self._score_zp = int(self.get_parameter("score_dequant_zero_point").value)
        stop_topic = self.get_parameter("stop_topic").value
        self._roi = _parse_roi(self.get_parameter("roi").value)
        self._min_overlap = float(self.get_parameter("min_overlap_ratio").value)
        n_block = int(self.get_parameter("frames_to_block").value)
        n_clear = int(self.get_parameter("frames_to_clear").value)
        self._debouncer = _HysteresisDebouncer(n_block, n_clear)

        # ── CLIP text tokenizer (transformers, tokenizer only — no torch) ────
        self.get_logger().info(f"Loading CLIP tokenizer: {tokenizer_dir}")
        from transformers import AutoTokenizer

        self._tokenizer = AutoTokenizer.from_pretrained(tokenizer_dir)

        # ── ONNX Runtime session ────────────────────────────────────────────
        self.get_logger().info(f"Loading ONNX model: {model_path}")
        sess_opts = ort.SessionOptions()
        if intra_threads > 0:
            sess_opts.intra_op_num_threads = intra_threads
        sess_opts.log_severity_level = int(self.get_parameter("log_severity_level").value)

        if "QNNExecutionProvider" in execution_providers:
            self._session = self._create_qnn_session(model_path, sess_opts, qnn_backend)
        else:
            self.get_logger().info(f"ORT providers (requested): {execution_providers}")
            self._session = ort.InferenceSession(
                model_path, sess_options=sess_opts, providers=execution_providers
            )

        # Map the three named inputs / three named outputs (order is not
        # guaranteed, so resolve by name).
        in_names = {i.name: i for i in self._session.get_inputs()}
        self._pix_name = self._pick_name(in_names, ("pixel_values", "image"))
        self._ids_name = self._pick_name(in_names, ("input_ids",))
        self._attn_name = self._pick_name(in_names, ("attention_mask",))
        out_names = [o.name for o in self._session.get_outputs()]
        self._box_name = self._pick_seq(out_names, ("boxes", "pred_boxes"))
        self._score_name = self._pick_seq(out_names, ("scores", "logits", "pred_scores"))
        self._out_names = [self._box_name, self._score_name]

        # numpy dtype ORT expects for the (possibly quantised) pixel input.
        _pix_type = in_names[self._pix_name].type  # e.g. 'tensor(uint16)'
        self._pix_dtype = {
            "tensor(float)": np.float32,
            "tensor(float16)": np.float16,
            "tensor(uint8)": np.uint8,
            "tensor(int8)": np.int8,
            "tensor(uint16)": np.uint16,
            "tensor(int16)": np.int16,
            "tensor(int32)": np.int32,
        }.get(_pix_type, np.float32)

        self.get_logger().info(
            f"ONNX session ready — providers(active)={self._session.get_providers()} "
            f"pixel='{self._pix_name}' ({_pix_type}) ids='{self._ids_name}' "
            f"boxes='{self._box_name}' scores='{self._score_name}' "
            f"in_quant={'on' if self._in_quant_scale > 0 else 'off'}"
        )

        # ── Prompt (tokenised text) ─────────────────────────────────────────
        self._input_ids = None
        self._attention_mask = None
        self._label = ""
        self._apply_prompt(str(self.get_parameter("prompt").value))
        self._prompt_file_mtime = 0.0

        # ── Pub / Sub ───────────────────────────────────────────────────────
        self._pub = self.create_publisher(Image, out_topic, 5)
        self._det_pub = self.create_publisher(Detection2DArray, det_out_topic, 5)
        self._stop_pub = self.create_publisher(Bool, stop_topic, 10)
        if self._use_compressed:
            comp_topic = in_topic.rstrip("/") + "/compressed"
            self._sub = self.create_subscription(
                CompressedImage, comp_topic, self._compressed_cb, 5
            )
            self._sub_topic = comp_topic
        else:
            self._sub = self.create_subscription(Image, in_topic, self._image_cb, 5)
            self._sub_topic = in_topic
        self._prompt_sub = self.create_subscription(
            String, prompt_topic, self._prompt_cb, 5
        )
        if self._prompt_file:
            self.create_timer(0.5, self._poll_prompt_file)

        self._frame_count = 0
        self._last_infer_t = 0.0
        self.get_logger().info(
            f"OwlDetectNode ready — subscribing '{self._sub_topic}'"
            + (" (compressed)" if self._use_compressed else "")
            + ", "
            f"prompt='{self._label}', "
            f"publishing viz '{out_topic}', detections '{det_out_topic}', "
            f"protective stop '{stop_topic}' (overlap>{self._min_overlap})"
        )

    # ────────────────────────────────────────────────────────────────────────
    # Name resolution helpers
    # ────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _pick_name(names, candidates):
        for c in candidates:
            if c in names:
                return c
        raise RuntimeError(f"model input not found; wanted one of {candidates}, "
                           f"have {list(names)}")

    @staticmethod
    def _pick_seq(names, candidates):
        for c in candidates:
            if c in names:
                return c
        raise RuntimeError(f"model output not found; wanted one of {candidates}, "
                           f"have {names}")

    # ────────────────────────────────────────────────────────────────────────
    # QNN plugin-EP session (Qualcomm variant) — identical pattern to
    # so101_depth_demo's depth_anything_node.
    # ────────────────────────────────────────────────────────────────────────

    def _create_qnn_session(self, model_path, sess_opts, backend):
        """Build an InferenceSession that offloads to the Qualcomm accelerator
        via ONNX Runtime's QNN plugin execution provider.

        ``backend`` selects the accelerator: ``htp`` (Hexagon NPU — fastest,
        the default), ``gpu`` (Adreno) or ``cpu`` (QNN reference). Unsupported
        nodes automatically fall back to ORT's built-in CPU EP.

        When ``qnn_context_cache_path`` is set (and a htp/gpu backend is used),
        the compiled QNN context binary is cached to that path via the ORT
        EPContext mechanism: the first run compiles + writes it (~40 s), later
        runs reload it (~1 s). A cache that fails to load (e.g. after a QNN
        version change) is deleted and regenerated.
        """
        import onnxruntime_qnn as ort_qnn

        backend_type = {
            "htp": "htp", "npu": "htp", "libqnnhtp.so": "htp",
            "gpu": "gpu", "adreno": "gpu", "libqnngpu.so": "gpu",
            "cpu": "cpu", "libqnncpu.so": "cpu",
        }.get(str(backend).lower(), str(backend).lower())

        try:
            ort.register_execution_provider_library(
                "QNNExecutionProvider", ort_qnn.get_library_path()
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"QNN plugin EP register: {exc}")

        qnn_devices = [
            d for d in ort.get_ep_devices() if d.ep_name == "QNNExecutionProvider"
        ]
        if not qnn_devices:
            raise RuntimeError(
                "QNNExecutionProvider registered but no QNN OrtEpDevice found"
            )

        options = {"backend_type": backend_type}
        if backend_type == "htp":
            options["htp_performance_mode"] = "burst"
        sess_opts.add_provider_for_devices(qnn_devices, options)

        self.get_logger().info(
            f"QNN plugin EP: backend_type='{backend_type}' "
            f"lib='{ort_qnn.get_library_path()}' "
            f"ADSP_LIBRARY_PATH='{os.environ.get('ADSP_LIBRARY_PATH', '<unset>')}'"
        )

        # EPContext (QNN context-binary) caching. QNN compiles the graph to an
        # HTP context binary on the first session creation (~40 s for OWL-ViT);
        # persisting it and reloading it later cuts that to ~1 s. Only meaningful
        # for the accelerated backends (htp/gpu), not the QNN reference CPU.
        cache = self._qnn_ctx_cache
        if cache and backend_type in ("htp", "gpu"):
            if os.path.exists(cache):
                try:
                    self.get_logger().info(f"QNN: loading cached context '{cache}'")
                    return ort.InferenceSession(cache, sess_options=sess_opts)
                except Exception as exc:  # noqa: BLE001
                    # Stale/incompatible cache (e.g. a QNN/QAIRT version change):
                    # drop it and fall through to regenerate.
                    self.get_logger().warn(
                        f"QNN: cached context failed to load ({exc}); regenerating"
                    )
                    try:
                        os.remove(cache)
                    except OSError:
                        pass
            cache_dir = os.path.dirname(cache)
            if cache_dir:
                os.makedirs(cache_dir, exist_ok=True)
            sess_opts.add_session_config_entry("ep.context_enable", "1")
            sess_opts.add_session_config_entry("ep.context_file_path", cache)
            sess_opts.add_session_config_entry("ep.context_embed_mode", "1")
            self.get_logger().info(
                f"QNN: compiling + caching context to '{cache}' "
                f"(first run — expect ~40 s)"
            )
            return ort.InferenceSession(model_path, sess_options=sess_opts)

        # No context caching: the QNN device was added to sess_opts, and the
        # built-in CPU EP is added automatically for any unsupported nodes.
        return ort.InferenceSession(model_path, sess_options=sess_opts)

    # ────────────────────────────────────────────────────────────────────────
    # Prompt handling (live-updatable)
    # ────────────────────────────────────────────────────────────────────────

    def _apply_prompt(self, text: str):
        """Tokenise a new text prompt and cache its input_ids/attention_mask.

        Cheap (~ms): only re-runs the CLIP tokenizer, never reloads the model.
        """
        text = (text or "").strip()
        if not text:
            self.get_logger().warn("Ignoring empty prompt")
            return
        enc = self._tokenizer(
            [text],
            padding="max_length",
            max_length=self._max_text_len,
            truncation=True,
            return_tensors="np",
        )
        self._input_ids = enc["input_ids"].astype(np.int32).reshape(1, self._max_text_len)
        self._attention_mask = (
            enc["attention_mask"].astype(np.int32).reshape(1, self._max_text_len)
        )
        self._label = text
        self.get_logger().info(f"Prompt set to '{text}'")

    def _prompt_cb(self, msg: String):
        self._apply_prompt(msg.data)

    def _poll_prompt_file(self):
        try:
            mtime = os.path.getmtime(self._prompt_file)
        except OSError:
            return
        if mtime <= self._prompt_file_mtime:
            return
        self._prompt_file_mtime = mtime
        try:
            with open(self._prompt_file, "r") as fh:
                text = fh.readline().strip()
        except OSError:
            return
        if text and text != self._label:
            self._apply_prompt(text)

    # ────────────────────────────────────────────────────────────────────────
    # ROS Image -> BGR numpy (same approach as depth/yolo nodes)
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
    # Preprocessing / inference / postprocessing
    # ────────────────────────────────────────────────────────────────────────

    def _preprocess(self, bgr_frame: np.ndarray) -> np.ndarray:
        """BGR frame -> NCHW model input tensor (1, 3, S, S).

        Replicates the OwlViT image processor: RGB, aspect-distorting squash
        resize to SxS (bicubic), rescale to [0,1], CLIP mean/std normalise.
        Quantises to the model's integer input dtype when configured (the
        AI-Hub w8a16 model takes a uint16 image).
        """
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(
            rgb, (self._in_size, self._in_size), interpolation=cv2.INTER_CUBIC
        )
        x = resized.astype(np.float32) / 255.0
        x = (x - _OWL_MEAN) / _OWL_STD
        x = np.ascontiguousarray(x.transpose(2, 0, 1))[None, ...]  # NCHW

        if self._in_quant_scale > 0.0:
            info = np.iinfo(self._pix_dtype)
            q = np.round(x / self._in_quant_scale) + self._in_quant_zp
            x = np.clip(q, info.min, info.max).astype(self._pix_dtype)
        return x

    def _infer(self, pixel_values: np.ndarray):
        """Run one forward pass; return (boxes[576,4], scores[576]) dequantised.

        Boxes are in model-input (SxS) pixel space, xyxy. Scores are sigmoid
        probabilities in [0,1].
        """
        feed = {
            self._pix_name: pixel_values,
            self._ids_name: self._input_ids,
            self._attn_name: self._attention_mask,
        }
        boxes_q, scores_q = self._session.run(self._out_names, feed)
        boxes = boxes_q.astype(np.float32).reshape(-1, 4)
        scores = scores_q.astype(np.float32).reshape(-1)
        if self._box_scale > 0.0:
            boxes = (boxes - self._box_zp) * self._box_scale
        if self._score_scale > 0.0:
            scores = (scores - self._score_zp) * self._score_scale
        return boxes, scores

    def _postprocess(self, boxes, scores, orig_w: int, orig_h: int):
        """Score-threshold, rescale to the original frame, and NMS."""
        keep = scores >= self._score_threshold
        boxes = boxes[keep]
        scores = scores[keep]
        if boxes.shape[0] == 0:
            return []

        sx = orig_w / float(self._in_size)
        sy = orig_h / float(self._in_size)
        boxes = boxes * np.array([sx, sy, sx, sy], dtype=np.float32)
        boxes[:, 0::2] = np.clip(boxes[:, 0::2], 0.0, orig_w)
        boxes[:, 1::2] = np.clip(boxes[:, 1::2], 0.0, orig_h)

        detections = []
        for i in self._nms(boxes, scores, self._iou_threshold, self._max_det):
            x1, y1, x2, y2 = boxes[i]
            detections.append((float(x1), float(y1), float(x2), float(y2),
                               float(scores[i]), 0))
        return detections

    @staticmethod
    def _nms(boxes, scores, iou_thr, max_det):
        """Greedy IoU non-maximum suppression (OWL-ViT has no in-graph NMS)."""
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = np.maximum(x2 - x1, 0.0) * np.maximum(y2 - y1, 0.0)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0 and len(keep) < max_det:
            i = int(order[0])
            keep.append(i)
            if order.size == 1:
                break
            rest = order[1:]
            xx1 = np.maximum(x1[i], x1[rest])
            yy1 = np.maximum(y1[i], y1[rest])
            xx2 = np.minimum(x2[i], x2[rest])
            yy2 = np.minimum(y2[i], y2[rest])
            inter = np.maximum(xx2 - xx1, 0.0) * np.maximum(yy2 - yy1, 0.0)
            iou = inter / (areas[i] + areas[rest] - inter + 1e-9)
            order = rest[iou < iou_thr]
        return keep

    # ────────────────────────────────────────────────────────────────────────
    # Safety-trigger logic (ROI overlap + hysteresis debounce)
    # ────────────────────────────────────────────────────────────────────────

    def _evaluate_stop(self, detections, frame_w: int, frame_h: int):
        """Given this frame's (prompt-matched) detections, return
        (stop_state, best_overlap_ratio) using the ACTUAL camera dimensions."""
        ix1, iy1, ix2, iy2 = _roi_pixel_bounds(self._roi, frame_w, frame_h)
        best_overlap = 0.0
        triggered = False
        for x1, y1, x2, y2, _conf, _cls_id in detections:
            ox1, oy1 = max(x1, ix1), max(y1, iy1)
            ox2, oy2 = min(x2, ix2), min(y2, iy2)
            inter_w, inter_h = max(ox2 - ox1, 0.0), max(oy2 - oy1, 0.0)
            inter_area = inter_w * inter_h
            box_area = max(x2 - x1, 0.0) * max(y2 - y1, 0.0)
            if box_area <= 0.0:
                continue
            overlap_ratio = inter_area / box_area
            best_overlap = max(best_overlap, overlap_ratio)
            if overlap_ratio >= self._min_overlap:
                triggered = True

        stop_state = self._debouncer.update(triggered)
        return stop_state, best_overlap

    def _draw_roi_overlay(self, canvas: np.ndarray, stop_state: bool):
        """Draw the ROI rectangle (green = clear, red = stop) onto the viz."""
        h, w = canvas.shape[:2]
        ix1, iy1, ix2, iy2 = _roi_pixel_bounds(self._roi, w, h)
        color = (0, 0, 255) if stop_state else (0, 200, 0)  # BGR
        cv2.rectangle(canvas, (ix1, iy1), (ix2, iy2), color, 3)
        label = "PROTECTIVE STOP" if stop_state else "clear"
        cv2.putText(
            canvas, label, (ix1 + 4, max(iy1 - 8, 12)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
        )

    # ────────────────────────────────────────────────────────────────────────
    # Publishing helpers
    # ────────────────────────────────────────────────────────────────────────

    def _publish_detections(self, detections, stamp, frame_id):
        msg = Detection2DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        for x1, y1, x2, y2, conf, _cls_id in detections:
            det = Detection2D()
            det.header.stamp = stamp
            det.header.frame_id = frame_id

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self._label
            hyp.hypothesis.score = conf
            det.results.append(hyp)

            bbox = BoundingBox2D()
            bbox.center = Pose2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = max(x2 - x1, 0.0)
            bbox.size_y = max(y2 - y1, 0.0)
            det.bbox = bbox
            det.id = self._label

            msg.detections.append(det)
        self._det_pub.publish(msg)

    def _draw_and_publish_viz(self, bgr_frame, detections, stop_state, stamp, frame_id):
        canvas = bgr_frame.copy()
        color = (255, 128, 0)  # BGR
        for x1, y1, x2, y2, conf, _cls_id in detections:
            p1 = (int(x1), int(y1))
            p2 = (int(x2), int(y2))
            cv2.rectangle(canvas, p1, p2, color, 2)
            text = f"{self._label} {conf:.2f}"
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(
                canvas, (p1[0], max(p1[1] - th - 4, 0)), (p1[0] + tw + 2, p1[1]), color, -1
            )
            cv2.putText(
                canvas, text, (p1[0] + 1, max(p1[1] - 3, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
            )

        self._draw_roi_overlay(canvas, stop_state)

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

    def _throttled(self) -> bool:
        if self._min_period > 0.0:
            now = time.monotonic()
            if now - self._last_infer_t < self._min_period:
                return True
            self._last_infer_t = now
        return False

    def _image_cb(self, msg: Image):
        if self._throttled():
            return
        try:
            bgr = self._image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        stamp = msg.header.stamp if msg.header.stamp.sec else self.get_clock().now().to_msg()
        frame_id = msg.header.frame_id or "camera"
        self._process(bgr, stamp, frame_id)

    def _compressed_cb(self, msg: CompressedImage):
        if self._throttled():
            return
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn(
                "Failed to decode compressed image", throttle_duration_sec=5.0
            )
            return
        stamp = msg.header.stamp if msg.header.stamp.sec else self.get_clock().now().to_msg()
        frame_id = msg.header.frame_id or "camera"
        self._process(bgr, stamp, frame_id)

    def _process(self, bgr: np.ndarray, stamp, frame_id: str):
        h, w = bgr.shape[:2]
        boxes, scores = self._infer(self._preprocess(bgr))
        detections = self._postprocess(boxes, scores, w, h)

        stop_state, best_overlap = self._evaluate_stop(detections, w, h)
        self._stop_pub.publish(Bool(data=stop_state))

        self._publish_detections(detections, stamp, frame_id)
        self._draw_and_publish_viz(bgr, detections, stop_state, stamp, frame_id)

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"OWL inference running — frame {self._frame_count}, "
                f"prompt='{self._label}', {len(detections)} detection(s), "
                f"overlap={best_overlap:.2f}, stop={stop_state}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OwlDetectNode()
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
