"""
Depth Anything V2 (Small) inference node — CPU / ONNX Runtime
=============================================================
Subscribes to a ROS image topic, runs monocular relative-depth inference with
ONNX Runtime, and republishes a colorised depth visualisation (for humans,
with the safety ROI overlaid), a raw normalised depth map (for other machine
consumers), and a Bool protective-stop signal computed directly from the
depth map.

This node is hardware-agnostic about the ONNX Runtime execution provider: by
default it uses ``CPUExecutionProvider`` (the non-NVIDIA counterpart to the
snap-twin TensorRT demo), but the ``execution_providers`` /
``engine_cache_dir`` parameters let the NVIDIA ``ai-vision-ros2`` snap variant
run the exact same node on a Jetson GPU via ``TensorrtExecutionProvider`` /
``CUDAExecutionProvider`` (ORT builds/caches the TensorRT engine on first run).
It does NOT import ``tensorrt`` or ``pycuda`` directly and does NOT open a
camera device directly — the camera is provided by the existing
``so101_bringup`` camera stack (or any other publisher / bag / simulated
camera).

Safety-trigger logic (ROI proximity + hysteresis debounce) lives directly in
this node rather than a separate ``so101_safety`` monitor: this is the
"ai-vision-ros2" single-snap architecture, where swapping the AI backend via
``snap refresh --channel=...`` must swap the ENTIRE behaviour (including what
counts as a protective stop), not just the raw perception output. The
enforcement side (``safety_pause_bridge`` / ``trajectory_safety_gate``) is
unchanged and still just subscribes to the shared ``stop_topic`` contract.
(The sibling package ``so101_yolo_demo``'s ``yolo_detect_node`` follows the
identical pattern with its own detection-specific trigger logic.)

Subscribes:
  <input_image_topic>   sensor_msgs/Image   (rgb8 | bgr8 | mono8)

Publishes:
  <output_image_topic>  sensor_msgs/Image   encoding=rgb8   (INFERNO colormap
                                                             + ROI/trigger
                                                             overlay, for
                                                             viewing)
  <output_depth_topic>  sensor_msgs/Image   encoding=32FC1  (normalised [0,1]
                                                             depth, higher =
                                                             closer; for other
                                                             machine consumers)
  <stop_topic>          std_msgs/Bool       True = protective stop requested

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
  execution_providers (string list) ORT execution providers, in priority order
                               (default ["CPUExecutionProvider"]; NVIDIA snap
                               uses ["TensorrtExecutionProvider",
                               "CUDAExecutionProvider", "CPUExecutionProvider"])
  engine_cache_dir    (string) TensorRT engine cache dir ("" = ORT default;
                               NVIDIA snap points it at $SNAP_DATA so the
                               first-run engine build persists)
  qnn_backend_path    (string) QNN accelerator for QNNExecutionProvider:
                               "htp" (Hexagon NPU, default), "gpu" (Adreno) or
                               "cpu" (QNN reference); Qualcomm snap variant only
  stop_topic          (string) Bool protective-stop output (default /safety/protective_stop)
  roi                 (string) "x1,y1,x2,y2" normalised center ROI
  near_threshold      (float)  normalised depth [0,1] above which a pixel is near
  near_margin         (float)  margin added on top of the background reference
  min_area_ratio      (float)  fraction of ROI that must be near to trigger
  frames_to_block     (int)    consecutive near frames to assert stop
  frames_to_clear     (int)    consecutive clear frames to release stop
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
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# ImageNet normalisation constants (float32, broadcast-ready over HWC)
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


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
        # Hardware-agnostic ORT execution-provider selection. Default is pure
        # CPU (no NVIDIA); the NVIDIA ai-vision-ros2 snap variant overrides
        # these via its configure hook to run on the Jetson GPU with TensorRT.
        self.declare_parameter("execution_providers", ["CPUExecutionProvider"])
        self.declare_parameter("engine_cache_dir", "")
        # QNN (Qualcomm) accelerator selector for QNNExecutionProvider. One of
        # "htp" (Hexagon NPU — fastest, default), "gpu" (Adreno) or "cpu" (QNN
        # reference). Maps to the QNN "backend_type" option. Only used when
        # "QNNExecutionProvider" is requested (the Qualcomm ai-vision-ros2
        # variant). Kept named qnn_backend_path for config-key compatibility.
        self.declare_parameter("qnn_backend_path", "htp")
        # ONNX Runtime log severity: 0=Verbose 1=Info 2=Warning(default) 3=Error
        # 4=Fatal. Set to 0/1 to surface why an execution provider (e.g. QNN)
        # declines nodes or fails to initialise and falls back to CPU.
        self.declare_parameter("log_severity_level", 2)
        # ── Model I/O profile ───────────────────────────────────────────────
        # Different depth models need different pre/post-processing:
        #   * "imagenet": RGB /255 then ImageNet mean/std normalisation, feed
        #     float32 (the HuggingFace onnx-community fp32 model does the ViT
        #     normalisation OUTSIDE the graph).
        #   * "unit": RGB /255 only, no mean/std (the Qualcomm AI-Hub model bakes
        #     the ImageNet normalisation INTO the graph, so it wants a plain
        #     [0,1] image).
        self.declare_parameter("input_normalization", "imagenet")
        # Optional input quantisation: when input_quant_scale > 0 the preprocessed
        # [0,1]/normalised tensor is quantised to the model's integer input dtype
        # as q = round(real/scale) + zero_point (the AI-Hub w8a16 model takes a
        # uint16 input). 0 = feed float32 as-is.
        self.declare_parameter("input_quant_scale", 0.0)
        self.declare_parameter("input_quant_zero_point", 0)
        # Optional output dequantisation: when output_dequant_scale > 0 the model
        # output is dequantised as real = (q - zero_point) * scale (the AI-Hub
        # model returns uint16 depth). 0 = output is already float.
        self.declare_parameter("output_dequant_scale", 0.0)
        self.declare_parameter("output_dequant_zero_point", 0)
        self.declare_parameter("stop_topic", "/safety/protective_stop")
        self.declare_parameter("roi", "0.25,0.2,0.75,0.85")
        self.declare_parameter("near_threshold", 0.6)
        self.declare_parameter("near_margin", 0.15)
        self.declare_parameter("min_area_ratio", 0.12)
        self.declare_parameter("frames_to_block", 2)
        self.declare_parameter("frames_to_clear", 3)

        model_path = self.get_parameter("model_path").value
        in_topic = self.get_parameter("input_image_topic").value
        out_topic = self.get_parameter("output_image_topic").value
        depth_out_topic = self.get_parameter("output_depth_topic").value
        self._in_size = int(self.get_parameter("model_input_size").value)
        self._pub_w = int(self.get_parameter("publish_width").value)
        self._pub_h = int(self.get_parameter("publish_height").value)
        self._min_period = float(self.get_parameter("min_period_s").value)
        intra_threads = int(self.get_parameter("intra_op_threads").value)
        execution_providers = list(self.get_parameter("execution_providers").value)
        engine_cache_dir = str(self.get_parameter("engine_cache_dir").value)
        qnn_backend = str(self.get_parameter("qnn_backend_path").value)
        self._normalization = str(
            self.get_parameter("input_normalization").value
        ).lower()
        self._in_quant_scale = float(self.get_parameter("input_quant_scale").value)
        self._in_quant_zp = int(self.get_parameter("input_quant_zero_point").value)
        self._out_dequant_scale = float(
            self.get_parameter("output_dequant_scale").value
        )
        self._out_dequant_zp = int(
            self.get_parameter("output_dequant_zero_point").value
        )
        stop_topic = self.get_parameter("stop_topic").value
        self._roi = _parse_roi(self.get_parameter("roi").value)
        self._near = float(self.get_parameter("near_threshold").value)
        self._margin = float(self.get_parameter("near_margin").value)
        self._min_area = float(self.get_parameter("min_area_ratio").value)
        n_block = int(self.get_parameter("frames_to_block").value)
        n_clear = int(self.get_parameter("frames_to_clear").value)
        self._debouncer = _HysteresisDebouncer(n_block, n_clear)

        # ── ONNX Runtime session ────────────────────────────────────────────
        self.get_logger().info(f"Loading ONNX model: {model_path}")
        sess_opts = ort.SessionOptions()
        if intra_threads > 0:
            sess_opts.intra_op_num_threads = intra_threads
        log_severity = int(self.get_parameter("log_severity_level").value)
        sess_opts.log_severity_level = log_severity

        # QNN is a *plugin* execution provider (ORT >=1.24) and is selected
        # through a different API than the classic providers list: the
        # onnxruntime-qnn wheel ships the provider library plus the QNN backend
        # libs, and ORT only exposes QNNExecutionProvider once the plugin
        # library is registered AND a QNN OrtEpDevice is added to the session
        # options via add_provider_for_devices() (passing providers=[...] to
        # InferenceSession is silently ignored for plugin EPs). The specific
        # accelerator is chosen with the "backend_type" option (htp = Hexagon
        # NPU, gpu = Adreno, cpu = QNN reference), NOT a backend_path.
        if "QNNExecutionProvider" in execution_providers:
            self._session = self._create_qnn_session(
                model_path, sess_opts, qnn_backend
            )
        else:
            # Classic providers path (CPU / TensorRT / CUDA — e.g. the NVIDIA
            # snap variant). Provider-specific options: when a TensorRT engine
            # cache dir is given, persist the first-run engine build there so
            # subsequent daemon starts skip the ~1-2 min rebuild.
            provider_options = []
            for provider in execution_providers:
                if provider == "TensorrtExecutionProvider":
                    opts = {"trt_fp16_enable": True}
                    if engine_cache_dir:
                        os.makedirs(engine_cache_dir, exist_ok=True)
                        opts["trt_engine_cache_enable"] = True
                        opts["trt_engine_cache_path"] = engine_cache_dir
                    provider_options.append(opts)
                else:
                    provider_options.append({})
            self.get_logger().info(
                f"ORT providers (requested): {execution_providers} "
                f"engine_cache_dir='{engine_cache_dir or '<default>'}'"
            )
            self._session = ort.InferenceSession(
                model_path,
                sess_options=sess_opts,
                providers=execution_providers,
                provider_options=provider_options,
            )

        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        # numpy dtype ORT expects for the input tensor (e.g. float32, or uint16
        # for the AI-Hub quantised model). Used to cast the quantised input.
        _ort_type = self._session.get_inputs()[0].type  # e.g. 'tensor(uint16)'
        self._input_np_dtype = {
            "tensor(float)": np.float32,
            "tensor(float16)": np.float16,
            "tensor(uint8)": np.uint8,
            "tensor(int8)": np.int8,
            "tensor(uint16)": np.uint16,
            "tensor(int16)": np.int16,
            "tensor(int32)": np.int32,
        }.get(_ort_type, np.float32)
        self.get_logger().info(
            f"ONNX session ready — providers(active)={self._session.get_providers()} "
            f"input='{self._input_name}' ({_ort_type}) output='{self._output_name}' "
            f"norm='{self._normalization}' "
            f"in_quant={'on' if self._in_quant_scale > 0 else 'off'} "
            f"out_dequant={'on' if self._out_dequant_scale > 0 else 'off'}"
        )

        # ── Pub / Sub ───────────────────────────────────────────────────────
        self._pub = self.create_publisher(Image, out_topic, 5)
        self._depth_pub = self.create_publisher(Image, depth_out_topic, 5)
        self._stop_pub = self.create_publisher(Bool, stop_topic, 10)
        self._sub = self.create_subscription(Image, in_topic, self._image_cb, 5)

        self._frame_count = 0
        self._last_infer_t = 0.0
        self.get_logger().info(
            f"DepthAnythingNode ready — subscribing '{in_topic}', "
            f"publishing viz '{out_topic}' ({self._pub_w}x{self._pub_h}), "
            f"publishing raw depth '{depth_out_topic}' (32FC1), "
            f"publishing protective stop '{stop_topic}' "
            f"(near>{self._near}, area>{self._min_area})"
        )

    # ────────────────────────────────────────────────────────────────────────
    # QNN plugin-EP session (Qualcomm variant)
    # ────────────────────────────────────────────────────────────────────────

    def _create_qnn_session(self, model_path, sess_opts, backend):
        """Build an InferenceSession that offloads to the Qualcomm accelerator
        via ONNX Runtime's QNN plugin execution provider.

        ``backend`` selects the accelerator: ``htp`` (Hexagon NPU — fastest,
        the default), ``gpu`` (Adreno) or ``cpu`` (QNN reference). Unsupported
        nodes automatically fall back to ORT's built-in CPU EP.

        Notes:
          * The onnxruntime-qnn wheel version must match the on-device QAIRT
            SDK (the daemon pins the matching pair) or the QNN backend refuses
            the platform.
          * The model must have FIXED input shapes for QNN to finalise the
            graph (the snap fixes them at build time).
          * HTP additionally needs the fastrpc userspace lib (libcdsprpc.so,
            dlopen'd by libQnnHtp.so), the DSP skel on ADSP_LIBRARY_PATH, and
            access to /dev/fastrpc-cdsp — all provided by the snap.
        """
        import onnxruntime_qnn as ort_qnn

        # Normalise the backend selector to QNN's "backend_type" value.
        backend_type = {
            "htp": "htp", "npu": "htp", "libqnnhtp.so": "htp",
            "gpu": "gpu", "adreno": "gpu", "libqnngpu.so": "gpu",
            "cpu": "cpu", "libqnncpu.so": "cpu",
        }.get(str(backend).lower(), str(backend).lower())

        # Register the plugin EP library (idempotent-ish; ignore re-register).
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
        # No providers= here: the QNN device was added to sess_opts, and the
        # built-in CPU EP is added automatically for any unsupported nodes.
        return ort.InferenceSession(model_path, sess_options=sess_opts)

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
        """BGR frame -> NCHW model input tensor (1, 3, S, S).

        Normalisation depends on the model profile ("imagenet" applies ViT
        mean/std, "unit" just scales to [0,1] because the AI-Hub model bakes the
        normalisation into the graph). If input quantisation is configured, the
        result is quantised to the model's integer input dtype.
        """
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(
            rgb, (self._in_size, self._in_size), interpolation=cv2.INTER_LINEAR
        )
        x = resized.astype(np.float32) / 255.0
        if self._normalization == "imagenet":
            x = (x - _MEAN) / _STD
        x = np.ascontiguousarray(x.transpose(2, 0, 1))[None, ...]  # NCHW

        if self._in_quant_scale > 0.0:
            info = np.iinfo(self._input_np_dtype)
            q = np.round(x / self._in_quant_scale) + self._in_quant_zp
            x = np.clip(q, info.min, info.max).astype(self._input_np_dtype)
        return x

    def _infer(self, model_input: np.ndarray) -> np.ndarray:
        """Run one forward pass; return a 2-D (S, S) relative depth map.

        If output dequantisation is configured (AI-Hub quantised model), the raw
        integer output is dequantised back to a float depth first.
        """
        out = self._session.run([self._output_name], {self._input_name: model_input})[0]
        if self._out_dequant_scale > 0.0:
            depth = (out.astype(np.float32) - self._out_dequant_zp) * self._out_dequant_scale
        else:
            depth = np.asarray(out, dtype=np.float32)
        return np.squeeze(depth)  # (S, S)

    # ────────────────────────────────────────────────────────────────────────
    # Safety-trigger logic (ROI proximity + hysteresis debounce)
    # ────────────────────────────────────────────────────────────────────────

    def _evaluate_stop(self, depth_out: np.ndarray):
        """Given the normalised [0,1] depth map, return (stop_state, area_ratio)."""
        h, w = depth_out.shape
        ix1, iy1, ix2, iy2 = _roi_pixel_bounds(self._roi, w, h)
        roi_arr = depth_out[iy1:iy2, ix1:ix2]

        # Background reference: median depth of the border region OUTSIDE the
        # ROI. Depth Anything gives relative (inverse) depth normalised per
        # frame, so an absolute threshold is meaningless. Instead we trigger
        # only when the center is clearly NEARER than the surrounding scene,
        # which is what happens when a hand/object approaches the camera.
        bg_mask = np.ones((h, w), dtype=bool)
        bg_mask[iy1:iy2, ix1:ix2] = False
        background = float(np.median(depth_out[bg_mask])) if bg_mask.any() else 0.0
        near_level = max(background + self._margin, self._near)
        area_ratio = float(np.mean(roi_arr > near_level)) if roi_arr.size else 0.0

        stop_state = self._debouncer.update(area_ratio >= self._min_area)
        return stop_state, area_ratio

    def _draw_roi_overlay(self, bgr_frame: np.ndarray, stop_state: bool) -> np.ndarray:
        """Draw the ROI rectangle (green = clear, red = protective stop
        triggered) directly onto the colorised depth visualisation."""
        h, w = bgr_frame.shape[:2]
        ix1, iy1, ix2, iy2 = _roi_pixel_bounds(self._roi, w, h)
        color = (0, 0, 255) if stop_state else (0, 200, 0)  # BGR
        cv2.rectangle(bgr_frame, (ix1, iy1), (ix2, iy2), color, 3)
        label = "PROTECTIVE STOP" if stop_state else "clear"
        cv2.putText(
            bgr_frame, label, (ix1 + 4, max(iy1 - 8, 12)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
        )
        return bgr_frame

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

        # ── Safety-trigger evaluation + publish ──────────────────────────────
        stop_state, area_ratio = self._evaluate_stop(depth_out)
        self._stop_pub.publish(Bool(data=stop_state))

        # ── Raw depth (machine consumers) ────────────────────────────────────
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

        # ── Colorised visualisation + ROI/trigger overlay (humans, rqt) ──────
        depth_u8 = (depth_out * 255).astype(np.uint8)
        color_bgr = cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)
        self._draw_roi_overlay(color_bgr, stop_state)
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
            self.get_logger().info(
                f"Depth inference running — frame {self._frame_count}, "
                f"area_ratio={area_ratio:.2f}, stop={stop_state}"
            )


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
