# so101_yolo_demo

CPU-capable **YOLOv8n** person/object detection demo for the SO-101 — **no
NVIDIA GPU required**.

This is the "swap the model, do something different" sibling of
[`so101_depth_demo`](../so101_depth_demo): same overall shape (ONNX Runtime
CPU inference, subscribes to a ROS image topic, publishes a viz image +
machine-readable topic), but recognises object classes (person, by default)
instead of estimating depth. It also computes and publishes
`/safety/protective_stop` directly (ROI overlap on the already
class-filtered detections + hysteresis debounce) — the SAME topic
`depth_anything_node` publishes to — so `safety_pause_bridge` /
`trajectory_safety_gate` (`so101_safety`, enforcement-only, perception-
agnostic) never need to change, only the perception method driving the stop.

---

## Nodes

| Executable | Description |
|---|---|
| `yolo_detect_node` | Subscribes to an image topic, runs YOLOv8n ONNX detection (NMS baked in), publishes an annotated viz image (with the safety ROI + trigger state overlaid) + `vision_msgs/Detection2DArray` + `std_msgs/Bool` protective-stop. |
| `test_image_publisher` | Publishes a synthetic or static image so you can test with no camera. |

## Parameters (`yolo_detect_node`)

| Param | Default | Notes |
|---|---|---|
| `model_path` | `~/models/yolov8n.onnx` | ONNX model file (NMS baked in — see below). |
| `input_image_topic` | `/static_camera/image_raw` | Overhead camera topic in this repo. |
| `output_image_topic` | `/camera/detections/visualization` | Annotated detection viz output (boxes + labels + safety ROI/trigger overlay). |
| `output_detections_topic` | `/perception/detections` | `vision_msgs/Detection2DArray` for other machine consumers. |
| `input_size` | `640` | Square model input. |
| `conf_threshold` | `0.4` | Minimum detection score to keep. |
| `class_filter` | `person` | Comma-separated COCO class names/ids to keep; empty = all 80 classes. |
| `min_period_s` | `0.0` | Min seconds between inferences (CPU throttle). |
| `intra_op_threads` | `0` | ONNX Runtime threads (0 = library default). |
| `stop_topic` | `/safety/protective_stop` | `std_msgs/Bool` protective-stop output. |
| `roi` | `0.25,0.2,0.75,0.85` | Normalised `x1,y1,x2,y2` center ROI. |
| `min_overlap_ratio` | `0.2` | Minimum fraction of a watched detection's box that must fall inside the ROI to count. |
| `frames_to_block` | `2` | Consecutive triggered frames to assert stop. |
| `frames_to_clear` | `3` | Consecutive clear frames to release stop. |

---

## The model

`model_path` must point at a YOLOv8n ONNX export with **NMS baked into the
graph** (`output0` shape `[1, 300, 6]` = up to 300 `[x1, y1, x2, y2, conf,
cls]` rows, zero-padded, in `input_size`×`input_size` pixel space) — this
node does no manual NMS/decoding. Produced once from Ultralytics' official
`yolov8n.pt`:

```bash
pip install ultralytics onnx onnxslim
python3 -c "
from ultralytics import YOLO
YOLO('yolov8n.pt').export(
    format='onnx', imgsz=640, dynamic=False,
    simplify=True, nms=True, opset=12,
)
"
mkdir -p ~/models && cp yolov8n.onnx ~/models/yolov8n.onnx
```

The same file is vendored (not re-downloaded) at
[`models/yolov8n.onnx`](models/yolov8n.onnx) and bundled directly into the
packaged `ai-vision-ros2` snap at build time (no content interface, no
separate model snap) — see `docs/ai_vision_ros2_channel_demo.md`.

**Licensing note:** the YOLOv8n weights and the Ultralytics export tooling
are AGPL-3.0 licensed by Ultralytics.

---

## Dependencies

Python: `onnxruntime`, `numpy`, `opencv-python`, plus ROS 2 `rclpy`,
`sensor_msgs`, `vision_msgs`. Same PEP 668 / venv notes as
`so101_depth_demo` apply — install into the interpreter `ros2 launch` uses:

```bash
uv pip install onnxruntime numpy opencv-python
# or: python3 -m pip install --break-system-packages onnxruntime numpy opencv-python
```

---

## Manual container verification

```bash
cd <workspace_root>
colcon build --packages-select so101_yolo_demo
source install/setup.bash
```

Terminal A — synthetic camera:

```bash
ros2 run so101_yolo_demo test_image_publisher \
  --ros-args -p image_topic:=/static_camera/image_raw -p fps:=15.0
```

Terminal B — detection demo:

```bash
ros2 launch so101_yolo_demo yolo_demo.launch.py \
  model_path:=$HOME/models/yolov8n.onnx \
  input_image_topic:=/static_camera/image_raw
```

Terminal C — checks:

```bash
ros2 topic hz /camera/detections/visualization
ros2 topic echo --once /camera/detections/visualization --field encoding   # expect: rgb8
ros2 topic hz /perception/detections
ros2 run rqt_image_view rqt_image_view /camera/detections/visualization
```

## Swap into the full safety-stop demo

```bash
ros2 launch so101_depth_demo full_demo_real_servo.launch.py perception_backend:=detection
# or
ros2 launch so101_depth_demo full_demo_real_split.launch.py perception_backend:=detection
```

See [`docs/ai_vision_ros2_channel_demo.md`](../docs/ai_vision_ros2_channel_demo.md)
for the snap-based (production) setup -- including the single-snap,
channel-swap demo with the depth variant -- and
[`docs/demo_runbook.md`](../docs/demo_runbook.md) for the full hardware
runbook.

---

## What this package intentionally does **not** do

- It does **not** run NMS/box-decoding itself — that's baked into the
  exported ONNX graph.
- It does **not** open camera devices directly — use `so101_bringup` cameras.
- It does **not** require TensorRT, PyCUDA, CUDA, or an NVIDIA GPU.
- It does **not** change your robot integration (`so101_bringup`,
  `so101_teleop`, `so101_description`, controllers) — run those exactly as
  you do today.
