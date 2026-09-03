# so101_depth_demo

CPU-capable **Depth Anything V2 (Small)** monocular depth visualization demo for
the SO-101 — **no NVIDIA GPU required**.

This package extracts the *vision* part of the
[`snap-twin`](../snap-twin) demo and reworks it so it:

- runs inference with **ONNX Runtime (CPU)** instead of TensorRT + PyCUDA,
- **subscribes to a ROS image topic** instead of opening a camera device directly,
- reuses your existing robot / camera / Gazebo integration from `so101_bringup`.

It publishes a colorised (INFERNO) relative-depth image on
`/camera/depth/visualization` (`sensor_msgs/Image`, `rgb8`) — the same topic the
original snap-twin display expects.

> **NVIDIA / Jetson variant:** the same `depth_anything_node` also runs on a
> Jetson GPU. This `nvidia/ai-demo` branch carries the TensorRT variant of the
> `ai-vision-ros2` snap (`snap/snapcraft.yaml`, ROS 2 Humble on core22) that
> runs ONNX Runtime's `TensorrtExecutionProvider` — selected purely via the
> `execution_providers` / `engine_cache_dir` ROS parameters, so this node
> itself stays hardware-agnostic. See
> [`../docs/ai_vision_ros2_nvidia.md`](../docs/ai_vision_ros2_nvidia.md).

> **Why not just run snap-twin?** The snap-twin
> [`depth_anything_node.py`](../snap-twin/so101_ros2/so101_ros2/depth_anything_node.py)
> imports `tensorrt` and `pycuda` at module load and loads a `.engine` file. On a
> non-NVIDIA machine it fails before inference. This package is the CPU path.

---

## Nodes

| Executable | Description |
|---|---|
| `depth_anything_node` | Subscribes to an image topic, runs ONNX depth inference, publishes colorised depth. |
| `depth_display_node` | Optional OpenCV window for the depth topic (needs X11). |
| `test_image_publisher` | Publishes a synthetic or static image so you can test with no camera. |

## Parameters (`depth_anything_node`)

| Param | Default | Notes |
|---|---|---|
| `model_path` | `~/models/depth_anything_v2_small.onnx` | ONNX model file. |
| `input_image_topic` | `/follower/image_raw` | Wrist camera topic in this repo. |
| `output_image_topic` | `/camera/depth/visualization` | Colorised depth output (with the safety ROI/trigger overlay). |
| `output_depth_topic` | `/perception/depth` | Raw normalised depth (32FC1) for other machine consumers. |
| `model_input_size` | `518` | Square model input. |
| `publish_width` / `publish_height` | `518` | Output image size. |
| `min_period_s` | `0.0` | Min seconds between inferences (CPU throttle). |
| `intra_op_threads` | `0` | ONNX Runtime threads (0 = library default). |
| `stop_topic` | `/safety/protective_stop` | `std_msgs/Bool` protective-stop output, computed directly by this node. |
| `roi` | `0.25,0.2,0.75,0.85` | Normalised `x1,y1,x2,y2` center ROI. |
| `near_threshold` | `0.6` | Normalised depth `[0,1]` above which a pixel is near. |
| `near_margin` | `0.15` | Margin added on top of the background reference. |
| `min_area_ratio` | `0.12` | Fraction of ROI that must be near to trigger. |
| `frames_to_block` | `2` | Consecutive near frames to assert stop. |
| `frames_to_clear` | `3` | Consecutive clear frames to release stop. |

This node computes and publishes the protective-stop decision itself (ROI
proximity + hysteresis debounce) — see
[`docs/ai_vision_ros2_channel_demo.md`](../docs/ai_vision_ros2_channel_demo.md)
for why (it enables a live `snap refresh --channel=...` swap with the
sibling `so101_yolo_demo` package, with zero ROS-side restart).

---

## Dependencies

Python: `onnxruntime`, `numpy`, `opencv-python`, plus ROS 2 `rclpy`,
`sensor_msgs`.

> **Ubuntu 24.04 / containers:** the system Python is externally managed
> (PEP 668), so a bare `pip3 install` fails. Install into the **same**
> interpreter that `ros2 launch` uses — i.e. the uv/venv you have sourced.
> If you install into a different environment, the node will fail with
> `ModuleNotFoundError: No module named 'onnxruntime'`.

With your uv/venv environment sourced:

```bash
# uv (recommended)
uv pip install onnxruntime numpy opencv-python

# or plain pip inside the active venv
python3 -m pip install onnxruntime numpy opencv-python
```

Verify the interpreter that ROS will use can import it:

```bash
python3 -c "import onnxruntime, sys; print('OK', sys.executable)"
```

If you must use the system Python (no venv), opt in explicitly:

```bash
python3 -m pip install --break-system-packages onnxruntime numpy opencv-python
```

## Get the model

```bash
# Saves $HOME/models/depth_anything_v2_small.onnx (no sudo / root needed)
bash so101_depth_demo/scripts/download_model.sh
```

---

## Manual container verification

All steps assume you are inside your ROS 2 container with this workspace mounted.
Run each block in its own terminal where noted, and `source install/setup.bash`
in every terminal after building.

### 0. Build

```bash
cd <workspace_root>
colcon build --packages-select so101_depth_demo
source install/setup.bash
```

### 1. Import check (confirm it is NVIDIA-free)

```bash
python3 -c "import so101_depth_demo.depth_anything_node as m; print('import OK')"
# Confirm no tensorrt / pycuda requirement:
python3 - <<'EOF'
import so101_depth_demo.depth_anything_node  # noqa
import sys
assert "tensorrt" not in sys.modules, "tensorrt should NOT be imported"
assert "pycuda" not in sys.modules, "pycuda should NOT be imported"
print("No tensorrt / pycuda imported — OK")
EOF
```

### 2. Model check

```bash
bash so101_depth_demo/scripts/download_model.sh
ls -lh ~/models/depth_anything_v2_small.onnx
python3 - <<'EOF'
import onnxruntime as ort
import os
s = ort.InferenceSession(os.path.expanduser("~/models/depth_anything_v2_small.onnx"),
                         providers=["CPUExecutionProvider"])
print("providers:", s.get_providers())
print("input:", s.get_inputs()[0].name, s.get_inputs()[0].shape)
print("output:", s.get_outputs()[0].name, s.get_outputs()[0].shape)
EOF
```

### 3. End-to-end with a synthetic image (no camera)

Terminal A — synthetic camera:

```bash
ros2 run so101_depth_demo test_image_publisher \
  --ros-args -p image_topic:=/follower/image_raw -p fps:=15.0
```

Terminal B — depth demo:

```bash
ros2 launch so101_depth_demo depth_demo.launch.py \
  model_path:=$HOME/models/depth_anything_v2_small.onnx \
  input_image_topic:=/follower/image_raw
```

Terminal C — checks:

```bash
ros2 topic list | grep depth
ros2 topic hz /camera/depth/visualization
ros2 topic echo --once /camera/depth/visualization --field encoding   # expect: rgb8
```

View it (needs X11) with either:

```bash
ros2 run rqt_image_view rqt_image_view /camera/depth/visualization
# or the bundled display:
ros2 launch so101_depth_demo depth_demo.launch.py use_display:=true \
  model_path:=$HOME/models/depth_anything_v2_small.onnx
```

### 4. Real Logitech C920

Make sure the container can see the camera (e.g. `--device /dev/video0` or
mount `/dev/v4l/by-id`). Then point the existing camera stack at the C920.

Terminal A — camera (V4L2). Set `video_device` to your C920 in
[`so101_v4l2_cam.yaml`](../so101_bringup/config/cameras/so101_v4l2_cam.yaml)
(e.g. a stable `/dev/v4l/by-id/...` path), then:

```bash
ros2 launch so101_bringup cameras.launch.py \
  cameras_config:=$(ros2 pkg prefix so101_bringup)/share/so101_bringup/config/cameras/so101_v4l2_cam.yaml
```

Find the actual image topic:

```bash
ros2 topic list | grep image_raw
```

Terminal B — depth demo against that topic (example shown for the wrist cam):

```bash
ros2 launch so101_depth_demo depth_demo.launch.py \
  model_path:=$HOME/models/depth_anything_v2_small.onnx \
  input_image_topic:=/follower/cam_wrist/image_raw
```

> Tip: if CPU inference can't keep up, throttle it:
> add `min_period_s:=0.2` (≈5 Hz) to the depth launch.

### 5. Gazebo + depth demo together

Terminal A — existing robot simulation (unchanged):

```bash
ros2 launch so101_bringup gazebo_teleop_sim.launch.py
```

Terminal B — depth demo. The default Gazebo world has no simulated camera yet,
so feed it the C920 (step 4) or the synthetic publisher (step 3):

```bash
ros2 run so101_depth_demo test_image_publisher \
  --ros-args -p image_topic:=/follower/image_raw -p fps:=15.0 &
ros2 launch so101_depth_demo depth_demo.launch.py \
  model_path:=$HOME/models/depth_anything_v2_small.onnx \
  input_image_topic:=/follower/image_raw
```

Confirm the robot sim stays healthy while depth publishes:

```bash
ros2 topic hz /camera/depth/visualization
ros2 topic list | grep -E "joint_states|controller"
```

### 6. CPU performance note

`ros2 topic hz /camera/depth/visualization` shows your effective rate. Depth
Anything V2 Small on CPU is meant for **functional testing** — expect well below
the Jetson TensorRT demo's 15 Hz. Tune with `min_period_s` and `intra_op_threads`.

---

## What this package intentionally does **not** do

- It does **not** include snap-twin teleop / gesture / overhead-vision logic.
- It does **not** open camera devices directly — use `so101_bringup` cameras.
- It does **not** require TensorRT, PyCUDA, CUDA, or an NVIDIA GPU.
- It does **not** change your robot integration (`so101_bringup`, `so101_teleop`,
  `so101_description`, controllers) — run those exactly as you do today.
