# yolodetect + usb-cam snap setup

Quick guide to stand up the detection-based perception pipeline entirely
from snaps — **`usb-cam`** (camera) → **`yolodetect`** (ONNX YOLOv8n
detection) — no ROS workspace build required on the target machine. This is
the "swap the model, do something different" sibling of
[`depthanything_usbcam_setup.md`](depthanything_usbcam_setup.md).

```
usb-cam (real camera, e.g. /dev/video4)
  --publishes--> /static_camera/image_raw (sensor_msgs/Image)
                        |
                        v
yolodetect (daemon, ONNX Runtime CPU inference, NMS baked into the model)
  --publishes--> /camera/detections/visualization (sensor_msgs/Image, rgb8)
  --publishes--> /perception/detections (vision_msgs/Detection2DArray)
```

Both are always-on daemons configured at runtime with `snap set` (no launch
files, no manual `ros2 run`).

## 1. Build (or obtain) the three snaps

| Snap | Built from |
|---|---|
| `usb-cam` | [`snap-usb-cam/snapcraft.yaml`](../snap-usb-cam/snapcraft.yaml) |
| `yolodetect-model` | [`snap-yolodetect-model/snap/snapcraft.yaml`](../snap-yolodetect-model/snap/snapcraft.yaml) |
| `yolodetect` | [`so101_yolo_demo/snap/snapcraft.yaml`](../so101_yolo_demo/snap/snapcraft.yaml) |

```bash
cd snap-usb-cam && snapcraft                          # -> usb-cam_*.snap
cd ../snap-yolodetect-model && snapcraft               # -> yolodetect-model_*.snap
cd ../so101_yolo_demo && snapcraft                     # -> yolodetect_*.snap
```

> Unlike `depthanything-model`, `yolodetect-model` does not download
> anything at build time -- its `models/yolov8n.onnx` is vendored directly
> in the source tree (see `snap-yolodetect-model/README.md` for why), so it
> builds offline.
>
> `yolodetect` still won't build locally until `yolodetect-model` is
> published to the store (snapcraft resolves `default-provider` as a
> build-snap it tries to `snap install`). For a local test build, temporarily
> remove the `default-provider: yolodetect-model` line, build, then put it
> back — don't commit it removed.

## 2. Install

```bash
sudo snap install ./snap-usb-cam/usb-cam_*.snap --dangerous
sudo snap install ./snap-yolodetect-model/yolodetect-model_*.snap --dangerous
sudo snap install ./so101_yolo_demo/yolodetect_*.snap --dangerous

# content interface doesn't auto-connect for --dangerous local installs:
sudo snap connect yolodetect:yolodetect-model yolodetect-model:yolodetect-model
# camera interface likewise needs a manual connect for local hardware access:
sudo snap connect usb-cam:camera
```

## 3. Configure

```bash
# Point usb-cam at the real device and give it a stable namespace/topic:
sudo snap set usb-cam device=/dev/video4 frame-id=cam_overhead \
  camera-name=cam_overhead namespace=/static_camera

# Point yolodetect at whatever usb-cam actually publishes -- don't guess,
# confirm with `ros2 topic list` / `ros2 topic info` first:
sudo snap set yolodetect input-image-topic=/static_camera/image_raw

# Optional: change which classes trigger downstream logic, confidence, etc.
sudo snap set yolodetect class-filter=person conf-threshold=0.4
```

Both `configure` hooks render ROS params and (re)start their daemon
automatically — no reboot / manual restart needed after `snap set`.

## 4. Verify

```bash
snap services usb-cam yolodetect              # both should show "active"
sudo snap logs yolodetect -n 20                # expect "ONNX session ready", "YoloDetectNode ready"

# From anywhere with ROS 2 sourced:
source /opt/ros/jazzy/setup.bash
ros2 topic list                                                    # image_raw, detections/visualization, /perception/detections
ros2 topic info /static_camera/image_raw --verbose                 # confirm usb-cam has a real publisher (Publisher count: 1)
ros2 topic hz /camera/detections/visualization                     # confirm yolodetect is actually publishing
ros2 topic echo --once /camera/detections/visualization --field encoding   # expect: rgb8
ros2 topic hz /perception/detections
```

If `ros2 topic hz` shows nothing: check `ros2 topic info <input-image-topic>
--verbose` for `Publisher count: 0` (usually a topic/namespace mismatch
between the two `snap set` calls above, not a code bug).

## 5. Swap with `depthanything` at runtime

This is the actual "power of snaps" demo: install/remove the perception snap
to change what triggers the robot's protective stop, without touching the
camera, the arms, teleop, or the enforcement nodes at all (they both
converge on the same `/safety/protective_stop` topic via their respective
`so101_safety` monitor node):

```bash
sudo snap remove depthanything
sudo snap install ./so101_yolo_demo/yolodetect_*.snap --dangerous
sudo snap connect yolodetect:yolodetect-model yolodetect-model:yolodetect-model
sudo snap set yolodetect input-image-topic=/static_camera/image_raw

# relaunch the full demo with the detection backend:
ros2 launch so101_depth_demo full_demo_real_servo.launch.py perception_backend:=detection
# or:
ros2 launch so101_depth_demo full_demo_real_split.launch.py perception_backend:=detection
```

To go back: `sudo snap remove yolodetect && sudo snap install
./so101_depth_demo/depthanything_*.snap --dangerous`, reconnect its content
interface, and relaunch with `perception_backend:=depth` (the default).

## Other useful `snap set yolodetect` options

`output-image-topic`, `output-detections-topic`, `input-size`,
`min-period-s` (CPU throttle), `intra-op-threads`, `namespace`, `node-name`,
`ros-domain-id`, `rmw-implementation`.

## Known gotchas (shared with `depthanything`, kept here for context)

- **`opencv-python-headless` needs system BLAS/LAPACK at runtime.** Same
  `liblapack3`/`libblas3` staging + `LD_LIBRARY_PATH` fix as `depthanything`
  (see `depthanything_usbcam_setup.md` for the full explanation).
- **`default-provider` + local builds don't mix.** Same as
  `depthanything`/`depthanything-model`.
- **A topic-name mismatch, not a broken node, is the usual cause of
  `ros2 topic hz` showing nothing** — always confirm with `ros2 topic list`
  / `ros2 topic info --verbose`.
- **`image_width`/`image_height` on `person_safety_monitor` must match the
  camera's actual resolution** (default 640x480) — the ROI is interpreted in
  pixel space against the detections' bounding boxes, unlike
  `depth_safety_monitor`'s ROI which is normalised against the depth image's
  own reported size.
