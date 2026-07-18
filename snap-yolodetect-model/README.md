# yolodetect snaps

Production packaging for the SO-101 YOLOv8n person/object-detection demo,
split into **two snaps** using the snap **content interface** -- the "swap
the model, do something different" sibling of the `depthanything` /
`depthanything-model` pair.

> For a full end-to-end setup (including `usb-cam` as the real image source)
> see [`docs/yolodetect_setup.md`](../docs/yolodetect_setup.md).

| Snap | Role | Location |
|---|---|---|
| `yolodetect-model` | Content snap — ships only the ONNX weights (vendored in-repo, NMS baked in), exposed via a `content` slot. | [snapcraft.yaml](snap/snapcraft.yaml) |
| `yolodetect` | ROS 2 Jazzy app snap — the `yolo_detect_node` pipeline. Plugs the model in at `$SNAP/models`. | [../so101_yolo_demo/snap/snapcraft.yaml](../so101_yolo_demo/snap/snapcraft.yaml) |

## Why the model is vendored, not downloaded at build time

Unlike `depthanything-model` (which `curl`s its weights from the Hugging
Face hub at build time), no trustworthy pre-exported YOLOv8n ONNX with the
export flags this demo needs (`imgsz=640`, `dynamic=False`, `simplify=True`,
`nms=True`, `opset=12`) was found hosted anywhere with clear, verifiable
provenance. The model was exported once locally from Ultralytics' own
official `yolov8n.pt` release and committed to `models/yolov8n.onnx`:

```bash
pip install ultralytics onnx onnxslim
python3 -c "
from ultralytics import YOLO
YOLO('yolov8n.pt').export(
    format='onnx', imgsz=640, dynamic=False,
    simplify=True, nms=True, opset=12,
)
"
```

`nms=True` bakes NMS into the exported graph, so `yolo_detect_node` (and any
other consumer) only needs to parse the final `output0` tensor
(`[1, 300, 6]` = up to 300 `[x1, y1, x2, y2, conf, cls]` rows, zero-padded) --
no manual NMS/decoding.

**Licensing note:** the YOLOv8n weights and the Ultralytics export tooling
used to produce this ONNX file are AGPL-3.0 licensed by Ultralytics.

## Build

```bash
# Model content snap (vendored weights, no network access needed at build time)
cd <repo-root>/snap-yolodetect-model
snapcraft            # -> yolodetect-model_*.snap

# App snap (ROS 2 Jazzy, colcon build of so101_yolo_demo)
cd <repo-root>/so101_yolo_demo
snapcraft            # -> yolodetect_*.snap
```

> **Local/dev builds of `yolodetect` will fail until `yolodetect-model` is
> published to the store** (same `default-provider` build-snap resolution
> issue as `depthanything`/`depthanything-model` -- see
> `docs/depthanything_usbcam_setup.md`). For a local test build, temporarily
> comment out `default-provider: yolodetect-model` in
> `so101_yolo_demo/snap/snapcraft.yaml`, build, then put it back.

## Install locally (from built .snap files)

```bash
snap install --dangerous yolodetect-model_*.snap
snap install --dangerous yolodetect_*.snap
# Content interface auto-connects for store installs; for --dangerous local
# installs connect it manually:
snap connect yolodetect:yolodetect-model yolodetect-model:yolodetect-model
```

## Run

`yolodetect` is an **always-on daemon** (like `usb-cam` / `depthanything`),
not a one-shot CLI. Configure it with `snap set` -- the `configure` hook
renders ROS parameters and (re)starts the service:

```bash
snap set yolodetect input-image-topic=/static_camera/image_raw
snap set yolodetect output-image-topic=/camera/detections/visualization
snap set yolodetect output-detections-topic=/perception/detections
snap set yolodetect class-filter=person conf-threshold=0.4

snap services yolodetect
snap logs yolodetect -n 20
ros2 topic hz /perception/detections
```

Other options: `input-size`, `min-period-s` (CPU throttle),
`intra-op-threads`, `namespace`, `node-name`, `ros-domain-id`,
`rmw-implementation`.

If the `yolodetect-model` content interface isn't connected yet (common
right after a `--dangerous` local install), the daemon stays stopped and
`snap services yolodetect` / `snap changes` reports a blocked health until
you connect it and re-run `snap set`.

## Swapping with `depthanything` (the "power of snaps" demo)

Both snaps publish a different perception-specific topic
(`/perception/depth` vs `/perception/detections`), but both ultimately feed
the SAME `/safety/protective_stop` contract via their respective
`so101_safety` monitor node (`depth_safety_monitor` vs
`person_safety_monitor`) -- so swapping the perception snap changes what the
robot reacts to, without touching the enforcement side (`safety_pause_bridge`
/ `trajectory_safety_gate`) at all:

```bash
sudo snap remove depthanything
sudo snap install ./yolodetect_*.snap --dangerous
sudo snap connect yolodetect:yolodetect-model yolodetect-model:yolodetect-model
sudo snap set yolodetect input-image-topic=/static_camera/image_raw

# then relaunch the demo with the detection backend:
ros2 launch so101_depth_demo full_demo_real_servo.launch.py perception_backend:=detection
```

## Notes

- ROS 2 traffic uses the `network` / `network-bind` interfaces (DDS discovery).
- `onnxruntime`, `numpy`, and `opencv-python-headless` are installed into the
  snap's `dist-packages` (they are not ROS rosdeps), matching `depthanything`.
- Same `liblapack3`/`libblas3` staging + `LD_LIBRARY_PATH` fix as
  `depthanything` (`opencv-python-headless` dlopens BLAS/LAPACK at runtime).
- The daemon execs `yolo_detect_node` directly (no `ros2 launch`), matching
  the `usb-cam`/`depthanything` snap pattern.
