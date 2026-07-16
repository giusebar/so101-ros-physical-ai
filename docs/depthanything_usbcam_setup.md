# DepthAnything + usb-cam snap setup

Quick guide to stand up the real perception pipeline entirely from snaps —
**`usb-cam`** (camera) → **`depthanything`** (ONNX depth inference) — no ROS
workspace build required on the target machine.

```
usb-cam (real camera, e.g. /dev/video4)
  --publishes--> /static_camera/image_raw (sensor_msgs/Image)
                        |
                        v
depthanything (daemon, ONNX Runtime CPU inference)
  --publishes--> /camera/depth/visualization (sensor_msgs/Image, rgb8)
```

Both are always-on daemons configured at runtime with `snap set` (no launch
files, no manual `ros2 run`).

## 1. Build (or obtain) the three snaps

| Snap | Built from |
|---|---|
| `usb-cam` | [`snap-usb-cam/snapcraft.yaml`](../snap-usb-cam/snapcraft.yaml) |
| `depthanything-model` | [`snaps/depthanything-model/snapcraft.yaml`](../snaps/depthanything-model/snapcraft.yaml) |
| `depthanything` | [`so101_depth_demo/snap/snapcraft.yaml`](../so101_depth_demo/snap/snapcraft.yaml) |

```bash
cd snap-usb-cam && snapcraft                          # -> usb-cam_*.snap
cd ../snaps/depthanything-model && snapcraft           # -> depthanything-model_*.snap
cd ../../so101_depth_demo && snapcraft                 # -> depthanything_*.snap
```

> `depthanything` won't build locally until `depthanything-model` is
> published to the store (snapcraft resolves `default-provider` as a
> build-snap it tries to `snap install`). For a local test build, temporarily
> remove the `default-provider: depthanything-model` line, build, then put it
> back — don't commit it removed. See `snaps/README.md` for details.

## 2. Install

```bash
sudo snap install ./snap-usb-cam/usb-cam_*.snap --dangerous
sudo snap install ./snaps/depthanything-model/depthanything-model_*.snap --dangerous
sudo snap install ./so101_depth_demo/depthanything_*.snap --dangerous

# content interface doesn't auto-connect for --dangerous local installs:
sudo snap connect depthanything:depthanything-model depthanything-model:depthanything-model
# camera interface likewise needs a manual connect for local hardware access:
sudo snap connect usb-cam:camera
```

## 3. Configure

```bash
# Point usb-cam at the real device and give it a stable namespace/topic:
sudo snap set usb-cam device=/dev/video4 frame-id=cam_overhead \
  camera-name=cam_overhead namespace=/static_camera

# Point depthanything at whatever usb-cam actually publishes -- don't guess,
# confirm with `ros2 topic list` / `ros2 topic info` first (the namespace you
# chose above determines the exact topic name, e.g. /static_camera/image_raw):
sudo snap set depthanything input-image-topic=/static_camera/image_raw
```

Both `configure` hooks render ROS params and (re)start their daemon
automatically — no reboot / manual restart needed after `snap set`.

## 4. Verify

```bash
snap services usb-cam depthanything          # both should show "active"
sudo snap logs depthanything -n 20           # expect "ONNX session ready", "DepthAnythingNode ready"

# From anywhere with ROS 2 sourced (e.g. `workshop exec open-manipulator -- bash -lc '...'`):
source /opt/ros/jazzy/setup.bash
ros2 topic list                                              # both image_raw and depth/visualization should appear
ros2 topic info /static_camera/image_raw --verbose            # confirm usb-cam has a real publisher (Publisher count: 1)
ros2 topic hz /camera/depth/visualization                     # confirm depthanything is actually publishing (expect a few Hz on CPU)
ros2 topic echo --once /camera/depth/visualization --field encoding   # expect: rgb8
```

If `ros2 topic hz` shows nothing: check `ros2 topic info <input-image-topic>
--verbose` for `Publisher count: 0` (nobody's actually publishing there —
usually a topic/namespace mismatch between the two `snap set` calls above,
not a code bug).

## Other useful `snap set depthanything` options

`output-image-topic`, `model-input-size`, `publish-width`, `publish-height`,
`min-period-s` (CPU throttle), `intra-op-threads`, `namespace`, `node-name`,
`ros-domain-id`, `rmw-implementation`.

## Known gotchas (already fixed in the checked-in snapcraft.yaml, kept here for context)

- **`opencv-python-headless` needs system BLAS/LAPACK at runtime.** `cv2`
  `dlopen()`s `libblas.so.3`/`liblapack.so.3`, which aren't on the default
  library search path inside a strict-confinement snap. Symptom is a
  *misleading* crash-loop error: `ImportError: ... you should not try to
  import numpy from its source directory` — the real cause is a few lines
  above: `libblas.so.3: cannot open shared object file`. Fixed by staging
  `liblapack3`/`libblas3` and exporting `LD_LIBRARY_PATH` in
  `snap/local/depthanything-launch` (same fix `usb-cam` already needed for
  `cv_bridge`).
- **`default-provider` + local builds don't mix.** Snapcraft tries to
  `snap install` a content plug's `default-provider` from the store during
  the build, which fails until that snap is actually published.
- **`ros2_topic_hz` returning nothing usually means a topic-name mismatch**,
  not a broken node — always confirm the exact topic with `ros2 topic list`
  / `ros2 topic info --verbose` rather than assuming defaults line up between
  independently-configured snaps.
