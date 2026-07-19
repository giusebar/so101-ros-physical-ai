# ai-vision-ros2: single bundled-model snap, two channels

`ai-vision-ros2` is **one snap name** with two completely different,
independently-built implementations published on different channels:

| Channel | Built from | Behavior |
|---|---|---|
| `latest/stable` | [`so101_depth_demo/snap/snapcraft.yaml`](../so101_depth_demo/snap/snapcraft.yaml) | Depth Anything V2 (Small) monocular depth proximity |
| `latest/edge` | [`so101_yolo_demo/snap/snapcraft.yaml`](../so101_yolo_demo/snap/snapcraft.yaml) | YOLOv8n person/object detection |

Both variants:
- run the **same app/service name**, `perception` (so `snap services
  ai-vision-ros2` / the systemd unit stay identically named across a channel
  swap, even though the underlying ROS package is completely different),
- **bundle their ONNX model weights directly in the snap** at build time —
  no `content` interface, no separate model snap, no manual `snap connect`
  step, no `default-provider` build-snap gotcha (unlike the depthanything/
  depthanything-model and yolodetect/yolodetect-model split used elsewhere in
  this repo — see [`depthanything_usbcam_setup.md`](depthanything_usbcam_setup.md)
  / [`yolodetect_setup.md`](yolodetect_setup.md) for that architecture),
- **compute and publish `/safety/protective_stop` (`std_msgs/Bool`) directly,
  themselves** -- `depth_anything_node` (ROI + background-reference proximity
  + hysteresis debounce) and `yolo_detect_node` (ROI overlap on the
  already class-filtered detections + hysteresis debounce) each own their
  full trigger logic, rather than delegating it to a separate
  `so101_safety` monitor node. `so101_safety` only contains the two
  perception-agnostic enforcement nodes (`safety_pause_bridge`,
  `trajectory_safety_gate`), which just subscribe to that one shared topic
  and don't care who published it or why.

This last point is what makes the "swap the AI model" demo work with **zero
ROS-side restart at all**: once your full demo (`full_demo_real_servo.launch.py`
/ `full_demo_real_split.launch.py`) is up and running against the snap, a
`snap refresh ai-vision-ros2 --channel=...` swaps the entire perception +
decision-making behaviour underneath it live -- the enforcement node just
keeps consuming whatever the currently-installed backend publishes to
`/safety/protective_stop`. You never need to touch the ROS launch at all --
this is the **official Snap Store channel/refresh mechanism**, not
installing/removing two separate snaps:

```
snap install ai-vision-ros2                       # latest/stable -> depth proximity
snap refresh ai-vision-ros2 --channel=edge         # -> now person detection, ROS side untouched
snap refresh ai-vision-ros2 --channel=stable       # -> back to depth proximity, ROS side untouched
```

## ⚠️ Channels require the Snap Store — `--dangerous` local installs don't support them

`snap refresh --channel=...` is a Store concept. A `--dangerous` local
`.snap` file install has no channel metadata at all, so you **cannot** demo
the channel swap purely locally. To test the swap for real, the snap needs
to be **registered and pushed to the Store** (it can be unlisted/private —
it does not need to be public) under a shared name.

You can, however, fully test **each variant standalone** locally with
`--dangerous` first (recommended before publishing) — see below.

## 1. Local testing (no Store needed) — verify each variant works before publishing

Build and test each variant **one at a time** (they share the same snap
name, so only one can be installed locally at once — `snap install` a
second `.snap` with the same `name:` replaces the first, same as a channel
refresh would):

```bash
# --- Depth variant ---
cd so101_depth_demo/snap
snapcraft                                    # -> ai-vision-ros2_1.0.0-depth_amd64.snap
bash smoke_tests.sh                          # installs --dangerous, configures, checks logs/services

# --- swap to the detection variant locally (simulates what a channel refresh will do) ---
sudo snap remove ai-vision-ros2
cd ../../so101_yolo_demo/snap
snapcraft                                    # -> ai-vision-ros2_1.0.0-yolo_amd64.snap
bash smoke_tests.sh
```

Each `smoke_tests.sh` installs, configures (`snap set`), and checks
`snap services` / `snap logs` — no content-interface connect step needed
either way, since the model is already bundled in the snap you just built.

## 2. Publish to the Store (you drive this step)

```bash
snapcraft login
snapcraft register ai-vision-ros2       # once; can be for an unlisted/private snap

# Depth variant -> latest/stable
cd so101_depth_demo/snap
snapcraft
snapcraft upload ai-vision-ros2_1.0.0-depth_amd64.snap --release=stable

# Detection variant -> latest/edge
cd ../../so101_yolo_demo/snap
snapcraft
snapcraft upload ai-vision-ros2_1.0.0-yolo_amd64.snap --release=edge
```

## 3. The channel-swap demo

### Perception + viz only (no arms)

```bash
# Camera (usb-cam snap) -- unchanged, independent of which AI variant is running:
sudo snap set usb-cam device=/dev/video4 frame-id=cam_overhead \
  camera-name=cam_overhead namespace=/static_camera

# Install the depth variant (stable is the default channel):
sudo snap install ai-vision-ros2
sudo snap set ai-vision-ros2 input-image-topic=/static_camera/image_raw

snap services ai-vision-ros2                 # perception: active
ros2 topic hz /safety/protective_stop         # confirm depth proximity is running and publishing the stop signal directly
ros2 run rqt_image_view rqt_image_view /camera/depth/visualization
# -> colorised depth with a green/red ROI box + "clear"/"PROTECTIVE STOP" label drawn on it

# Swap to person detection, same snap name, just a different channel:
sudo snap refresh ai-vision-ros2 --channel=edge
sudo snap set ai-vision-ros2 input-image-topic=/static_camera/image_raw
# (snap set is per-revision, so re-apply the topic config after a channel swap
#  the first time -- each variant has a different set of `snap set` keys, see
#  each package's README)

snap services ai-vision-ros2                 # perception: active (different code now)
ros2 run rqt_image_view rqt_image_view /camera/detections/visualization
# -> now shows bounding boxes + the same ROI box/label overlay, driven by detection instead of depth

# Swap back:
sudo snap refresh ai-vision-ros2 --channel=stable
```

### Full arm demo -- swap live, with ZERO ROS-side restart

Since both variants publish `/safety/protective_stop` directly, once the
full demo is running you can refresh channels as many times as you like
without ever touching `ros2 launch`. `perception_backend` still exists as a
launch arg, but only for `launch_depth:=true` (dev/testing without the snap
-- picks which node, `depth_anything_node` or `yolo_detect_node`, to bring
up inline). It has no effect on the snap-based (default) path, on
safety-monitor selection (there is no separate monitor to select), or on
the viewer (`rqt_image_view` has its own live topic dropdown, see below).

```bash
# One-time: bring up the whole demo (arms, teleop, RViz, safety enforcement,
# and a single rqt_image_view window with a live topic-selector dropdown --
# perception_backend has no effect on it at all).
ros2 launch so101_depth_demo full_demo_real_servo.launch.py \
  leader_usb_port:=/dev/ttyACM1 follower_usb_port:=/dev/ttyACM0 use_viewer:=true

# ... now, in another terminal, at any point:
sudo snap refresh ai-vision-ros2 --channel=edge     # arm now stops for a detected person instead of depth proximity
sudo snap refresh ai-vision-ros2 --channel=stable   # back to depth proximity
# The demo launch never restarted. Only the AI snap did. Switch the
# rqt_image_view dropdown between /camera/depth/visualization and
# /camera/detections/visualization yourself to match whichever channel you're on.
```

## Verify

```bash
snap info ai-vision-ros2       # shows installed channel + both tracked channels' versions
snap services ai-vision-ros2   # always "perception", regardless of channel
sudo snap logs ai-vision-ros2 -n 20
ros2 topic list                # different perception-side topics appear/disappear across the swap
```

## Configuration keys per variant

**Depth (`latest/stable`)** — mirrors `depth_demo.launch.py`:
`input-image-topic`, `output-image-topic`, `output-depth-topic`,
`model-input-size`, `publish-width`, `publish-height`, `min-period-s`,
`intra-op-threads`, `namespace`, `node-name`, `ros-domain-id`,
`rmw-implementation`, plus the protective-stop trigger tuning:
`stop-topic`, `roi`, `near-threshold`, `near-margin`, `min-area-ratio`,
`frames-to-block`, `frames-to-clear`.

**Detection (`latest/edge`)** — mirrors `yolo_demo.launch.py`:
`input-image-topic`, `output-image-topic`, `output-detections-topic`,
`input-size`, `conf-threshold`, `class-filter`, `min-period-s`,
`intra-op-threads`, `namespace`, `node-name`, `ros-domain-id`,
`rmw-implementation`, plus the protective-stop trigger tuning:
`stop-topic`, `roi`, `min-overlap-ratio`, `frames-to-block`,
`frames-to-clear`.

`model_path` is never user-settable in either variant — it always points at
the model bundled inside that specific snap revision.

## Known gotchas

- **`intra-op-threads` defaults to `4` in both snaps' `configure` hooks
  (unlike the plain-launch/ROS-param default of `0`).** Leaving it at `0`
  lets ONNX Runtime auto-detect the thread count as the number of CPU cores
  *and* try to `pthread_setaffinity_np()` each worker thread to a specific
  core. Strict confinement's seccomp profile denies that syscall, so on a
  many-core host this floods `snap logs` with alarming (but actually
  harmless/non-fatal) `pthread_setaffinity_np failed ... Operation not
  permitted` errors — one per thread, so potentially 20+ lines per daemon
  start. It looks like a crash-loop but isn't one (`snap services` still
  shows `active`, and the ONNX session loads and starts publishing
  normally regardless). Passing an explicit thread count (`snap set
  ai-vision-ros2 intra-op-threads=N`) skips ORT's auto-affinity-pinning path
  entirely and eliminates the noise. If you still see it, rebuild — this is
  now the default in `snap/hooks/configure` in both variants.
- **Benign confinement-related warnings on every start** (also harmless):
  `GetGpuDevices ... Permission denied` (ORT probing `/sys/class/drm` /
  `/sys/bus/pci/devices` for a GPU it will never use — falls back to
  `CPUExecutionProvider` cleanly) and `RTPS_TRANSPORT_SHM ... Failed to
  create Shared Memory Manager` (fastDDS falls back to non-shared-memory
  transport; ROS 2 communication still works over the network loopback).
- Same BLAS/LAPACK and `default-provider`-vs-local-build gotchas as
  `depthanything`/`yolodetect` apply where relevant — see
  `depthanything_usbcam_setup.md` / `yolodetect_setup.md`.

## Why bundle instead of using the content interface (recap)

- **No Store approval wait for `default-provider` auto-connect** — the
  content-interface split (`depthanything`/`depthanything-model`,
  `yolodetect`/`yolodetect-model`) needs its content-plug default-provider
  approved before `snap install <app>` can auto-install and auto-connect the
  model snap. Bundling sidesteps that entirely.
- **Enables the channel-refresh trick** — a content-interface snap's model
  is a *separate* installable snap; there's no clean way to make "swap the
  model" a single `snap refresh --channel=...` on one snap name. Bundling
  puts the whole capability (code + weights) in one revision, so a channel
  swap really does swap everything at once.
- **Trade-off:** each new model revision re-uploads the full weights (tens
  of MB) as part of the app snap, instead of being a separately-versioned,
  independently-releasable content snap. Fine for this demo; worth
  reconsidering for a production release cadence where the model is updated
  much more often than the code (that's what the content-interface
  architecture in `depthanything_usbcam_setup.md` / `yolodetect_setup.md` is
  for).

## Why the perception nodes publish `/safety/protective_stop` directly (no `so101_safety` monitor)

The original architecture (still used by `depthanything`/`yolodetect`, see
the other docs) has a separate `so101_safety` monitor node per backend
(`depth_safety_monitor`, `person_safety_monitor`) that consumes a raw
perception topic and computes the stop decision. That works fine when you
restart the ROS launch on every backend swap (the launch file just picks
which monitor to bring up) -- but it doesn't work for a *live*
`snap refresh` swap: the ROS graph has no way to know the backend changed
underneath it, so whichever monitor was launched keeps listening to a topic
that may no longer have a publisher.

For `ai-vision-ros2`, the decision logic (ROI geometry, thresholds/overlap,
hysteresis debounce) was moved directly into `depth_anything_node` /
`yolo_detect_node` themselves, each publishing straight to
`/safety/protective_stop`. Since a single snap name only ever has ONE
revision actually running at a time (that's how snap channels work), there's
never a moment with two competing publishers or a stale abandoned one --
whichever backend is currently installed is simply the only thing publishing
that topic. `so101_safety` shrinks down to just the two enforcement nodes
(`safety_pause_bridge`, `trajectory_safety_gate`), which are fully
perception-agnostic and never need to change.

**Trade-off:** this couples "what counts as unsafe" domain logic into each
perception package instead of centralizing it in `so101_safety` (which is
still true, and preferable, for the `depthanything`/`yolodetect`
content-interface architecture, where a live no-restart swap isn't the
goal). Each perception node also carries a small local copy of the
ROI-parsing/hysteresis-debounce helper code (not imported from
`so101_safety`) -- required because each snap's `snapcraft.yaml` builds with
`source: .` scoped to just that one ROS package, so it has no visibility
into sibling packages at build time.
