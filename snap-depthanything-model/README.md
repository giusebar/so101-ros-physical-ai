# DepthAnything snaps

Production packaging for the SO-101 Depth Anything V2 (Small) depth demo, split
into **two snaps** using the snap **content interface**:

> For a full end-to-end setup (including `usb-cam` as the real image source)
> see [`docs/depthanything_usbcam_setup.md`](../docs/depthanything_usbcam_setup.md).

| Snap | Role | Location |
|---|---|---|
| `depthanything-model` | Content snap — ships only the ONNX weights, exposed via a `content` slot. | [snaps/depthanything-model/snapcraft.yaml](depthanything-model/snapcraft.yaml) |
| `depthanything` | ROS 2 Jazzy app snap — the `depth_anything_node` pipeline. Plugs the model in at `$SNAP/models`. | [../so101_depth_demo/snap/snapcraft.yaml](../so101_depth_demo/snap/snapcraft.yaml) |

## Why two snaps

- **Independent release cadence** — reship weights (retrain / new variant)
  without rebuilding the ROS app, and vice-versa.
- **Reuse** — other snaps can consume the same weights via the content interface.
- **Smaller app deltas** — code updates don't re-push tens of MB of weights.

The app declares the model plug with `default-provider: depthanything-model`, so
end users only run one command:

```bash
snap install depthanything
```

snapd resolves and installs `depthanything-model` automatically and auto-connects
the content interface. The weights are mounted read-only at `$SNAP/models`.

## Build

The app snap lives at `so101_depth_demo/snap/snapcraft.yaml` (the canonical
snapcraft location, relative to the `so101_depth_demo` ROS package — this
keeps `colcon`/`rosdep` scoped to just this package instead of the whole
monorepo), so it builds from that package directory:

```bash
# App snap (ROS 2 Jazzy, colcon build of so101_depth_demo)
cd <repo-root>/so101_depth_demo
snapcraft            # -> depthanything_0.1.0_amd64.snap
```

The model snap is self-contained (it downloads weights from Hugging Face at
build time) and builds from its own directory:

```bash
# Model content snap
cd <repo-root>/snaps/depthanything-model
snapcraft            # -> depthanything-model_2024.10.1_amd64.snap
```

> **Local/dev builds of `depthanything` will fail until `depthanything-model`
> is published to the store.** Snapcraft resolves a content plug's
> `default-provider` as a build-snap and tries to `snap install` it from the
> store during the build — this only works once the model snap is actually
> published (any channel). Until then, either build `depthanything-model`
> first and manually pre-install it into the build instance, or temporarily
> comment out `default-provider: depthanything-model` in `snap/snapcraft.yaml`
> for a local test build (put it back before publishing/committing).

## Install locally (from built .snap files)

```bash
snap install --dangerous depthanything-model_*.snap
snap install --dangerous depthanything_*.snap
# Content interface auto-connects for store installs; for --dangerous local
# installs connect it manually:
snap connect depthanything:depthanything-model depthanything-model:depthanything-model
```

## Run

`depthanything` is an **always-on daemon** (like `usb-cam`), not a one-shot
CLI. Configure it with `snap set` — the `configure` hook renders ROS
parameters and (re)starts the service:

```bash
snap set depthanything input-image-topic=/follower/image_raw
snap set depthanything output-image-topic=/camera/depth/visualization
snap set depthanything min-period-s=0.1

snap services depthanything
snap logs depthanything -n 20
ros2 topic hz /camera/depth/visualization
```

Other options: `model-input-size`, `publish-width`, `publish-height`,
`intra-op-threads`, `namespace`, `node-name`, `ros-domain-id`,
`rmw-implementation`.

If the `depthanything-model` content interface isn't connected yet (common
right after a `--dangerous` local install), the daemon stays stopped and
`snap services depthanything` / `snap changes` reports a blocked health
until you connect it and re-run `snap set`.

## Notes

- ROS 2 traffic uses the `network` / `network-bind` interfaces (DDS discovery).
- `opencv-python-headless`, `onnxruntime`, and `numpy` are installed into the
  snap's `dist-packages` (they are not ROS rosdeps).
- The model snap validates the download (size + not-an-HTML-error-page) so a
  broken mirror fails the build instead of shipping a corrupt model.
- The daemon execs `depth_anything_node` directly (no `ros2 launch`), matching
  the `usb-cam` snap's pattern. The optional `depth_display_node` (needs X11)
  is not packaged in the daemon.
