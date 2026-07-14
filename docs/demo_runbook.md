# SO-101 Hardware Demo Runbook

Practical guide to running the SO-101 leader/follower demos on **real hardware**
inside the Canonical workshop container, plus the equivalent **simulation**
commands. Also documents the one-time container setup that had to be done
manually, so it can be turned into reproducible provisioning.

> Status: working notes from bring-up. Items marked **[PROD TODO]** are things to
> clean up before this is production-ready.

---

## 1. Environment facts (this container)

| Thing | Value |
| --- | --- |
| ROS distro | `jazzy` (ros2-desktop SDK) |
| Colcon workspace | `/home/workshop/workspace` (src symlinks to `/project`) |
| Base ROS source | `source /opt/ros/jazzy/setup.bash` |
| Overlay source | `source /home/workshop/workspace/install/setup.bash` |
| **Leader arm** | `/dev/ttyACM1` (serial `5AB9065439`) |
| **Follower arm** | `/dev/ttyACM0` (serial `5AB9064894`) |
| **Overhead camera** | Logitech C920 at `/dev/video4` |
| Depth model | `~/models/depth_anything_v2_small.onnx` |
| Display | `DISPLAY=:0` (X11 + Wayland available) |

> Serial device numbering (`ttyACM0/1`) is **not stable** across replug/reboot.
> Confirm with:
> ```bash
> for d in ttyACM0 ttyACM1; do echo "$d -> $(cat /sys/class/tty/$d/device/../serial)"; done
> ```
> **[PROD TODO]** Add udev rules for stable `/dev/so101_leader` / `/dev/so101_follower`
> symlinks (see `docs/hardware.md`). The container currently has no
> `/dev/serial/by-id`, so we pass raw `/dev/ttyACM*` paths.

Every new terminal needs the two source lines:

```bash
source /opt/ros/jazzy/setup.bash
source /home/workshop/workspace/install/setup.bash
```

---

## 2. One-time container setup (manual steps we had to do)

These were **not** provisioned by the workshop image and had to be done by hand.
See §6 for turning them into workshop.yaml actions.

### 2.1 Fetch the vendored driver submodule

`feetech_ros2_driver` is a git submodule and was **not checked out** (empty dir →
`ros2_control` could not find the `feetech_ros2_driver/FeetechHardwareInterface`
plugin).

```bash
cd /project
git submodule update --init --recursive
```

### 2.2 System dependencies

```bash
# Feetech driver build dependency (declared rosdep, was missing)
sudo apt-get update
sudo apt-get install -y libserial-dev

# pip for the system Python (was missing)
sudo apt-get install -y python3-pip
```

### 2.3 Python dependency for the depth demo

The depth node runs under the interpreter `ros2 launch` uses (`/usr/bin/python3`).
Ubuntu 24.04 is PEP 668 (externally managed), so:

```bash
python3 -m pip install --break-system-packages onnxruntime
# numpy + opencv (cv2) were already present from the ROS image
python3 -c "import onnxruntime, numpy, cv2; print('deps OK')"
```

> **[PROD TODO]** A venv is cleaner, but the ROS Python entry-point nodes run with
> the system interpreter, so onnxruntime must be importable from `/usr/bin/python3`
> (or you must make `ros2` use the venv interpreter). For provisioning, either
> `--break-system-packages` or a properly-sourced venv both work — pick one and
> pin the version.

### 2.4 Download the depth model

```bash
bash /project/so101_depth_demo/scripts/download_model.sh
ls -lh ~/models/depth_anything_v2_small.onnx   # ~95 MB
```

### 2.5 Build the workspace

```bash
cd /home/workshop/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select feetech_ros2_driver so101_teleop so101_bringup so101_depth_demo
source install/setup.bash
```

> **[PROD TODO] Repo fixes already applied in-tree** (so a fresh build works):
> - `so101_teleop/CMakeLists.txt` — the `trajectory_safety_gate` build target was
>   missing; re-added (executable + deps + install). Without it the installed
>   binary was a stale version listening on the wrong topic.
> - `so101_depth_demo/launch/depth_safety_stop.launch.py` — camera driver switched
>   `v4l2_camera` → `usb_cam` (v4l2_camera not installed), and gate wired for real
>   hardware (`use_sim_time:=false`, `trajectory_controller`).
> - `so101_depth_demo/launch/full_demo_real.launch.py` — new real-hardware launch.
> - `so101_bringup/launch/layout_tf.launch.py` — arm layout aligned to Gazebo.

---

## 3. Run: teleoperation only (real arms)

Move the **physical leader** by hand → the follower mirrors it. No keyboard/xterm
is needed on real hardware (the leader arm is the input device).

### 3.1 One command

```bash
source /opt/ros/jazzy/setup.bash
source /home/workshop/workspace/install/setup.bash
ros2 launch so101_bringup teleop.launch.py \
  leader_usb_port:=/dev/ttyACM1 \
  follower_usb_port:=/dev/ttyACM0 \
  use_cameras:=false \
  use_teleop_rviz:=true
```

This brings up both arms + the teleop relay + a combined RViz (both arms
side-by-side).

### 3.2 Separate terminals (same result, decomposed)

```bash
# Terminal 1 — leader arm
ros2 launch so101_bringup leader.launch.py \
  hardware_type:=real usb_port:=/dev/ttyACM1 use_rviz:=false

# Terminal 2 — follower arm (forward_controller = smooth direct teleop)
ros2 launch so101_bringup follower.launch.py \
  hardware_type:=real usb_port:=/dev/ttyACM0 \
  arm_controller:=forward_controller use_rviz:=false

# Terminal 3 — static TF layout (positions the two arms in 'world')
ros2 launch so101_bringup layout_tf.launch.py

# Terminal 4 — teleop relay (leader joint_states -> follower controller)
ros2 launch so101_teleop teleop.launch.py \
  leader_namespace:=leader follower_namespace:=follower \
  arm_controller:=forward_controller

# Terminal 5 — RViz
rviz2 -d $(ros2 pkg prefix so101_bringup)/share/so101_bringup/rviz/teleop.rviz
```

---

## 4. Run: full safety-stop demo (real arms)

Leader drives follower **through a safety gate**. The overhead camera runs
Depth Anything V2; a hand/palm close to the camera raises
`/safety/protective_stop` and the gate freezes the follower.

> ⚠️ **Safety:** the protective stop is ~1 Hz CPU monocular-depth inference — a
> functional demo, **not** a real safety device (≈1 s latency, arm-trajectory
> only). Keep a physical e-stop / power cut and test clear of people first.

There are **two** interchangeable versions of this demo (run one at a time —
both drive the follower and open the same serial port):

| Launch | Leader→follower path | Protective stop mechanism | Follower controller |
| --- | --- | --- | --- |
| `full_demo_real.launch.py` | **MoveIt Servo** (`leader_servo_jog` → JointJog) | `safety_pause_bridge` pauses Servo | `arm_forward_controller` (split) |
| `full_demo_real_split.launch.py` | **Direct `teleop_split`** (absolute-position mirror) | `trajectory_safety_gate` freezes the arm | `arm_trajectory_controller` (split) |

The `_split` variant is the direct-copy teleop (no Servo) — snappier following,
no per-joint gain/lead tuning, but no Servo joint-limit/collision safety.

### 4.1 One launch (+ 2 GUI terminals)

GUI nodes (RViz, image_view) are started **separately** to avoid Qt-context
conflicts at launch time.

```bash
# Terminal 1 — everything except the GUIs
source /opt/ros/jazzy/setup.bash
source /home/workshop/workspace/install/setup.bash

# --- MoveIt Servo version ---
ros2 launch so101_depth_demo full_demo_real.launch.py \
  leader_usb_port:=/dev/ttyACM1 \
  follower_usb_port:=/dev/ttyACM0 \
  camera_device:=/dev/video4

# --- OR: direct teleop_split version (safety gate, no Servo) ---
ros2 launch so101_depth_demo full_demo_real_split.launch.py \
  leader_usb_port:=/dev/ttyACM1 \
  follower_usb_port:=/dev/ttyACM0 \
  camera_device:=/dev/video4

# Terminal 2 — depth debug overlay (green = clear, red = STOP)
source /opt/ros/jazzy/setup.bash
ros2 run image_view image_view --ros-args -r image:=/safety/depth_debug_image

# Terminal 3 — RViz (both arms)
source /opt/ros/jazzy/setup.bash
source /home/workshop/workspace/install/setup.bash
rviz2 -d $(ros2 pkg prefix so101_bringup)/share/so101_bringup/rviz/teleop.rviz
```

Verify the stop signal:

```bash
ros2 topic echo /safety/protective_stop   # flips to 'data: true' when palm detected
```

### 4.2 Separate terminals (full manual decomposition)

The follower must run `trajectory_controller` (6 joints incl. gripper) so the
gate can freeze it with a `JointTrajectory` hold. The teleop relay is pointed at
the gate input instead of straight at the controller.

```bash
# T1 — leader arm
ros2 launch so101_bringup leader.launch.py \
  hardware_type:=real usb_port:=/dev/ttyACM1 use_rviz:=false \
  controller_config_file:=$(ros2 pkg prefix so101_bringup)/share/so101_bringup/config/ros2_control/leader_controllers.yaml

# T2 — follower arm (trajectory_controller!)
ros2 launch so101_bringup follower.launch.py \
  hardware_type:=real usb_port:=/dev/ttyACM0 \
  arm_controller:=trajectory_controller use_rviz:=false \
  controller_config_file:=$(ros2 pkg prefix so101_bringup)/share/so101_bringup/config/ros2_control/follower_controllers.yaml

# T3 — static TF layout
ros2 launch so101_bringup layout_tf.launch.py

# T4 — overhead camera (C920)
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/static_camera -r __node:=cam_overhead \
  --params-file $(ros2 pkg prefix so101_bringup)/share/so101_bringup/config/cameras/so101_usb_cam.yaml \
  -p video_device:=/dev/video4 -p camera_name:=cam_overhead -p frame_id:=cam_overhead

# T5 — depth proximity -> /safety/protective_stop
ros2 run so101_depth_demo depth_proximity_node --ros-args \
  -p model_path:=$HOME/models/depth_anything_v2_small.onnx \
  -p input_image_topic:=/static_camera/image_raw \
  -p stop_topic:=/safety/protective_stop \
  -p debug_image_topic:=/safety/depth_debug_image \
  -p publish_debug_image:=true -p inference_hz:=1.0

# T6 — safety gate (intercepts teleop, freezes follower on stop)
ros2 run so101_teleop trajectory_safety_gate --ros-args \
  -p input_topic:=/safety/follower/arm_trajectory_in \
  -p output_topic:=/follower/trajectory_controller/joint_trajectory \
  -p safety_stop_topic:=/safety/protective_stop \
  -p joint_states_topic:=/follower/joint_states \
  -p "arm_joints:=[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]"

# T7 — teleop relay routed THROUGH the gate
ros2 run so101_teleop teleop --ros-args \
  --params-file $(ros2 pkg prefix so101_teleop)/share/so101_teleop/config/teleop.yaml \
  -p arm_mode:=joint_trajectory \
  -p leader_topic:=/leader/joint_states \
  -p jtc_topic:=/safety/follower/arm_trajectory_in \
  -p fwd_topic:=/follower/forward_controller/commands

# T8 — depth overlay viewer
ros2 run image_view image_view --ros-args -r image:=/safety/depth_debug_image

# T9 — RViz
rviz2 -d $(ros2 pkg prefix so101_bringup)/share/so101_bringup/rviz/teleop.rviz
```

---

## 5. Run: simulation (no hardware)

Gazebo-based, no arms/camera required.

```bash
source /opt/ros/jazzy/setup.bash
source /home/workshop/workspace/install/setup.bash
```

| Demo | Command |
| --- | --- |
| Two-arm keyboard teleop | `ros2 launch so101_bringup gazebo_teleop_sim.launch.py` |
| Leader→follower via MoveIt Servo | `ros2 launch so101_bringup sim_servo_teleop.launch.py` |
| Follower + MoveIt planning | `ros2 launch so101_bringup gazebo_follower_moveit.launch.py` |

### Depth demo in simulation

The default Gazebo world has no simulated camera, so feed the depth node either
the real C920 or a synthetic publisher, alongside the sim arms:

```bash
# Terminal 1 — sim arms
ros2 launch so101_bringup gazebo_teleop_sim.launch.py

# Terminal 2 — synthetic image (or use the real C920, see §4)
ros2 run so101_depth_demo test_image_publisher \
  --ros-args -p image_topic:=/follower/image_raw -p fps:=15.0

# Terminal 3 — depth visualization only
ros2 launch so101_depth_demo depth_demo.launch.py \
  model_path:=$HOME/models/depth_anything_v2_small.onnx \
  input_image_topic:=/follower/image_raw
```

> **[PROD TODO]** `depth_safety_stop.launch.py` (used by the *sim* `full_demo.launch.py`)
> was changed to target **real** hardware (`use_sim_time:=false`,
> `trajectory_controller`). To restore the fully-wired sim safety demo,
> parametrize `use_sim_time` and the controller name in that launch instead of
> hardcoding them.

---

## 6. workshop.yaml — provisioning actions

Current `workshop.yaml` only grants the camera plug. To make the manual setup
(§2) and hardware access reproducible, extend it. **Verify field names against
your workshop tool version** — treat the `actions`/`setup` block below as a
candidate to adapt.

```yaml
name: open-manipulator
base: ubuntu@24.04
sdks:
  - name: ros2-desktop
    channel: jazzy/stable
    plugs:
      camera:
        interface: camera
      # Arms enumerate as /dev/ttyACM* — grant serial access:
      serial-port:
        interface: serial-port
      raw-usb:
        interface: raw-usb
  - name: vscode-remote
  - name: project-tty-sdk

# Candidate provisioning actions (adapt to workshop schema).
# Run once at workshop launch/refresh, from the project dir (/project).
actions:
  setup:
    - git submodule update --init --recursive
    - sudo apt-get update
    - sudo apt-get install -y libserial-dev python3-pip
    - python3 -m pip install --break-system-packages onnxruntime
    - bash so101_depth_demo/scripts/download_model.sh
    - >-
      bash -lc 'source /opt/ros/jazzy/setup.bash &&
      cd /home/workshop/workspace &&
      colcon build --packages-select feetech_ros2_driver so101_teleop
      so101_bringup so101_depth_demo'
```

> **[PROD TODO]** Also add the user to `dialout` and install udev rules for stable
> arm symlinks (`docs/hardware.md`). In this container the `/dev/ttyACM*` nodes
> happened to be world-writable (`crw-rw-rw-`), so `dialout` wasn't strictly
> required, but a production image should not rely on that.

---

## 7. Troubleshooting

| Symptom | Cause / fix |
| --- | --- |
| `Bad file descriptor` opening `/dev/ttyACM*` | A **stale `ros2_control_node`** from a previous run still holds the port. Kill leftovers (below). Also check the leader/follower port args are correct. |
| `The 'type' param was not defined for '<controller>'` | Follower loaded the wrong controller YAML (leader's leaked in). Ensure both `leader`/`follower` includes pass their own `controller_config_file` explicitly. Fixed in `full_demo_real.launch.py`. |
| Follower follows in RViz but doesn't physically move | Servo power / wrong controller. Confirm `trajectory_controller`/`forward_controller` active and follower servos powered. |
| Safety gate never stops the arm | Gate binary out of date / topic mismatch. Rebuild `so101_teleop`; gate must subscribe to `/safety/protective_stop`. Check `ros2 topic echo /safety/protective_stop`. |
| `ModuleNotFoundError: No module named 'PyQt5' / 'PySide2'` | Qt-context conflict when multiple GUI nodes start from one launch. Run `rviz2` and `image_view` in their own terminals (defaults `use_rviz:=false`, `use_viewer:=false`). |
| `No module named 'onnxruntime'` | Install into the interpreter `ros2` uses: `python3 -m pip install --break-system-packages onnxruntime`. |
| `class ... FeetechHardwareInterface ... does not exist` | Submodule not checked out / not built. `git submodule update --init --recursive` then rebuild. |
| Camera fails: `package v4l2_camera not found` | Use `usb_cam` (installed). Already fixed in the launches. |
| Arms overlap in RViz | `layout_tf` not running or RViz Fixed Frame ≠ `world`. |

### Kill stale processes (run before relaunching)

```bash
pkill -9 -f ros2_control_node
pkill -9 -f robot_state_publisher
pkill -9 -f spawner
pkill -9 -f follower_command_relay
pkill -9 -f trajectory_safety_gate
pkill -9 -f depth_proximity
pkill -9 -f usb_cam_node
pkill -9 -f rviz2
pkill -9 -f image_view
sleep 2
```

---

## 8. Production checklist

- [ ] udev rules → stable `/dev/so101_leader` / `/dev/so101_follower`; use those as launch defaults.
- [ ] Add user to `dialout`; don't rely on world-writable tty nodes.
- [ ] Pin `onnxruntime` version; decide venv vs `--break-system-packages` and bake into the image.
- [ ] Provision §2 steps via workshop.yaml actions (verified schema).
- [ ] Parametrize `depth_safety_stop.launch.py` (`use_sim_time`, controller name) so sim + real both work.
- [ ] Commit the `so101_teleop/CMakeLists.txt` gate target and launch fixes upstream.
- [ ] Camera intrinsics: provide `cam_overhead.yaml` (currently logs "Unable to open camera calibration file").
- [ ] Consider higher `inference_hz` / GPU for a more responsive safety stop.
```
