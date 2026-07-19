# SO-101 ROS 2 Physical-AI Project — Agent Context

## What this project is

A ROS 2 (Jazzy) stack for a **two-arm SO-101 leader/follower teleoperation**
demo plus physical-AI tooling (episode recording, LeRobot dataset conversion,
policy inference, a monocular-depth safety stop). The **leader** arm is moved by
hand and the **follower** mirrors it. Arms use Feetech STS3215 servos driven by
`feetech_ros2_driver` (a ros2_control hardware interface).

There are two mirroring implementations, kept side by side:
- **Direct teleop** (`teleop_split`): copies the leader's absolute joint
  positions to the follower. Snappy, no IK, no safety envelope. This is the
  original, reliable path.
- **MoveIt Servo** (`leader_servo_jog` → JointJog): drives the follower through
  MoveIt Servo for joint-limit/collision/singularity safety. Newer, still being
  tuned.

## Environment (how to run anything)

Everything runs inside the **`open-manipulator` workshop container**, not the
host. From the host:
- Interactive: `workshop shell open-manipulator`
- One-off: `workshop exec open-manipulator -- bash -lc '<cmd>'`

Inside the container:
- ROS: `source /opt/ros/jazzy/setup.bash`
- Overlay: `source /home/workshop/workspace/install/setup.bash`
- Colcon workspace: `/home/workshop/workspace` (its `src/so101-ros-physical-ai`
  is a **symlink to `/project`**, which is this repo — edits here are seen there
  immediately; you still must `colcon build` to install configs/launches/libs).
- Build: `cd /home/workshop/workspace && colcon build --packages-select <pkg>`

Hardware (device numbering is NOT stable across replug):
- **Leader** arm: `/dev/ttyACM1` (state-only, no command interfaces)
- **Follower** arm: `/dev/ttyACM0` (position-commanded)
- **Overhead camera** (C920): `/dev/video4`
- Depth model: `~/models/depth_anything_v2_small.onnx`

⚠️ **Hardware safety:** restarting the follower `ros2_control_node` runs the
driver's `on_deactivate`, which **torque-offs all follower joints** → the arm
goes limp and can drop under gravity. Warn the user / have them support the arm
before any follower restart. Prefer a physical e-stop nearby.

## Package map

| Package | Role |
| --- | --- |
| `so101_description` | URDF/xacro, meshes. `variant:=leader|follower`, `hardware_type:=mock|real|gazebo`. |
| `so101_moveit_config` | MoveIt + **Servo** config: `so101_servo.yaml`, `kinematics.yaml` (`pick_ik`), `joint_limits.yaml`, SRDF, `servo.launch.py`. |
| `so101_bringup` | Launch orchestration, ros2_control controller YAMLs, hardware joint configs, cameras, RViz. |
| `so101_teleop` | Teleop C++ nodes: `teleop_split`, `teleop`, `leader_servo_jog`, `leader_sim_keyboard`. (Safety nodes `trajectory_safety_gate`/`safety_pause_bridge` moved to `so101_safety`.) |
| `so101_safety` | Safety subsystem: on this branch, **enforcement-only** — `trajectory_safety_gate`/`safety_pause_bridge` (C++, moved from `so101_teleop`), pure `ament_cmake`, no Python/ML deps. Both just subscribe to `/safety/protective_stop` (`std_msgs/Bool`) and don't care who published it or why. The perception-driven trigger logic (ROI/thresholds/hysteresis) that used to live here as separate `depth_safety_monitor`/`person_safety_monitor` nodes has moved directly into `so101_depth_demo`'s `depth_anything_node` / `so101_yolo_demo`'s `yolo_detect_node` — see `docs/ai_vision_ros2_channel_demo.md` for why (enables a live `snap refresh --channel=...` swap with zero ROS-side restart). Home for future safety behaviours (diagnostics, watchdogs, ...). |
| `so101_depth_demo` | Depth-Anything inference (`depth_anything_node`): publishes colorized viz (`/camera/depth/visualization`, with the safety ROI + trigger state drawn on it) AND raw normalised depth (`/perception/depth`, 32FC1) AND, on this branch, computes and publishes `/safety/protective_stop` directly itself (ROI proximity + hysteresis debounce). |
| `so101_yolo_demo` | YOLOv8n (nano) object/person detection (`yolo_detect_node`, NMS baked into the exported ONNX graph): publishes annotated viz (`/camera/detections/visualization`, with the safety ROI + trigger state drawn on it) AND `vision_msgs/Detection2DArray` (`/perception/detections`) AND, on this branch, computes and publishes `/safety/protective_stop` directly itself (ROI overlap on the already class-filtered detections + hysteresis debounce). The "swap the model, do something different" sibling of `so101_depth_demo` — same CPU/ONNX Runtime pattern and camera input, different recognition task. |
| `feetech_ros2_driver` | Feetech STS ros2_control hardware interface (git submodule). |
| `so101_camera_calibration` | Offline hand-eye calibration (the ONLY consumer of `so101_kinematics`). |
| `so101_kinematics` / `_msgs` | **Legacy** custom IK (robokin/Placo/Viser). **Not used by any demo.** MoveIt/Servo get kinematics from `so101_moveit_config`. |
| `episode_recorder`, `rosbag_to_lerobot`, `so101_inference`, `policy_server` | Physical-AI data/inference tooling. |
| `snap-usb-cam` | Standalone strict-confinement snap packaging upstream `ros-drivers/usb_cam` (production alternative to the apt `usb_cam` dep). Configured via `snap set usb-cam device=... frame-id=... camera-name=... namespace=...`, not a params YAML. See `snap-usb-cam/snapcraft.yaml`. |
| `depthanything` + `depthanything-model` | **(legacy, superseded on this branch by `ai-vision-ros2`, see below)** Strict-confinement snap pair for `depth_anything_node` (inference only, no safety logic). `depthanything` is an **always-on daemon** (like `usb-cam`) built from `so101_depth_demo/snap/snapcraft.yaml`, configured via `snap set depthanything input-image-topic=... output-image-topic=... output-depth-topic=...`. `depthanything-model` is a content-only snap shipping the ONNX weights, mounted at `$SNAP/models`. See `docs/depthanything_usbcam_setup.md` for the full setup/verify walkthrough. |
| `yolodetect` + `yolodetect-model` | **(legacy, superseded on this branch by `ai-vision-ros2`, see below)** Strict-confinement snap pair for `yolo_detect_node` (inference only, no safety logic) — mirrors `depthanything`/`depthanything-model`'s conventions exactly. `yolodetect` is an **always-on daemon** built from `so101_yolo_demo/snap/snapcraft.yaml`, configured via `snap set yolodetect input-image-topic=... output-image-topic=... output-detections-topic=... class-filter=... conf-threshold=...`. `yolodetect-model` is a content-only snap shipping the ONNX weights (vendored in-repo, not downloaded at build time), mounted at `$SNAP/models`. See `docs/yolodetect_setup.md` for the full setup/verify/swap walkthrough. |
| `ai-vision-ros2` | **One snap name, two independently-built variants published on different channels** — the "swap the model via `snap refresh --channel`" demo. `latest/stable` (built from `so101_depth_demo/snap/snapcraft.yaml`) = depth proximity; `latest/edge` (built from `so101_yolo_demo/snap/snapcraft.yaml`) = YOLOv8n person detection. Both bundle their ONNX weights directly in the snap at build time (no content interface, no separate model snap, no `default-provider` approval wait), and share the same app/service name (`perception`) so `snap services ai-vision-ros2` stays consistent across a channel swap. See `docs/ai_vision_ros2_channel_demo.md` for the full build/publish/swap walkthrough. Requires the Store (can be unlisted/private) — `--dangerous` local installs don't support channels. |

Joint order everywhere: `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll]` (+ `gripper` handled separately). Namespaces: `/leader`, `/follower`.

## The two real-hardware demos (mutually exclusive)

Both bring up leader + follower + overhead camera + protective-stop + RViz.
Run one at a time (they share the follower and serial port). By default both
assume the camera + AI perception snap are already running externally (the
`ai-vision-ros2` + `usb-cam` snaps, per `docs/ai_vision_ros2_channel_demo.md`);
pass `launch_depth:=true` to bring the camera + perception node up inline
instead. On this branch, `/safety/protective_stop` is published DIRECTLY by
whichever AI perception node is running (`depth_anything_node` or
`yolo_detect_node`, each with their own trigger logic baked in) — so
swapping AI backends is just `snap refresh ai-vision-ros2
--channel=stable|edge`, with **zero restart of this launch file needed**.
`perception_backend:=depth|detection` (default `depth`) only still matters
for `launch_depth:=true` (which node to bring up inline) and which viz
topic `use_viewer` opens — it has nothing to do with safety-monitor
selection any more (there is no separate monitor; see
`docs/ai_vision_ros2_channel_demo.md`).

- **Servo:** `ros2 launch so101_depth_demo full_demo_real_servo.launch.py`
  - `leader_servo_jog` (P-controller on joint error) → `/follower/servo_node/delta_joint_cmds` (JointJog) → `servo_node` → `arm_forward_controller`.
  - `so101_safety`'s `safety_pause_bridge`: `/safety/protective_stop` → Servo pause.
  - Follower controllers: `follower_split_controllers.yaml`, `arm_controller:=arm_forward_controller`.
- **Split (direct):** `ros2 launch so101_depth_demo full_demo_real_split.launch.py`
  - `teleop_split` (absolute-position mirror, `arm_mode:=joint_trajectory`) → `so101_safety`'s `trajectory_safety_gate` → `arm_trajectory_controller`.
  - Gate freezes the arm (trajectory hold) on protective stop.

Typical args: `leader_usb_port:=/dev/ttyACM1 follower_usb_port:=/dev/ttyACM0`
(add `launch_depth:=true camera_device:=/dev/video4` if not using the snaps).
RViz/image_view are usually run in their own terminals (Qt conflicts). See
`docs/demo_runbook.md` for the full runbook.

## Gotchas / decisions already made (don't relitigate without reason)

- **Servo follows slowly / elbow stalls:** MoveIt Servo is closed-loop on the
  *measured* follower state, so it only ever commands ~one tick ahead of actual.
  Combined with the Feetech's weak position gain (`p_coefficient=16`,
  `d_coefficient=32`), small steps don't overcome gravity (worst on `elbow_flex`).
  Mitigated by `scale.joint: 3.0` in `so101_servo.yaml`. Proper fix = raise
  Feetech P / lower D in `so101_bringup/config/hardware/follower_joints.yaml`
  (EEPROM-written at driver configure; needs follower restart).
- **Gripper:** `gripper_controller` is a `forward_command_controller/`
  `ForwardCommandController` (a `/follower/gripper_controller/commands`
  `Float64MultiArray` topic). Both demos forward the leader gripper *position*
  there (`leader_servo_jog` directly; `teleop_split` with
  `gripper_mode:=forward_position`). It is NOT a GripperActionController.
- **Camera calibration** (`config/cameras/calibrations/cam_overhead.yaml`) is
  **not used** by the demos — `so101_usb_cam.yaml` sets no `camera_info_url`, and
  the depth node consumes only the raw image.
- Servo runs in **JOINT_JOG** mode (pure joint space) — no IK/Jacobian in play
  for the mirror.
- **`arm_forward_controller` vs `arm_trajectory_controller`:** both are defined
  in `follower_split_controllers.yaml` but only one is spawned/used per demo.
  - `arm_forward_controller` (`forward_command_controller/ForwardCommandController`,
    `interface_name: position`) is a dumb pipe: whatever position array arrives
    on its `/commands` topic is written straight to hardware every cycle, no
    interpolation/timing/action API. Used by the **Servo** demo because Servo
    itself already does the interpolation/rate-limiting/collision-checking.
  - `arm_trajectory_controller` (`joint_trajectory_controller/JointTrajectoryController`)
    takes `JointTrajectory`/`FollowJointTrajectory` goals, interpolates between
    waypoints, respects `joint_limits.yaml`, and exposes an action API (so
    `trajectory_safety_gate` can hold/freeze the last goal on protective stop).
    Used by the **split** demo since `teleop_split` has no safety layer of its
    own and needs the controller to provide smoothing + a "freeze" mechanism.
- **Servo "velocity" input is converted to position output**, because the
  Feetech firmware/`arm_forward_controller` only accept position writes:
  `leader_servo_jog` publishes unitless `JointJog` (`command_in_type: unitless`)
  → `servo_node` integrates each tick (`cmd * scale.joint * publish_period`)
  against the *measured* follower position, runs it through the Butterworth
  smoother + collision/singularity checks, then emits an **absolute position**
  (`publish_joint_positions: true`, `publish_joint_velocities: false` in
  `so101_servo.yaml`) on `/follower/arm_forward_controller/commands`. Servo is
  effectively the integrator/interpolator standing in for what the JTC would
  normally do.
- **One depth node, three outputs, and it now decides the safety trigger
  itself (was two nodes with duplicate inference, then depth-node +
  separate `so101_safety` monitor):** `depth_anything_node.py` runs ONNX
  inference once and publishes `/camera/depth/visualization` (rgb8,
  colorized, with the safety ROI + trigger state drawn on it),
  `/perception/depth` (32FC1, normalised `[0,1]`, higher = closer, for other
  machine consumers), AND `/safety/protective_stop` (`std_msgs/Bool`) --
  computed directly from the same depth map, in the same node, no separate
  monitor involved. It crops a center ROI, computes a **background
  reference** from the median depth *outside* the ROI (the model output is
  only relative/per-frame, no absolute scale), flags "near" pixels vs.
  background+margin, debounces with hysteresis (`frames_to_block`/
  `frames_to_clear`), and publishes the result straight to
  `/safety/protective_stop`. `yolo_detect_node.py` follows the identical
  pattern with ROI-overlap on its already class-filtered detections instead.
  Feeds `safety_pause_bridge` (Servo demo) or `trajectory_safety_gate` (split
  demo), both now perception-agnostic, enforcement-only nodes in
  `so101_safety` -- see `docs/ai_vision_ros2_channel_demo.md` for why this
  moved (enables a live `snap refresh --channel=...` swap with zero
  ROS-side restart).
- **Strict-confinement snap + `opencv-python-headless` = missing BLAS/LAPACK
  at runtime.** `cv2` `dlopen()`s `libblas.so.3`/`liblapack.so.3`, which
  aren't on the default library search path inside a strict snap. The
  crash-loop symptom is a *misleading* `ImportError: ... you should not try
  to import numpy from its source directory` — the real cause, a few lines
  above in `snap logs`, is `libblas.so.3: cannot open shared object file`.
  Fix: stage `liblapack3`/`libblas3` and export `LD_LIBRARY_PATH` to
  `usr/lib/<triplet>/{lapack,blas}` in the launcher (see
  `so101_depth_demo/snap/local/depthanything-launch`; `usb-cam` needed the
  identical fix for `cv_bridge`).
- **Content-interface `default-provider` breaks local snap builds.**
  Snapcraft resolves a content plug's `default-provider` as a build-snap and
  tries to `snap install` it from the store during the build — this fails
  until that snap is actually published. Workaround for local test builds:
  temporarily strip `default-provider:` from `snapcraft.yaml`, build, restore
  it before committing/publishing.

## Live debugging tips

- `ros2 control` CLI is NOT installed; query the controller_manager service
  directly (`/follower/controller_manager/list_controllers`).
- Servo health: `/follower/servo_node/status` (`code: 0` = OK).
- Watch the mirror: compare `/leader/joint_states` vs `/follower/joint_states`,
  and `/follower/servo_node/delta_joint_cmds` /
  `/follower/arm_forward_controller/commands`.
- Verifying `usb-cam` / `depthanything` snaps: `snap services <name>` +
  `sudo snap logs <name> -n 30` first; if `ros2 topic hz` on the depth output
  shows nothing, check `ros2 topic info <input-image-topic> --verbose` for
  `Publisher count: 0` before assuming a code bug — it's almost always a
  topic/namespace mismatch between independently-`snap set`-configured snaps
  (each snap's default topic name is not guaranteed to match another's
  default). See `docs/depthanything_usbcam_setup.md`.
