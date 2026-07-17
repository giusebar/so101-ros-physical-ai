# SO-101 — ROS 2 Communication, Safety & Production Architecture

Design reference for the MoveIt Servo teleoperation stack, the depth-based
obstacle-avoidance (protective-stop) subsystem, and a production-ready snap
packaging with a swappable AI perception model.

Scope covered:

1. [Overview](#1-overview)
2. [Node & component inventory](#2-node--component-inventory)
3. [ROS 2 communication map — Simulation](#3-ros-2-communication-map--simulation)
4. [ROS 2 communication map — Real hardware](#4-ros-2-communication-map--real-hardware)
5. [Obstacle avoidance / protective-stop subsystem](#5-obstacle-avoidance--protective-stop-subsystem)
6. [Sim vs real: what changes](#6-sim-vs-real-what-changes)
7. [Production architecture (snaps) & swappable AI model](#7-production-architecture-snaps--swappable-ai-model)

---

## 1. Overview

The follower arm is teleoperated **through MoveIt Servo** rather than by direct
joint copy. A leader arm (physical, or the Gazebo sim leader driven by a
keyboard) publishes joint states; an adapter converts leader/follower joint
error into `control_msgs/JointJog` velocity commands; Servo turns those into a
collision-, singularity- and joint-limit-safe position stream to the follower's
`arm_forward_controller`.

A separate **perception subsystem** runs a monocular depth model (Depth Anything
V2 Small, ONNX Runtime, CPU) on a camera feed and raises a boolean
**protective-stop** signal when an object gets too close. Motion is halted while
the stop is asserted.

Design principles:

- **Single writer** to any controller command interface.
- **Namespaced** per arm (`/leader`, `/follower`) so the same nodes scale to
  multi-arm setups.
- **Stable topic contract** between perception, safety and motion so the AI
  model is replaceable without touching the motion stack.
- **Identical topic names/types** in simulation and on real hardware — only the
  bottom (hardware execution) layer changes.

---

## 2. Node & component inventory

| Component | Package | Executable | Role |
|---|---|---|---|
| Leader keyboard (sim) | `so101_teleop` | `leader_sim_keyboard` | Operator input → drives sim leader arm. Omitted on real HW. |
| **Servo adapter** | `so101_teleop` | `leader_servo_jog` | Leader/follower joint error → `JointJog`; gripper passthrough; switches Servo to `JOINT_JOG`. |
| MoveIt Servo | `moveit_servo` | `servo_node` | `JointJog`/`Twist`/`Pose` → safe position stream + status. |
| Safety gate (JTC path) | `so101_safety` | `trajectory_safety_gate` | Blocks/holds JointTrajectory stream on protective-stop. |
| Safety bridge (Servo path) | `so101_safety` | `safety_pause_bridge` | Bool → Servo `pause_servo` service. |
| Depth safety monitor | `so101_safety` | `depth_safety_monitor` | Consumes `/perception/depth` → `Bool` protective-stop + debug overlay. No ML/vision deps (pure topic consumer). |
| Depth inference | `so101_depth_demo` | `depth_anything_node` | Camera image → ONNX depth inference → colorised viz (`/camera/depth/visualization`) + raw depth (`/perception/depth`). Runs once; both `depth_safety_monitor` and viewers consume its output. |
| Controllers | `controller_manager` | `ros2_control_node` / `gz_ros_control` | Execute commands, publish joint states. |

Legacy (retired on the Servo path): `teleop_split` (direct joint-copy bridge).
It must **not** run alongside Servo — both would write the follower position
interface.

---

## 3. ROS 2 communication map — Simulation

Simulation runs a **leader + follower pair** in Gazebo (`ros_gz_sim` +
`gz_ros_control`), a shared `/clock`, and `use_sim_time:=true` everywhere.

Launch composition:

```
ros2 launch so101_bringup gazebo_teleop_sim.launch.py \
     launch_teleop:=false launch_keyboard:=false \
     follower_arm_controller:=arm_forward_controller
ros2 launch so101_moveit_config servo.launch.py use_sim_time:=true
ros2 launch so101_teleop teleop_servo.launch.py            # adapter + leader keyboard
```

### 3.1 Teleop data flow (sim)

```mermaid
flowchart TD
    KB["leader_sim_keyboard"] -->|"/leader/arm_forward_controller/commands<br/>Float64MultiArray"| LARM["/leader arm_forward_controller"]
    KB -->|"/leader/gripper_forward_controller/commands"| LGRIP["/leader gripper_forward_controller"]
    LARM --> GZL["Gazebo leader (gz_ros_control)"]
    LGRIP --> GZL
    GZL -->|"/leader/joint_states<br/>JointState @50Hz"| ADP["leader_servo_jog"]
    FJS["/follower/joint_states<br/>JointState @50Hz"] --> ADP
    ADP -->|"/follower/servo_node/delta_joint_cmds<br/>JointJog @100Hz"| SERVO["/follower/servo_node"]
    ADP -->|"/follower/gripper_controller/commands<br/>Float64MultiArray"| FGRIP["/follower gripper_controller"]
    ADP -.->|"switch_command_type (JOINT_JOG=0)"| SERVO
    FJS -.->|state monitor| SERVO
    SERVO -->|"/follower/arm_forward_controller/commands<br/>Float64MultiArray @100Hz"| FARM["/follower arm_forward_controller"]
    SERVO -->|"/follower/servo_node/status<br/>ServoStatus"| MON["monitoring"]
    FARM --> GZF["Gazebo follower (gz_ros_control)"]
    FGRIP --> GZF
    GZF --> FJS
```

### 3.2 Topic table (sim)

| Topic | Type | Publisher | Subscriber(s) | Rate |
|---|---|---|---|---|
| `/leader/arm_forward_controller/commands` | `std_msgs/Float64MultiArray` | `leader_sim_keyboard` | `/leader` arm ctrl | 50 Hz |
| `/leader/gripper_forward_controller/commands` | `std_msgs/Float64MultiArray` | `leader_sim_keyboard` | `/leader` gripper ctrl | 50 Hz |
| `/leader/joint_states` | `sensor_msgs/JointState` | `/leader/joint_state_broadcaster` | `leader_servo_jog` | 50 Hz |
| `/follower/joint_states` | `sensor_msgs/JointState` | `/follower/joint_state_broadcaster` | `leader_servo_jog`, `servo_node` | 50 Hz |
| `/follower/servo_node/delta_joint_cmds` | `control_msgs/JointJog` | `leader_servo_jog` | `servo_node` | 100 Hz |
| `/follower/arm_forward_controller/commands` | `std_msgs/Float64MultiArray` | `servo_node` | `/follower` arm ctrl | 100 Hz |
| `/follower/gripper_controller/commands` | `std_msgs/Float64MultiArray` | `leader_servo_jog` | `/follower` gripper ctrl | on-change |
| `/follower/servo_node/status` | `moveit_msgs/ServoStatus` | `servo_node` | monitoring | on-change |
| `/clock` | `rosgraph_msgs/Clock` | `ros_gz_bridge` | all (`use_sim_time`) | sim |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | both `robot_state_publisher` | Servo, RViz | — |

Arm array joint order: `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]`; gripper: `[gripper]`.

### 3.3 Services (sim & real, identical)

| Service | Type | Server | Client | Purpose |
|---|---|---|---|---|
| `/follower/servo_node/switch_command_type` | `moveit_msgs/srv/ServoCommandType` | `servo_node` | `leader_servo_jog` | Input mode. **`JOINT_JOG=0`**, `TWIST=1`, `POSE=2`. |
| `/follower/servo_node/pause_servo` | `std_srvs/srv/SetBool` | `servo_node` | safety bridge (§5) | Pause/resume Servo output. |
| `/follower/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | `controller_manager` | bringup | Activate `arm_forward_controller`. |

---

## 4. ROS 2 communication map — Real hardware

On real hardware the **motion contract is unchanged**. Differences:

- The leader is a **physical arm** publishing `/leader/joint_states` directly
  (via its own `ros2_control` hardware interface + `joint_state_broadcaster`).
  No `leader_sim_keyboard`, no Gazebo, **no `/clock`** (`use_sim_time:=false`).
- The follower is a **physical arm**; `gz_ros_control` is replaced by the
  Feetech serial hardware interface (`feetech_ros2_driver`).

Launch composition:

```
# Leader (publishes /leader/joint_states from the physical arm)
ros2 launch so101_bringup leader.launch.py hardware_type:=real usb_port:=/dev/so101_leader
# Follower on the position controller Servo drives
ros2 launch so101_bringup follower_split.launch.py \
     hardware_type:=real usb_port:=/dev/so101_follower \
     arm_controller:=arm_forward_controller
# Servo + adapter (no sim time)
ros2 launch so101_moveit_config servo.launch.py
ros2 launch so101_teleop teleop_servo.launch.py use_sim_time:=false launch_keyboard:=false
```

### 4.1 Teleop data flow (real)

```mermaid
flowchart TD
    LEADER["Physical leader arm<br/>ros2_control + Feetech HW"] -->|"/leader/joint_states<br/>JointState"| ADP["leader_servo_jog"]
    FJS["/follower/joint_states"] --> ADP
    ADP -->|"/follower/servo_node/delta_joint_cmds<br/>JointJog"| SERVO["/follower/servo_node"]
    ADP -->|"/follower/gripper_controller/commands"| FGRIP["/follower gripper_controller"]
    ADP -.->|"switch_command_type (JOINT_JOG)"| SERVO
    FJS -.->|state monitor| SERVO
    SERVO -->|"/follower/arm_forward_controller/commands"| FARM["/follower arm_forward_controller"]
    SERVO -->|"/follower/servo_node/status"| MON["monitoring / e-stop logic"]
    FARM --> HW["Physical follower arm<br/>Feetech serial HW interface"]
    FGRIP --> HW
    HW --> FJS
```

### 4.2 Delta vs simulation

| Aspect | Simulation | Real hardware |
|---|---|---|
| Leader source | `leader_sim_keyboard` → Gazebo leader | Physical leader arm |
| Follower execution | `gz_ros_control` | `feetech_ros2_driver` hardware interface |
| Time | `/clock`, `use_sim_time:=true` | System clock, `use_sim_time:=false` |
| `/leader/*_controller/commands` | present (drive sim leader) | absent (leader is backdriven/read-only) |
| Device access | none | serial/USB (`/dev/so101_leader`, `/dev/so101_follower`) |
| Everything else (topics/types) | **identical** | **identical** |

---

## 5. Obstacle avoidance / protective-stop subsystem

Perception is **the same on sim and real** — it is always a real camera (or a
synthetic publisher in a headless container) plus a CPU model. Only *how the
stop halts the arm* differs by motion path.

### 5.1 Perception → safety signal

```mermaid
flowchart LR
    CAM["usb_cam / usb-cam snap<br/>(USB camera)"] -->|"/static_camera/image_raw<br/>sensor_msgs/Image"| DA["depth_anything_node<br/>(so101_depth_demo / depthanything snap)<br/>Depth Anything V2 (ONNX, CPU)"]
    DA -->|"/perception/depth<br/>sensor_msgs/Image (32FC1)"| MON["depth_safety_monitor<br/>(so101_safety)"]
    DA -->|"/camera/depth/visualization<br/>sensor_msgs/Image (rgb8)"| VIZ["rqt_image_view (colorised depth)"]
    MON -->|"/safety/protective_stop<br/>std_msgs/Bool"| GATE["safety enforcement (so101_safety)"]
    MON -->|"/safety/depth_debug_image<br/>sensor_msgs/Image (mono8)"| VIEW["image_view (ROI overlay)"]
```

The model runs **once**: `depth_anything_node` (packaged standalone as the
`depthanything` snap) does the ONNX inference and publishes both the
colorised visualisation and the raw normalised depth. `depth_safety_monitor`
(in `so101_safety`) is a **pure topic consumer** — no ONNX/OpenCV dependency
— that does the ROI/background-reference/hysteresis logic on the already-computed
depth map and publishes `/safety/protective_stop`. This is a deliberate
separation of concerns: `so101_depth_demo` only ever does perception/inference;
`so101_safety` owns deciding *and* enforcing protective stops (and is the home
for future safety behaviours, e.g. diagnostics/watchdogs).

`depth_safety_monitor` logic: compare a center ROI against the surrounding
background depth; if the ROI is clearly *nearer* than the scene over
`frames_to_block` consecutive frames, assert `True`; release after
`frames_to_clear` clear frames (hysteresis). Depth Anything gives *relative*
depth, so triggering is relative-to-background, not an absolute distance.

### 5.2 Perception topics & key parameters

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `<input_image_topic>` (e.g. `/static_camera/image_raw`) | `sensor_msgs/Image` | in (to `depth_anything_node`) | rgb8/bgr8/mono8 |
| `/perception/depth` | `sensor_msgs/Image` (32FC1) | out (`depth_anything_node`) / in (`depth_safety_monitor`) | normalised `[0,1]`, higher = closer |
| `/camera/depth/visualization` | `sensor_msgs/Image` (rgb8) | out (`depth_anything_node`) | colorised depth, for viewing only |
| `/safety/protective_stop` | `std_msgs/Bool` | out (`depth_safety_monitor`) | `True` = stop requested |
| `/safety/depth_debug_image` | `sensor_msgs/Image` (mono8) | out (`depth_safety_monitor`) | depth + ROI marker (optional) |

| Parameter | Node | Default | Meaning |
|---|---|---|---|
| `model_path` | `depth_anything_node` | `~/models/depth_anything_v2_small.onnx` | **AI model swap point** (any Depth Anything V2 ONNX) |
| `roi` | `depth_safety_monitor` | `0.25,0.2,0.75,0.85` | normalised center region checked |
| `near_margin` | `depth_safety_monitor` | `0.15` | how much nearer than background counts as "near" |
| `min_area_ratio` | `depth_safety_monitor` | `0.12` | fraction of ROI near → candidate stop |
| `frames_to_block` / `frames_to_clear` | `depth_safety_monitor` | `2` / `3` | hysteresis |
| `monitor_hz` | `depth_safety_monitor` | `10.0` | processing rate (cheap now — no inference here) |
| `intra_op_threads` | `depth_anything_node` | `0` | ONNX Runtime threads (0 = default) |

### 5.3 Two ways to halt the arm

**A. JointTrajectory path (existing, JTC-based) — `trajectory_safety_gate`
(`so101_safety`).**
An in-line gate sits between a teleop bridge and the JTC. It forwards
trajectories only while clear; on stop it blocks the stream and publishes a
*hold* trajectory (current follower positions) so the arm freezes in place.

| Topic | Type | Dir |
|---|---|---|
| `/safety/follower/arm_trajectory_in` | `trajectory_msgs/JointTrajectory` | in (from teleop) |
| `/safety/protective_stop` | `std_msgs/Bool` | in (gate condition) |
| `/follower/joint_states` | `sensor_msgs/JointState` | in (hold pose) |
| `/follower/arm_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | out (to JTC) |

This path requires the **JTC** (`arm_trajectory_controller`) and does **not**
apply to the Servo path. Brought up via `so101_safety`'s
`safety_stop_jtc.launch.py` (monitor + gate together).

**B. Servo path (recommended with this stack) — `safety_pause_bridge`
(`so101_safety`) → `pause_servo`.**
Servo already owns the follower and stops smoothly. A tiny bridge subscribes
`/safety/protective_stop` and calls the Servo pause service — no trajectory
interception needed:

```mermaid
flowchart LR
    MON["/safety/protective_stop (Bool)"] --> BR["safety_pause_bridge (so101_safety)"]
    BR -->|"pause_servo: SetBool(true/false)"| SERVO["/follower/servo_node<br/>pause_servo"]
    SERVO -->|"halts /follower/arm_forward_controller/commands"| ARM["follower arm"]
```

| Interface | Type | Notes |
|---|---|---|
| `/safety/protective_stop` | `std_msgs/Bool` | in |
| `/follower/servo_node/pause_servo` | `std_srvs/srv/SetBool` | out (call `true` on stop, `false` on clear) |

Brought up via `so101_safety`'s `safety_stop_servo.launch.py` (monitor +
bridge together). This is the clean, Servo-native replacement for
`trajectory_safety_gate` on the forward-controller path.

### 5.4 Running the demo

- **Sim (arm simulated, real USB camera):** `full_demo_sim.launch.py` starts
  the Gazebo sim + teleop + `so101_safety`'s `safety_stop_jtc.launch.py`
  (assumes the depth topic is already published externally; pass
  `launch_depth:=true` to bring up the camera + `depth_anything_node` inline
  instead). For the **Servo** variant, run the §3 sim stack plus
  `safety_stop_servo.launch.py` instead of the JTC gate.
- **Headless container (no camera):** replace `usb_cam` with
  `test_image_publisher` (publishes `/follower/image_raw` / configurable) so
  the pipeline still runs.
- **Real hardware:** identical perception; motion via the §4 real stack. Put a
  hand in front of the camera → ROI turns red → `/safety/protective_stop=true`
  → Servo pauses → arm freezes; remove hand → resume.

---

## 6. Sim vs real: what changes

| Layer | Simulation | Real hardware |
|---|---|---|
| Arm execution | Gazebo `gz_ros_control` (leader + follower) | Feetech serial HW interface per arm |
| Leader input | `leader_sim_keyboard` → sim leader | Physical backdriven leader arm |
| Clock | `/clock` from `ros_gz_bridge` | System clock |
| Camera | Real USB cam or `test_image_publisher` | Real USB cam |
| Perception / safety | **Identical** | **Identical** |
| Motion topics/services/types | **Identical** | **Identical** |
| Device permissions | none | serial/USB, camera nodes |

The invariant: **the ROS 2 graph contract (names + types) is the same**; sim and
real differ only at the hardware-execution layer and the input source.

---

## 7. Production architecture (snaps) & swappable AI model

Package the stack as **strictly-confined snaps** that share one ROS 2 graph
(`ROS_DOMAIN_ID`, DDS over the `network`/`network-bind` interfaces). This mirrors
the existing `depthanything` + `depthanything-model` split (snap **content
interface** for weights) already in this repo, generalised to the whole system.

### 7.1 Snap decomposition

```mermaid
flowchart TB
    subgraph HW["so101-hardware (snap)"]
        RC["ros2_control + Feetech HW ifaces<br/>joint_state_broadcaster<br/>arm_forward / gripper controllers"]
    end
    subgraph MOT["so101-motion (snap)"]
        SV["moveit_servo servo_node<br/>leader_servo_jog adapter<br/>(optional move_group)"]
    end
    subgraph PER["so101-perception (depthanything snap, EXISTS TODAY)"]
        DA["depth_anything_node<br/>ONNX Runtime (CPU/GPU EP)"]
    end
    subgraph MODEL["depthanything-model (snap, content, EXISTS TODAY)"]
        W["ONNX weights @ /models"]
    end
    subgraph SAFE["so101-safety (package exists today; snap not yet built)"]
        MON["depth_safety_monitor<br/>(pure topic consumer, no ML/vision deps)"]
        GATE["trajectory_safety_gate"]
        SB["safety_pause_bridge"]
    end
    subgraph UI["so101-teleop-ui (snap)"]
        KBD["keyboard / joystick input"]
    end

    MODEL -. content interface: $SNAP/models .-> PER
    UI -->|leader input| MOT
    HW <-->|"joint_states / commands (DDS)"| MOT
    PER -->|"/perception/depth (sensor_msgs/Image, 32FC1)"| SAFE
    MON -->|"/safety/protective_stop"| GATE
    MON -->|"/safety/protective_stop"| SB
    SB -->|"pause_servo"| MOT
    GATE -->|"JointTrajectory (JTC path)"| HW
    CAM["camera-snap / usb-cam (camera interface)"] --> PER
```

| Snap | Contents | Key snap interfaces | Status |
|---|---|---|---|
| `so101-hardware` | bringup, `ros2_control`, Feetech driver, controllers, URDF | `raw-usb`, `serial-port`, `network`, `network-bind` | not yet snapped |
| `so101-motion` | Servo, `leader_servo_jog`, MoveIt config | `network`, `network-bind` | not yet snapped |
| `depthanything` (= `so101-perception`) | camera-fed ONNX inference (`depth_anything_node`), publishes both `/camera/depth/visualization` (viz) and `/perception/depth` (machine contract) | `network`, `network-bind`, **content plug** for model | **exists** — `so101_depth_demo/snap/snapcraft.yaml` |
| `depthanything-model` (= `so101-model`) | ONNX weights only, exposed via `content` slot | `content` slot | **exists** — `snaps/depthanything-model/snapcraft.yaml` |
| `so101-safety` | `depth_safety_monitor` (trigger) + `trajectory_safety_gate` + `safety_pause_bridge` (enforcement) — all consumers of the perception contract below, no ML/vision deps | `network`, `network-bind` | package exists (`so101_safety`), snap not yet built — should be simpler than `depthanything`'s (no pip/BLAS-LAPACK step) |
| `so101-teleop-ui` | keyboard / joystick input | `joystick`, `network`, `network-bind` | not yet snapped |

Note the `so101-safety` grouping deliberately diverges from an earlier sketch
of this diagram that split perception-trigger and enforcement into separate
snaps: keeping the trigger + enforcement + future diagnostics together in one
package/snap is simpler to reason about and matches how the code is actually
organised today (see `so101_safety/`).

Benefits: independent release cadence, least-privilege confinement (only the
hardware snap touches serial/USB, only perception touches the camera),
over-the-air updates per component, and horizontal scaling (add a second arm =
second `so101-hardware` instance under a new namespace).

### 7.2 Swappable AI model — the contract

The model is replaceable at **three levels**, from cheapest to most flexible:

1. **Swap the weights file (same architecture).** Point `model_path` at another
   Depth Anything V2 ONNX, or reship the `so101-model` content snap. No app
   rebuild — snapd remounts the new weights at `$SNAP/models`. This is the
   existing `default-provider` pattern:
   ```
   snap install so101-perception   # pulls so101-model, auto-connects content
   snap refresh so101-model         # ship new weights independently
   ```

2. **Swap the execution provider (same model, different accelerator).** ONNX
   Runtime already abstracts the backend. On a Jetson Orin, select
   `CUDAExecutionProvider` / `TensorRTExecutionProvider`; on a Pi/CPU, keep
   `CPUExecutionProvider`. Expose an `execution_provider` parameter so the
   perception snap is portable across x86, Orin and Pi5 without code changes.

3. **Swap the whole perception snap (different model *family*).** Because
   downstream consumers depend only on the **topic contract**, not on the model,
   any node that honours it is a drop-in replacement:

   > **Perception contract (stable API, implemented today):**
   > - **in:** `sensor_msgs/Image` on `<input_image_topic>`
   > - **out:** `sensor_msgs/Image` (32FC1) on `/perception/depth` — normalised
   >   `[0,1]` depth, higher = closer
   > - **out (optional):** `sensor_msgs/Image` (rgb8) colorised visualisation
   >
   > Downstream, `so101_safety`'s `depth_safety_monitor` turns `/perception/depth`
   > into `std_msgs/Bool` on `/safety/protective_stop` — it never touches the
   > model or the camera, so any perception node honouring the contract above
   > is a drop-in replacement without touching `so101_safety` at all.
   >
   > (future) additional structured output on a versioned topic, e.g.
   > `/perception/obstacles` (`vision_msgs/Detection2DArray`), for richer
   > avoidance beyond a single Bool.

   This lets you replace Depth Anything with a stereo-depth node, a 2D object
   detector, a segmentation model, or a learned VLA policy — the motion and
   safety snaps are untouched.

### 7.3 Recommended stable interface topics (target state)

| Topic | Type | Producer | Consumer | Purpose |
|---|---|---|---|---|
| `/perception/depth` | `sensor_msgs/Image` | perception snap | avoidance / viz | dense depth |
| `/perception/obstacles` | `vision_msgs/Detection2DArray` | perception snap | avoidance | structured detections |
| `/safety/protective_stop` | `std_msgs/Bool` | safety logic | motion (`pause_servo`) | binary halt |
| `/follower/servo_node/delta_joint_cmds` | `control_msgs/JointJog` | teleop / policy | Servo | motion command |
| `/follower/servo_node/status` | `moveit_msgs/ServoStatus` | Servo | monitoring / HMI | health / limit state |

Keeping perception behind `/perception/*` and reducing to `/safety/protective_stop`
means the AI model, the safety policy, and the motion controller each evolve
independently behind versioned ROS 2 contracts.

---

### Appendix — enum & scaling gotchas

- `moveit_msgs/srv/ServoCommandType`: **`JOINT_JOG=0`**, `TWIST=1`, `POSE=2`.
  Servo silently ignores `JointJog` until switched to mode `0`.
- `JointJog.header.stamp` **must** be `now()`; Servo drops commands older than
  `incoming_command_timeout` (0.1 s) and smooth-halts.
- Unitless `JointJog`: joint speed ≈ `velocity · scale.joint · publish_period`
  (`scale.joint = 1.5`, `publish_period = 0.01 s`). Adapter gain `kp` (default
  10) sets how fast error saturates the command.
- Only one node may publish `/follower/arm_forward_controller/commands` — Servo.
  Do not co-run `teleop_split` on the Servo path.
