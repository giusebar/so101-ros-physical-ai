"""Full SO-101 depth/detection safety-stop demo on REAL hardware, via MoveIt Servo.

Real-hardware counterpart to ``full_demo_sim.launch.py`` (Gazebo), and the
MoveIt-Servo counterpart to ``full_demo_real_split.launch.py``. Routes the
leader -> follower mirror through MoveIt Servo instead of a plain
JointTrajectory relay, matching the architecture used by ``so101_bringup``
``sim_servo_teleop.launch.py`` in simulation. Brings up, in order:

  1. Real leader + follower arms (``so101_bringup`` ``leader.launch.py`` /
     ``follower_split.launch.py``, ``hardware_type:=real``). The follower runs
     the split controllers (``arm_forward_controller`` for the 5 arm joints +
     ``gripper_controller`` for the gripper) so MoveIt Servo can stream
     position setpoints to it.
  2. The Servo node (``so101_moveit_config`` ``servo.launch.py``) in the
     ``/follower`` namespace, plus the ``leader_servo_jog`` adapter
     (``so101_teleop``) that reads ``/leader/joint_states`` +
     ``/follower/joint_states`` and drives Servo with JointJog commands
     (gripper forwarded directly, since it is outside the Servo move group).
  3. ``so101_safety``'s ``safety_stop_servo.launch.py``: just
     ``safety_pause_bridge``, which pauses/resumes Servo whenever
     ``/safety/protective_stop`` is asserted (Servo's own collision/joint-limit
     checking supersedes the ``trajectory_safety_gate`` used on the
     JointTrajectory path). This node is perception-agnostic -- it doesn't
     care WHO publishes the stop signal.
  4. The camera + AI perception, publishing ``/safety/protective_stop``
     directly (the "ai-vision-ros2" single-snap architecture: each AI
     backend's own node computes its own trigger logic and publishes this
     topic itself -- see so101_depth_demo's depth_anything_node /
     so101_yolo_demo's yolo_detect_node). By default this is assumed to be
     running EXTERNALLY (the ``ai-vision-ros2`` + ``usb-cam`` snaps, per
     ``docs/ai_vision_ros2_channel_demo.md``) -- set ``launch_depth:=true`` to
     bring it up inline instead (useful for sim/dev without the snap).

  Since the protective-stop trigger logic now lives entirely inside whichever
  AI perception node is running, swapping AI backends via
  ``snap refresh ai-vision-ros2 --channel=stable|edge`` changes the ENTIRE
  safety behaviour with NO restart of this launch file needed at all.
  ``perception_backend`` only still matters for ``launch_depth:=true``
  (which node to bring up inline) and which viz topic ``use_viewer`` opens --
  it has nothing to do with safety-monitor selection any more (there is no
  separate monitor).

Move the REAL leader arm by hand to drive the follower. Put your palm in front
of the camera (depth backend) or step into frame (detection backend) to raise
/safety/protective_stop; Servo then halts (and later resumes) the follower
until the obstacle/person clears.

  ###########################################################################
  #  SAFETY WARNING                                                          #
  #  This "protective stop" is EXPERIMENTAL CPU perception at ~1-8 Hz. It   #
  #  is NOT a functional-safety system: expect up to ~1 s of reaction        #
  #  latency, and it only pauses MoveIt Servo. Keep a physical e-stop /      #
  #  power cut as the real safety mechanism. Test with the arm clear of      #
  #  people first.                                                          #
  ###########################################################################

Run (ai-vision-ros2/usb-cam snaps already running, the default):
  ros2 launch so101_depth_demo full_demo_real_servo.launch.py

Run (bring up camera + depth model inline instead of via the snap):
  ros2 launch so101_depth_demo full_demo_real_servo.launch.py \
    launch_depth:=true camera_device:=/dev/cam_overhead

Run (detection backend inline, for dev/testing without the snap):
  ros2 launch so101_depth_demo full_demo_real_servo.launch.py \
    perception_backend:=detection launch_depth:=true

Prerequisites:
  - LeRobot motor setup + calibration done on both arms (EEPROM written).
  - udev symlinks /dev/so101_leader, /dev/so101_follower (or override the
    *_usb_port args), and the user in the `dialout` group.
  - If launch_depth:=true: ONNX model at ~/models/depth_anything_v2_small.onnx
    (so101_depth_demo/scripts/download_model.sh, or ~/models/yolov8n.onnx for
    perception_backend:=detection) and onnxruntime installed in the
    interpreter ros2 uses. Otherwise, the ai-vision-ros2 snap already bundles
    this (whichever channel you've installed).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    leader_usb = LaunchConfiguration("leader_usb_port")
    follower_usb = LaunchConfiguration("follower_usb_port")
    camera_device = LaunchConfiguration("camera_device")
    perception_backend = LaunchConfiguration("perception_backend")
    is_depth_backend = PythonExpression(["'", perception_backend, "' == 'depth'"])
    is_detection_backend = PythonExpression(["'", perception_backend, "' == 'detection'"])
    model_path = LaunchConfiguration("model_path")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    safety_stop_topic = LaunchConfiguration("safety_stop_topic")
    depth_viz_topic = LaunchConfiguration("depth_viz_topic")
    detections_viz_topic = LaunchConfiguration("detections_viz_topic")
    image_topic = LaunchConfiguration("image_topic")
    use_viewer = LaunchConfiguration("use_viewer")
    launch_depth = LaunchConfiguration("launch_depth")
    # NOTE: deliberately NOT named "use_rviz" - leader.launch.py and
    # follower_split.launch.py both declare a launch argument with that exact
    # name (each hardcoded to "false" below so they don't pop their own RViz
    # windows). ROS 2 launch configurations are not scoped per
    # IncludeLaunchDescription, so reusing "use_rviz" here would silently be
    # overwritten to "false" by the time this combined-view RViz node's
    # condition is evaluated.
    use_teleop_rviz = LaunchConfiguration("use_teleop_rviz")
    kp = LaunchConfiguration("kp")
    servo_start_delay = LaunchConfiguration("servo_start_delay")
    teleop_start_delay = LaunchConfiguration("teleop_start_delay")

    bringup_share = get_package_share_directory("so101_bringup")

    # --- 1. Real arms ---------------------------------------------------
    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "leader.launch.py")
        ),
        launch_arguments={
            "namespace": "leader",
            "hardware_type": "real",
            "usb_port": leader_usb,
            "frame_prefix": "leader/",
            "use_rviz": "false",
            "controller_config_file": os.path.join(
                bringup_share, "config", "ros2_control", "leader_controllers.yaml"
            ),
        }.items(),
    )

    # Split controllers (arm_forward_controller + gripper_controller): Servo
    # streams position setpoints to arm_forward_controller, while the gripper
    # is forwarded directly by leader_servo_jog since it is outside the Servo
    # move group.
    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "follower_split.launch.py")
        ),
        launch_arguments={
            "namespace": "follower",
            "hardware_type": "real",
            "usb_port": follower_usb,
            "frame_prefix": "follower/",
            "arm_controller": "arm_forward_controller",
            "use_rviz": "false",
            # NOTE: must be set explicitly. ROS 2 launch configurations are
            # NOT scoped per IncludeLaunchDescription: leader.launch.py above
            # already sets the (identically-named) "controller_config_file"
            # LaunchConfiguration to leader_controllers.yaml, and
            # DeclareLaunchArgument does not override an already-set value.
            # Leaving this unset would silently make the follower inherit the
            # leader's controller yaml (which has no arm_forward_controller /
            # gripper_controller entries).
            "controller_config_file": os.path.join(
                bringup_share,
                "config",
                "ros2_control",
                "follower_split_controllers.yaml",
            ),
        }.items(),
    )

    # --- 2. MoveIt Servo + leader -> Servo adapter ---------------------------
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_moveit_config"),
                "launch",
                "servo.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    teleop_servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_teleop"), "launch", "teleop_servo.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            # Real leader arm drives the mirror; no sim keyboard.
            "launch_keyboard": "false",
            "kp": kp,
        }.items(),
    )

    # --- 3. Servo pause bridge (so101_safety, perception-agnostic) ----------
    safety_stop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_safety"), "launch", "safety_stop_servo.launch.py"]
            )
        ),
        launch_arguments={
            "safety_stop_topic": safety_stop_topic,
            "pause_service": "/follower/servo_node/pause_servo",
            "use_sim_time": "false",
        }.items(),
    )

    # --- 4. Camera + AI perception (optional, external by default) ---------
    # Assumed to already be running externally (ai-vision-ros2 + usb-cam
    # snaps) unless launch_depth:=true.
    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="cam_overhead",
        namespace="static_camera",
        output="screen",
        parameters=[
            os.path.join(
                bringup_share, "config", "cameras", "so101_usb_cam.yaml"
            ),
            {
                "video_device": camera_device,
                "camera_name": "cam_overhead",
                "frame_id": "cam_overhead",
                "use_sim_time": False,
            },
        ],
        condition=IfCondition(launch_depth),
    )

    depth_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_depth_demo"), "launch", "depth_demo.launch.py"]
            )
        ),
        launch_arguments={
            "model_path": model_path,
            "input_image_topic": image_topic,
            "output_image_topic": depth_viz_topic,
            "stop_topic": safety_stop_topic,
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'",
                launch_depth,
                "' == 'true' and '",
                perception_backend,
                "' == 'depth'",
            ])
        ),
    )

    yolo_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_yolo_demo"), "launch", "yolo_demo.launch.py"]
            )
        ),
        launch_arguments={
            "model_path": yolo_model_path,
            "input_image_topic": image_topic,
            "output_image_topic": detections_viz_topic,
            "stop_topic": safety_stop_topic,
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'",
                launch_depth,
                "' == 'true' and '",
                perception_backend,
                "' == 'detection'",
            ])
        ),
    )

    # --- Debug viewer -----------------------------------------------
    # rqt_image_view has a live topic-selector dropdown built in, so a
    # single instance covers both backends -- no relaunch, no
    # perception_backend coupling needed at all: switch topics in the GUI
    # (/camera/depth/visualization or /camera/detections/visualization)
    # whenever you `snap refresh ai-vision-ros2 --channel=...`, matching the
    # "zero ROS-side restart" swap. Each viz topic is the REAL camera view
    # (colorised depth / annotated detections) with the safety ROI and
    # trigger state drawn directly onto it by the perception node itself.
    viewer = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="perception_viewer",
        output="screen",
        condition=IfCondition(use_viewer),
    )

    # --- 5. Layout TF + RViz ------------------------------------------------
    layout_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "layout_tf.launch.py")
        ),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="teleop_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "rviz", "teleop.rviz"]
            ),
        ],
        condition=IfCondition(use_teleop_rviz),
        output="screen",
    )

    # Camera + depth can start immediately. Servo needs the follower
    # controllers (arm_forward_controller + gripper_controller) up before it
    # can stream to them; the teleop adapter + safety bridge need Servo up
    # (both retry their Servo service calls, so the exact timing is
    # forgiving).
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "leader_usb_port", default_value="/dev/so101_leader"
            ),
            DeclareLaunchArgument(
                "follower_usb_port", default_value="/dev/so101_follower"
            ),
            DeclareLaunchArgument(
                "camera_device",
                default_value="/dev/cam_overhead",
                description="V4L2 device for the overhead (Logitech) camera "
                "(only used if launch_depth:=true).",
            ),
            DeclareLaunchArgument(
                "perception_backend",
                default_value="depth",
                description="Only affects launch_depth:=true -- which node "
                "(depth_anything_node or yolo_detect_node) to bring up "
                "inline for dev/testing without the ai-vision-ros2 snap. "
                "Has no effect when the snap is used externally (the "
                "default), and no effect on safety-monitor selection or on "
                "the viewer, since the protective-stop trigger logic lives "
                "directly in whichever AI perception node is actually "
                "running, and the viewer (rqt_image_view) lets you pick "
                "the topic live from its own dropdown.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=os.path.expanduser(
                    "~/models/depth_anything_v2_small.onnx"
                ),
                description="Only used if launch_depth:=true and "
                "perception_backend:=depth.",
            ),
            DeclareLaunchArgument(
                "yolo_model_path",
                default_value=os.path.expanduser("~/models/yolov8n.onnx"),
                description="Only used if launch_depth:=true and "
                "perception_backend:=detection.",
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/static_camera/image_raw"
            ),
            DeclareLaunchArgument(
                "safety_stop_topic",
                default_value="/safety/protective_stop",
                description="Bool protective-stop topic, published directly "
                "by whichever AI perception node (snap or inline) is "
                "running, consumed by safety_pause_bridge.",
            ),
            DeclareLaunchArgument(
                "depth_viz_topic",
                default_value="/camera/depth/visualization",
                description="depth_anything_node's colorised depth view "
                "with the safety ROI + trigger state overlaid -- pick this "
                "in the rqt_image_view dropdown (use_viewer) to watch the "
                "depth backend.",
            ),
            DeclareLaunchArgument(
                "detections_viz_topic",
                default_value="/camera/detections/visualization",
                description="yolo_detect_node's annotated camera view (real "
                "image + bounding boxes/labels + safety ROI/trigger state "
                "overlaid) -- pick this in the rqt_image_view dropdown "
                "(use_viewer) to watch the detection backend.",
            ),
            DeclareLaunchArgument(
                "use_viewer",
                default_value="false",
                description="Open a single rqt_image_view window (topic "
                "picked live from its own dropdown -- switch between "
                "depth_viz_topic/detections_viz_topic yourself, including "
                "after a snap refresh, no relaunch needed).",
            ),
            DeclareLaunchArgument("use_teleop_rviz", default_value="true"),
            DeclareLaunchArgument(
                "launch_depth",
                default_value="false",
                description="Bring up the camera + perception model inline "
                "instead of assuming the ai-vision-ros2 + usb-cam snaps are "
                "already running externally.",
            ),
            DeclareLaunchArgument(
                "kp",
                default_value="10.0",
                description="leader_servo_jog proportional gain (live-tunable).",
            ),
            DeclareLaunchArgument(
                "servo_start_delay",
                default_value="8.0",
                description="Seconds to wait before starting Servo (follower "
                "controllers need to spawn first).",
            ),
            DeclareLaunchArgument(
                "teleop_start_delay",
                default_value="11.0",
                description="Seconds to wait before starting the teleop "
                "adapter + safety bridge (needs Servo up; it retries the "
                "switch_command_type / pause_servo service calls).",
            ),
            leader,
            follower,
            layout_tf,
            camera,
            depth_model,
            yolo_model,
            safety_stop,
            viewer,
            rviz_node,
            TimerAction(period=servo_start_delay, actions=[servo]),
            TimerAction(period=teleop_start_delay, actions=[teleop_servo]),
        ]
    )
