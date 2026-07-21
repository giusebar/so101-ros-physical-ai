"""Full SO-101 depth/detection safety-stop demo on REAL hardware, via direct teleop_split.

Direct-copy (non-Servo) counterpart to ``full_demo_real_servo.launch.py``. This
is the "teleop split through a JointTrajectory safety gate" demo documented as
the manual, per-terminal decomposition in ``docs/demo_runbook.md`` (§4.2),
packaged here as a single launch so it can be run alongside (i.e. instead of,
one at a time) the MoveIt Servo demo. Brings up, in order:

  1 & 2. Real leader + follower arms + ``teleop_split``, all via
     ``so101_bringup``'s ``teleop_split.launch.py`` (which itself brings up
     ``leader.launch.py`` / ``follower_split.launch.py`` and
     ``so101_teleop``'s ``teleop_split.launch.py``). Configured with
     ``arm_controller:=arm_trajectory_controller`` (5 arm joints) so the
     safety gate can freeze the arm with a JointTrajectory hold, plus
     ``gripper_mode:=forward_position`` (the gripper_controller here is a
     plain ForwardCommandController, not a GripperActionController) and
     ``gate_input_topic`` set so the arm trajectory is routed to the safety
     gate INPUT (``/safety/follower/arm_trajectory_in``) instead of straight
     at the controller. The gripper is forwarded directly (outside the gate).
  3. ``so101_safety``'s ``safety_stop_jtc.launch.py``: just
     ``trajectory_safety_gate``, which passes the arm trajectory through to
     ``/follower/arm_trajectory_controller/joint_trajectory`` while
     ``/safety/protective_stop`` is false, and freezes the follower (holds the
     current pose) while true. This node is perception-agnostic -- it doesn't
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
``/safety/protective_stop``; the gate then freezes the follower until the
obstacle/person clears.

  ###########################################################################
  #  SAFETY WARNING                                                          #
  #  This "protective stop" is EXPERIMENTAL CPU perception at ~1-8 Hz. It   #
  #  is NOT a functional-safety system: expect up to ~1 s of reaction        #
  #  latency. Keep a physical e-stop / power cut as the real safety          #
  #  mechanism. Test with the arm clear of people first.                     #
  ###########################################################################

Note: run this OR ``full_demo_real_servo.launch.py`` (Servo), never both at
once — they both drive the follower and open the same serial port.

Run (ai-vision-ros2/usb-cam snaps already running, the default):
  ros2 launch so101_depth_demo full_demo_real_split.launch.py \
    leader_usb_port:=/dev/ttyACM1 \
    follower_usb_port:=/dev/ttyACM0

Run (bring up camera + depth model inline instead of via the snap):
  ros2 launch so101_depth_demo full_demo_real_split.launch.py \
    leader_usb_port:=/dev/ttyACM1 \
    follower_usb_port:=/dev/ttyACM0 \
    launch_depth:=true \
    camera_device:=/dev/video4

Run (detection backend inline, for dev/testing without the snap):
  ros2 launch so101_depth_demo full_demo_real_split.launch.py \
    leader_usb_port:=/dev/ttyACM1 \
    follower_usb_port:=/dev/ttyACM0 \
    perception_backend:=detection launch_depth:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
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
    use_teleop_rviz = LaunchConfiguration("use_teleop_rviz")
    teleop_start_delay = LaunchConfiguration("teleop_start_delay")

    # Arm trajectory gate wiring: teleop_split publishes here, the gate consumes
    # here and republishes to the controller.
    gate_input_topic = "/safety/follower/arm_trajectory_in"
    arm_trajectory_topic = "/follower/arm_trajectory_controller/joint_trajectory"

    bringup_share = get_package_share_directory("so101_bringup")

    # --- 1 & 2. Real arms + teleop_split ------------------------------------
    teleop_split_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "teleop_split.launch.py")
        ),
        launch_arguments={
            "hardware_type": "real",
            "leader_usb_port": leader_usb,
            "follower_usb_port": follower_usb,
            "leader_rviz": "false",
            "follower_rviz": "false",
            "arm_controller": "arm_trajectory_controller",
            "gripper_mode": "forward_position",
            "gate_input_topic": gate_input_topic,
            "teleop_delay_s": teleop_start_delay,
        }.items(),
    )

    # --- 3. Trajectory safety gate (so101_safety, perception-agnostic) -----
    safety_stop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_safety"), "launch", "safety_stop_jtc.launch.py"]
            )
        ),
        launch_arguments={
            "safety_stop_topic": safety_stop_topic,
            "gate_input_topic": gate_input_topic,
            "gate_output_topic": arm_trajectory_topic,
            "joint_states_topic": "/follower/joint_states",
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
            os.path.join(bringup_share, "config", "cameras", "so101_usb_cam.yaml"),
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

    # --- Debug viewer ---------------------------------------------------
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
                "running, consumed by trajectory_safety_gate.",
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
                "teleop_start_delay",
                default_value="8.0",
                description="Seconds to wait before starting teleop_split "
                "(follower controllers need to spawn first).",
            ),
            teleop_split_bringup,
            safety_stop,
            layout_tf,
            camera,
            depth_model,
            yolo_model,
            viewer,
            rviz_node,
        ]
    )
