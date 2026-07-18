"""Full SO-101 depth safety-stop demo on REAL hardware, via direct teleop_split.

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
  3. ``so101_safety``'s ``safety_stop_jtc.launch.py``: a ``depth_safety_monitor``
     that consumes the depth topic and raises ``/safety/protective_stop``, plus
     ``trajectory_safety_gate`` which passes the arm trajectory through to
     ``/follower/arm_trajectory_controller/joint_trajectory`` while the topic
     is false, and freezes the follower (holds the current pose) while true.
  4. The camera + perception model publishing whatever topic (3) consumes.
     By default this is assumed to be running EXTERNALLY (e.g. the
     ``depthanything``/``yolodetect`` + ``usb-cam`` snaps, per
     ``docs/depthanything_usbcam_setup.md`` / ``docs/yolodetect_setup.md``) --
     set ``launch_depth:=true`` to bring it up inline instead (useful for
     sim/dev without the snaps).

  ``perception_backend`` selects which perception method drives the
  protective stop, without touching anything else in this launch file:
    - ``depth`` (default): monocular relative-depth proximity
      (``depth_anything_node`` / ``depthanything`` snap +
      ``depth_safety_monitor``).
    - ``detection``: YOLOv8n person detection (``yolo_detect_node`` /
      ``yolodetect`` snap + ``person_safety_monitor``). Swap to this after
      installing the ``yolodetect`` (+ ``yolodetect-model``) snap in place of
      ``depthanything`` -- see ``docs/yolodetect_setup.md``.

Move the REAL leader arm by hand to drive the follower. Put your palm in front
of the camera (depth backend) or step into frame (detection backend) to raise
``/safety/protective_stop``; the gate then freezes the follower until the
obstacle/person clears.

  ###########################################################################
  #  SAFETY WARNING                                                          #
  #  This "protective stop" is EXPERIMENTAL CPU monocular-depth inference at #
  #  ~1 Hz. It is NOT a functional-safety system: expect up to ~1 s of       #
  #  reaction latency. Keep a physical e-stop / power cut as the real safety #
  #  mechanism. Test with the arm clear of people first.                     #
  ###########################################################################

Note: run this OR ``full_demo_real_servo.launch.py`` (Servo), never both at
once — they both drive the follower and open the same serial port.

Run (depthanything/usb-cam snaps already running, the default):
  ros2 launch so101_depth_demo full_demo_real_split.launch.py \
    leader_usb_port:=/dev/ttyACM1 \
    follower_usb_port:=/dev/ttyACM0

Run (bring up camera + depth model inline instead of via snaps):
  ros2 launch so101_depth_demo full_demo_real_split.launch.py \
    leader_usb_port:=/dev/ttyACM1 \
    follower_usb_port:=/dev/ttyACM0 \
    launch_depth:=true \
    camera_device:=/dev/video4

Run (detection backend, yolodetect/usb-cam snaps already running):
  ros2 launch so101_depth_demo full_demo_real_split.launch.py \
    leader_usb_port:=/dev/ttyACM1 \
    follower_usb_port:=/dev/ttyACM0 \
    perception_backend:=detection
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
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    detection_debug_image_topic = LaunchConfiguration("detection_debug_image_topic")
    detections_viz_topic = LaunchConfiguration("detections_viz_topic")
    image_topic = LaunchConfiguration("image_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    near_margin = LaunchConfiguration("near_margin")
    min_area_ratio = LaunchConfiguration("min_area_ratio")
    use_viewer = LaunchConfiguration("use_viewer")
    launch_depth = LaunchConfiguration("launch_depth")
    # LaunchConfiguration.perform() returns the raw configured string (e.g.
    # "true"/"false", lowercase) which is NOT valid Python -- PythonExpression
    # evaluates its concatenated substitutions with eval(), so bare `true`
    # raises "name 'true' is not defined". Compare as a quoted string instead
    # (mirrors is_depth_backend/is_detection_backend above), then combine
    # these boolean PythonExpressions with " and " below -- nested
    # PythonExpressions are perform()'d to "True"/"False" before the outer
    # expression is evaluated, so that combination is safe.
    is_launch_depth = PythonExpression(["'", launch_depth, "' == 'true'"])
    is_use_viewer = PythonExpression(["'", use_viewer, "' == 'true'"])
    # NOTE: deliberately NOT named "use_rviz" - leader.launch.py and
    # follower_split.launch.py both declare a launch argument with that exact
    # name (each hardcoded to "false" below). ROS 2 launch configurations are
    # not scoped per IncludeLaunchDescription, so reusing "use_rviz" here would
    # be silently overwritten to "false".
    use_teleop_rviz = LaunchConfiguration("use_teleop_rviz")
    teleop_start_delay = LaunchConfiguration("teleop_start_delay")

    # Arm trajectory gate wiring: teleop_split publishes here, the gate consumes
    # here and republishes to the controller.
    gate_input_topic = "/safety/follower/arm_trajectory_in"
    arm_trajectory_topic = "/follower/arm_trajectory_controller/joint_trajectory"

    bringup_share = get_package_share_directory("so101_bringup")

    # --- 1 & 2. Real arms + teleop_split ------------------------------------
    # Reuses so101_bringup's teleop_split.launch.py (leader + follower_split +
    # teleop_split), which already knows how to bring up the split controllers
    # and mirror the leader to the follower. We only override:
    #  - arm_controller: JointTrajectory controller, so the safety gate can
    #    freeze the arm with a trajectory hold.
    #  - gripper_mode: forward_position (the gripper_controller here is a
    #    plain ForwardCommandController, not a GripperActionController, so
    #    the default parallel_action mode would silently do nothing).
    #  - gate_input_topic: routes teleop_split's arm trajectory to the safety
    #    gate's input instead of straight at the controller.
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

    # --- 3. Perception safety monitor + trajectory safety gate (so101_safety) ---
    safety_stop_depth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_safety"), "launch", "safety_stop_jtc.launch.py"]
            )
        ),
        launch_arguments={
            "depth_image_topic": depth_image_topic,
            "safety_stop_topic": safety_stop_topic,
            "debug_image_topic": debug_image_topic,
            "near_margin": near_margin,
            "min_area_ratio": min_area_ratio,
            "publish_debug_image": use_viewer,
            "gate_input_topic": gate_input_topic,
            "gate_output_topic": arm_trajectory_topic,
            "joint_states_topic": "/follower/joint_states",
            "use_sim_time": "false",
        }.items(),
        condition=IfCondition(is_depth_backend),
    )

    safety_stop_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_safety"), "launch", "safety_stop_jtc_yolo.launch.py"]
            )
        ),
        launch_arguments={
            "detections_topic": detections_topic,
            "safety_stop_topic": safety_stop_topic,
            "debug_image_topic": detection_debug_image_topic,
            "publish_debug_image": use_viewer,
            "gate_input_topic": gate_input_topic,
            "gate_output_topic": arm_trajectory_topic,
            "joint_states_topic": "/follower/joint_states",
            "use_sim_time": "false",
        }.items(),
        condition=IfCondition(is_detection_backend),
    )

    # --- 4. Camera + perception model (optional, external by default) -------
    # Assumed to already be running externally (depthanything/yolodetect +
    # usb-cam snaps) unless launch_depth:=true.
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
            "output_depth_topic": depth_image_topic,
        }.items(),
        condition=IfCondition(PythonExpression([is_launch_depth, " and ", is_depth_backend])),
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
            "output_detections_topic": detections_topic,
        }.items(),
        condition=IfCondition(PythonExpression([is_launch_depth, " and ", is_detection_backend])),
    )

    # --- Debug viewer ---------------------------------------------------
    viewer_depth = Node(
        package="image_view",
        executable="image_view",
        name="depth_proximity_viewer",
        output="screen",
        remappings=[("image", debug_image_topic)],
        condition=IfCondition(PythonExpression([is_use_viewer, " and ", is_depth_backend])),
    )

    # Shows the ANNOTATED CAMERA VIEW (real image + bounding boxes/labels) --
    # this is what actually shows a detected person, unlike
    # depth_proximity_viewer's ROI-proximity overlay (there's no equivalent
    # "raw camera + markup" view for the depth backend, since depth_safety_monitor
    # only ever sees a normalised depth map, not the original frame).
    viewer_detection = Node(
        package="image_view",
        executable="image_view",
        name="detection_proximity_viewer",
        output="screen",
        remappings=[("image", detections_viz_topic)],
        condition=IfCondition(PythonExpression([is_use_viewer, " and ", is_detection_backend])),
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

    # Camera + depth can start immediately. teleop_split (inside
    # teleop_split_bringup) needs the follower controllers up first, which is
    # handled internally via its own teleop_delay_s (fed from
    # teleop_start_delay below). The safety gate can start immediately - it
    # just caches /follower/joint_states and passes through messages once
    # teleop_split starts publishing.
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
                description="Which perception method drives the protective "
                "stop: 'depth' (depth_anything_node/depthanything snap + "
                "depth_safety_monitor) or 'detection' (yolo_detect_node/"
                "yolodetect snap + person_safety_monitor).",
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
                "depth_image_topic",
                default_value="/perception/depth",
                description="Raw normalised depth (32FC1) topic published by "
                "depth_anything_node (snap or inline) and consumed by "
                "so101_safety's depth_safety_monitor.",
            ),
            DeclareLaunchArgument(
                "detections_topic",
                default_value="/perception/detections",
                description="vision_msgs/Detection2DArray topic published by "
                "yolo_detect_node (snap or inline) and consumed by "
                "so101_safety's person_safety_monitor.",
            ),
            DeclareLaunchArgument(
                "safety_stop_topic", default_value="/safety/protective_stop"
            ),
            DeclareLaunchArgument(
                "debug_image_topic", default_value="/safety/depth_debug_image"
            ),
            DeclareLaunchArgument(
                "detection_debug_image_topic",
                default_value="/safety/detection_debug_image",
                description="person_safety_monitor's synthetic ROI-overlap "
                "gauge (NOT a camera view). Use detections_viz_topic to "
                "see the actual annotated camera image.",
            ),
            DeclareLaunchArgument(
                "detections_viz_topic",
                default_value="/camera/detections/visualization",
                description="yolo_detect_node's annotated camera view (real "
                "image + bounding boxes/labels) -- what use_viewer opens for "
                "the detection backend.",
            ),
            DeclareLaunchArgument("near_margin", default_value="0.15"),
            DeclareLaunchArgument("min_area_ratio", default_value="0.12"),
            DeclareLaunchArgument("use_viewer", default_value="false"),
            DeclareLaunchArgument("use_teleop_rviz", default_value="true"),
            DeclareLaunchArgument(
                "launch_depth",
                default_value="false",
                description="Bring up the camera + perception model inline "
                "instead of assuming the depthanything/yolodetect + usb-cam "
                "snaps are already running externally.",
            ),
            DeclareLaunchArgument(
                "teleop_start_delay",
                default_value="8.0",
                description="Seconds to wait before starting teleop_split "
                "(follower controllers need to spawn first).",
            ),
            teleop_split_bringup,
            safety_stop_depth,
            safety_stop_detection,
            layout_tf,
            camera,
            depth_model,
            yolo_model,
            viewer_depth,
            viewer_detection,
            rviz_node,
        ]
    )
