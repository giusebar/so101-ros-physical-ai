"""Full SO-101 depth safety-stop demo on REAL hardware, via MoveIt Servo.

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
  3. ``so101_safety``'s ``safety_stop_servo.launch.py``: a
     ``depth_safety_monitor`` that consumes the depth topic and raises
     ``/safety/protective_stop``, plus ``safety_pause_bridge`` that
     pauses/resumes Servo on it (Servo's own collision/joint-limit checking
     supersedes the ``trajectory_safety_gate`` used on the JointTrajectory
     path).
  4. The camera + depth model (``depth_anything_node``) publishing the depth
     topic that (3) consumes. By default this is assumed to be running
     EXTERNALLY (e.g. the ``depthanything`` + ``usb-cam`` snaps, per
     ``docs/depthanything_usbcam_setup.md``) -- set ``launch_depth:=true`` to
     bring it up inline instead (useful for sim/dev without the snaps).

Move the REAL leader arm by hand to drive the follower. Put your palm in front
of the camera to raise /safety/protective_stop; Servo then halts (and later
resumes) the follower until the obstacle clears.

  ###########################################################################
  #  SAFETY WARNING                                                          #
  #  This "protective stop" is EXPERIMENTAL CPU monocular-depth inference at #
  #  ~1 Hz. It is NOT a functional-safety system: expect up to ~1 s of       #
  #  reaction latency, and it only pauses MoveIt Servo. Keep a physical      #
  #  e-stop / power cut as the real safety mechanism. Test with the arm      #
  #  clear of people first.                                                 #
  ###########################################################################

Run (depthanything/usb-cam snaps already running, the default):
  ros2 launch so101_depth_demo full_demo_real_servo.launch.py

Run (bring up camera + depth model inline instead of via snaps):
  ros2 launch so101_depth_demo full_demo_real_servo.launch.py \
    launch_depth:=true camera_device:=/dev/cam_overhead

Prerequisites:
  - LeRobot motor setup + calibration done on both arms (EEPROM written).
  - udev symlinks /dev/so101_leader, /dev/so101_follower (or override the
    *_usb_port args), and the user in the `dialout` group.
  - If launch_depth:=true: ONNX model at ~/models/depth_anything_v2_small.onnx
    (so101_depth_demo/scripts/download_model.sh) and onnxruntime installed in
    the interpreter ros2 uses. Otherwise, the depthanything snap already
    bundles this.
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    leader_usb = LaunchConfiguration("leader_usb_port")
    follower_usb = LaunchConfiguration("follower_usb_port")
    camera_device = LaunchConfiguration("camera_device")
    model_path = LaunchConfiguration("model_path")
    safety_stop_topic = LaunchConfiguration("safety_stop_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    image_topic = LaunchConfiguration("image_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    near_margin = LaunchConfiguration("near_margin")
    min_area_ratio = LaunchConfiguration("min_area_ratio")
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

    # --- 3. Depth safety monitor + Servo pause bridge (so101_safety) --------
    safety_stop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_safety"), "launch", "safety_stop_servo.launch.py"]
            )
        ),
        launch_arguments={
            "depth_image_topic": depth_image_topic,
            "safety_stop_topic": safety_stop_topic,
            "debug_image_topic": debug_image_topic,
            "near_margin": near_margin,
            "min_area_ratio": min_area_ratio,
            "publish_debug_image": use_viewer,
            "pause_service": "/follower/servo_node/pause_servo",
            "use_sim_time": "false",
        }.items(),
    )

    # --- 4. Camera + depth model (optional, external by default) ------------
    # Assumed to already be running externally (depthanything + usb-cam snaps)
    # unless launch_depth:=true.
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
            "output_depth_topic": depth_image_topic,
        }.items(),
        condition=IfCondition(launch_depth),
    )

    # --- Debug viewer -----------------------------------------------
    # Use C++ image_view instead of rqt_image_view to avoid PyQt5 import
    # issues in the workshop container.
    viewer = Node(
        package="image_view",
        executable="image_view",
        name="depth_proximity_viewer",
        output="screen",
        remappings=[("image", debug_image_topic)],
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
                "model_path",
                default_value=os.path.expanduser(
                    "~/models/depth_anything_v2_small.onnx"
                ),
                description="Only used if launch_depth:=true.",
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
                "safety_stop_topic", default_value="/safety/protective_stop"
            ),
            DeclareLaunchArgument(
                "debug_image_topic", default_value="/safety/depth_debug_image"
            ),
            DeclareLaunchArgument("near_margin", default_value="0.15"),
            DeclareLaunchArgument("min_area_ratio", default_value="0.12"),
            DeclareLaunchArgument("use_viewer", default_value="false"),
            DeclareLaunchArgument("use_teleop_rviz", default_value="true"),
            DeclareLaunchArgument(
                "launch_depth",
                default_value="false",
                description="Bring up the camera + depth_anything_node inline "
                "instead of assuming the depthanything/usb-cam snaps are "
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
            safety_stop,
            viewer,
            rviz_node,
            TimerAction(period=servo_start_delay, actions=[servo]),
            TimerAction(period=teleop_start_delay, actions=[teleop_servo]),
        ]
    )
