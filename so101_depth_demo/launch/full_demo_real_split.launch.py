"""Full SO-101 depth safety-stop demo on REAL hardware, via direct teleop_split.

Direct-copy (non-Servo) counterpart to ``full_demo_real.launch.py``. This is the
"teleop split through a JointTrajectory safety gate" demo documented as the
manual, per-terminal decomposition in ``docs/demo_runbook.md`` (§4.2), packaged
here as a single launch so it can be run alongside (i.e. instead of, one at a
time) the MoveIt Servo demo. Brings up, in order:

  1. Real leader + follower arms (``so101_bringup`` ``leader.launch.py`` /
     ``follower_split.launch.py``, ``hardware_type:=real``). The follower runs
     the split controllers with ``arm_controller:=arm_trajectory_controller``
     (5 arm joints) so the safety gate can freeze the arm with a JointTrajectory
     hold, plus ``gripper_controller`` for the gripper.
  2. ``teleop_split`` (``so101_teleop``): reads ``/leader/joint_states`` and
     mirrors the leader's *absolute* joint positions to the follower as a
     JointTrajectory. Its arm trajectory is routed to the safety gate INPUT
     (``/safety/follower/arm_trajectory_in``) instead of straight at the
     controller. The gripper is forwarded directly (outside the gate).
  3. ``trajectory_safety_gate`` (``so101_teleop``): passes the arm trajectory
     through to ``/follower/arm_trajectory_controller/joint_trajectory`` while
     ``/safety/protective_stop`` is false, and freezes the follower (holds the
     current pose) while it is true.
  4. The real overhead camera (usb_cam) + the CPU Depth Anything proximity node
     (-> ``/safety/protective_stop`` + debug overlay), same as the Servo demo.

Move the REAL leader arm by hand to drive the follower. Put your palm in front
of the camera to raise ``/safety/protective_stop``; the gate then freezes the
follower until the obstacle clears.

  ###########################################################################
  #  SAFETY WARNING                                                          #
  #  This "protective stop" is EXPERIMENTAL CPU monocular-depth inference at #
  #  ~1 Hz. It is NOT a functional-safety system: expect up to ~1 s of       #
  #  reaction latency. Keep a physical e-stop / power cut as the real safety #
  #  mechanism. Test with the arm clear of people first.                     #
  ###########################################################################

Note: run this OR ``full_demo_real.launch.py`` (Servo), never both at once —
they both drive the follower and open the same serial port.

Run:
  ros2 launch so101_depth_demo full_demo_real_split.launch.py \
    leader_usb_port:=/dev/ttyACM1 \
    follower_usb_port:=/dev/ttyACM0 \
    camera_device:=/dev/video4
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
    inference_hz = LaunchConfiguration("inference_hz")
    near_margin = LaunchConfiguration("near_margin")
    min_area_ratio = LaunchConfiguration("min_area_ratio")
    use_viewer = LaunchConfiguration("use_viewer")
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
    teleop_share = get_package_share_directory("so101_teleop")

    arm_joints = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

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

    # Split controllers with the JointTrajectory arm controller so the safety
    # gate can freeze the arm with a trajectory hold.
    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "follower_split.launch.py")
        ),
        launch_arguments={
            "namespace": "follower",
            "hardware_type": "real",
            "usb_port": follower_usb,
            "frame_prefix": "follower/",
            "arm_controller": "arm_trajectory_controller",
            "use_rviz": "false",
            # Must be set explicitly (launch configs are not scoped per include;
            # leader.launch.py above already set controller_config_file).
            "controller_config_file": os.path.join(
                bringup_share,
                "config",
                "ros2_control",
                "follower_split_controllers.yaml",
            ),
        }.items(),
    )

    # --- 2. teleop_split (leader absolute-position mirror -> gate input) -----
    teleop_split = Node(
        package="so101_teleop",
        executable="teleop_split",
        name="arm_gripper_teleop",
        output="screen",
        parameters=[
            os.path.join(teleop_share, "config", "teleop_split.yaml"),
            {
                "arm_mode": "joint_trajectory",
                "leader_topic": "/leader/joint_states",
                # Route the arm trajectory THROUGH the safety gate.
                "jtc_topic": gate_input_topic,
                # Gripper is outside the gate; forward its position directly to
                # the gripper ForwardCommandController's /commands topic (same
                # topic the Servo demo's leader_servo_jog uses).
                "gripper_mode": "forward_position",
                "gripper_fwd_topic": "/follower/gripper_controller/commands",
                "arm_joints": arm_joints,
                "gripper_joint": "gripper",
                "use_sim_time": False,
            },
        ],
    )

    # --- 3. Trajectory safety gate ------------------------------------------
    safety_gate = Node(
        package="so101_teleop",
        executable="trajectory_safety_gate",
        name="trajectory_safety_gate",
        output="screen",
        parameters=[
            {
                "input_topic": gate_input_topic,
                "output_topic": arm_trajectory_topic,
                "safety_stop_topic": safety_stop_topic,
                "joint_states_topic": "/follower/joint_states",
                "arm_joints": arm_joints,
                "use_sim_time": False,
            }
        ],
    )

    # --- 4a. Real overhead camera (usb_cam) ---------------------------------
    #camera = Node(
    #    package="usb_cam",
    #    executable="usb_cam_node_exe",
    #    name="cam_overhead",
    #    namespace="static_camera",
    #    output="screen",
    #    parameters=[
    #        os.path.join(bringup_share, "config", "cameras", "so101_usb_cam.yaml"),
    #        {
    #            "video_device": camera_device,
    #            "camera_name": "cam_overhead",
    #            "frame_id": "cam_overhead",
    #            "use_sim_time": False,
    #        },
    #    ],
    #)

    # --- 4b. Depth proximity -> protective stop -----------------------------
    depth_stop = Node(
        package="so101_depth_demo",
        executable="depth_proximity_node",
        name="depth_proximity_node",
        output="screen",
        parameters=[
            {
                "model_path": model_path,
                "input_image_topic": image_topic,
                "stop_topic": safety_stop_topic,
                "debug_image_topic": debug_image_topic,
                "near_margin": near_margin,
                "min_area_ratio": min_area_ratio,
                "inference_hz": inference_hz,
                "publish_debug_image": True,
            }
        ],
    )

    # --- 4c. Debug viewer ---------------------------------------------------
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

    # Camera + depth can start immediately. teleop_split + gate need the
    # follower controllers (arm_trajectory_controller + gripper_controller) up
    # before they can publish to them.
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
                description="V4L2 device for the overhead (Logitech) camera.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=os.path.expanduser(
                    "~/models/depth_anything_v2_small.onnx"
                ),
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/static_camera/image_raw"
            ),
            DeclareLaunchArgument(
                "safety_stop_topic", default_value="/safety/protective_stop"
            ),
            DeclareLaunchArgument(
                "debug_image_topic", default_value="/safety/depth_debug_image"
            ),
            DeclareLaunchArgument("inference_hz", default_value="1.0"),
            DeclareLaunchArgument("near_margin", default_value="0.15"),
            DeclareLaunchArgument("min_area_ratio", default_value="0.12"),
            DeclareLaunchArgument("use_viewer", default_value="false"),
            DeclareLaunchArgument("use_teleop_rviz", default_value="true"),
            DeclareLaunchArgument(
                "teleop_start_delay",
                default_value="8.0",
                description="Seconds to wait before starting teleop_split + the "
                "safety gate (follower controllers need to spawn first).",
            ),
            leader,
            follower,
            layout_tf,
            camera,
            depth_stop,
            viewer,
            rviz_node,
            TimerAction(
                period=teleop_start_delay, actions=[safety_gate, teleop_split]
            ),
        ]
    )
