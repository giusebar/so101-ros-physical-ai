"""Full SO-101 depth safety-stop demo — everything in one launch.

Brings up, in order:
  1. the Gazebo simulation        (so101_bringup gazebo_teleop_sim.launch.py),
  2. the gated teleop + leader keyboard (so101_teleop sim_teleop.launch.py),
     with the follower trajectory routed to /safety/follower/arm_trajectory_in,
  3. the depth safety monitor + trajectory safety gate (so101_safety
     safety_stop_jtc.launch.py), consuming the depth topic and freezing the
     follower on a protective stop,
  4. (optional, launch_depth:=true) the camera + depth_anything_node publishing
     that depth topic -- by default this is assumed to already be running
     externally (the depthanything + usb-cam snaps).

Put your palm in front of the camera to raise /safety/protective_stop; the
safety gate then freezes the simulated follower arm while you keep driving the
leader.

NOTE: per docs/demo_runbook.md this sim path has known staleness issues
(camera driver / gate topic naming) predating this refactor -- treat it as a
starting point, not a verified-working demo.

Run (depthanything/usb-cam snaps already running, the default):
  ros2 launch so101_depth_demo full_demo_sim.launch.py

Run (bring up camera + depth model inline instead of via snaps):
  ros2 launch so101_depth_demo full_demo_sim.launch.py launch_depth:=true camera_device:=/dev/video0

The leader keyboard opens in its own xterm window (needs xterm). Without xterm:
  ros2 launch so101_depth_demo full_demo_sim.launch.py launch_keyboard:=false
then run leader_sim_keyboard yourself in a spare terminal (see sim_teleop.launch.py).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_device = LaunchConfiguration("camera_device")
    launch_keyboard = LaunchConfiguration("launch_keyboard")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    safety_stop_topic = LaunchConfiguration("safety_stop_topic")
    image_topic = LaunchConfiguration("image_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    model_path = LaunchConfiguration("model_path")
    launch_depth = LaunchConfiguration("launch_depth")

    bringup_share = get_package_share_directory("so101_bringup")
    gazebo_launch = os.path.join(bringup_share, "launch", "gazebo_teleop_sim.launch.py")

    gate_input_topic = "/safety/follower/arm_trajectory_in"

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "launch_keyboard": "false",
            "launch_teleop": "false",
        }.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_teleop"), "launch", "sim_teleop.launch.py"]
            )
        ),
        launch_arguments={
            "jtc_topic": gate_input_topic,
            "launch_keyboard": launch_keyboard,
            "keyboard_prefix": keyboard_prefix,
        }.items(),
    )

    safety_stop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_safety"), "launch", "safety_stop_jtc.launch.py"]
            )
        ),
        launch_arguments={
            "depth_image_topic": depth_image_topic,
            "safety_stop_topic": safety_stop_topic,
            "gate_input_topic": gate_input_topic,
            "use_sim_time": "true",
        }.items(),
    )

    # Camera + depth model: optional, external by default (depthanything +
    # usb-cam snaps). Set launch_depth:=true to bring them up inline.
    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="cam_overhead",
        namespace="static_camera",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "config", "cameras", "so101_usb_cam.yaml"]
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_device", default_value="/dev/video4"),
            DeclareLaunchArgument("launch_keyboard", default_value="true"),
            DeclareLaunchArgument("keyboard_prefix", default_value="xterm -e"),
            DeclareLaunchArgument(
                "safety_stop_topic", default_value="/safety/protective_stop"
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/static_camera/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_image_topic", default_value="/perception/depth"
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=os.path.expanduser(
                    "~/models/depth_anything_v2_small.onnx"
                ),
                description="Only used if launch_depth:=true.",
            ),
            DeclareLaunchArgument(
                "launch_depth",
                default_value="false",
                description="Bring up the camera + depth_anything_node inline "
                "instead of assuming the depthanything/usb-cam snaps are "
                "already running externally.",
            ),
            sim,
            camera,
            depth_model,
            # Give the sim time to spawn robots and start controllers before the
            # teleop bridge and safety gate connect to them.
            TimerAction(period=8.0, actions=[teleop, safety_stop]),
        ]
    )
