"""Full SO-101 depth safety-stop demo — everything in one launch.

Brings up, in order:
  1. the Gazebo simulation        (so101_bringup gazebo_teleop_sim.launch.py),
  2. the gated teleop + leader keyboard (so101_teleop sim_teleop.launch.py),
     with the follower trajectory routed to /safety/follower/arm_trajectory_in,
  3. the camera + Depth Anything proximity stop + safety gate + viewer
     (so101_depth_demo depth_safety_stop.launch.py).

Put your palm in front of the camera to raise /safety/protective_stop; the
safety gate then freezes the simulated follower arm while you keep driving the
leader.

Run:
  ros2 launch so101_depth_demo full_demo.launch.py
  ros2 launch so101_depth_demo full_demo.launch.py camera_device:=/dev/video0

The leader keyboard opens in its own xterm window (needs xterm). Without xterm:
  ros2 launch so101_depth_demo full_demo.launch.py launch_keyboard:=false
then run leader_sim_keyboard yourself in a spare terminal (see sim_teleop.launch.py).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_device = LaunchConfiguration("camera_device")
    launch_keyboard = LaunchConfiguration("launch_keyboard")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    safety_stop_topic = LaunchConfiguration("safety_stop_topic")

    bringup_share = get_package_share_directory("so101_bringup")
    gazebo_launch = os.path.join(bringup_share, "launch", "gazebo_teleop_sim.launch.py")

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
            "jtc_topic": "/safety/follower/arm_trajectory_in",
            "launch_keyboard": launch_keyboard,
            "keyboard_prefix": keyboard_prefix,
        }.items(),
    )

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("so101_depth_demo"),
                    "launch",
                    "depth_safety_stop.launch.py",
                ]
            )
        ),
        launch_arguments={
            "camera_device": camera_device,
            "safety_stop_topic": safety_stop_topic,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_device", default_value="/dev/video4"),
            DeclareLaunchArgument("launch_keyboard", default_value="true"),
            DeclareLaunchArgument("keyboard_prefix", default_value="xterm -e"),
            DeclareLaunchArgument(
                "safety_stop_topic", default_value="/safety/protective_stop"
            ),
            sim,
            # Give the sim time to spawn robots and start controllers before the
            # teleop bridge and safety gate connect to them.
            TimerAction(period=8.0, actions=[teleop, safety]),
        ]
    )
