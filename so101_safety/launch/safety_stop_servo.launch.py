"""Depth-safety monitor + safety_pause_bridge (MoveIt Servo path).

Consumes an already-published depth topic (from the ``depthanything`` snap, or
``so101_depth_demo``'s ``depth_anything_node`` if ``launch_depth:=true`` was
used on the parent demo launch) and pauses/resumes MoveIt Servo via
``safety_pause_bridge`` whenever a near object is detected.

Run:
  ros2 launch so101_safety safety_stop_servo.launch.py
  ros2 launch so101_safety safety_stop_servo.launch.py depth_image_topic:=/perception/depth
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    safety_stop_topic = LaunchConfiguration("safety_stop_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")

    monitor = Node(
        package="so101_safety",
        executable="depth_safety_monitor",
        name="depth_safety_monitor",
        output="screen",
        parameters=[
            {
                "depth_image_topic": depth_image_topic,
                "stop_topic": safety_stop_topic,
                "debug_image_topic": debug_image_topic,
                "roi": LaunchConfiguration("roi"),
                "near_threshold": LaunchConfiguration("near_threshold"),
                "near_margin": LaunchConfiguration("near_margin"),
                "min_area_ratio": LaunchConfiguration("min_area_ratio"),
                "monitor_hz": LaunchConfiguration("monitor_hz"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            }
        ],
    )

    safety_bridge = Node(
        package="so101_safety",
        executable="safety_pause_bridge",
        name="safety_pause_bridge",
        output="screen",
        parameters=[
            {
                "safety_stop_topic": safety_stop_topic,
                "pause_service": LaunchConfiguration("pause_service"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/perception/depth",
                description="Raw normalised depth (32FC1) topic to consume.",
            ),
            DeclareLaunchArgument(
                "safety_stop_topic", default_value="/safety/protective_stop"
            ),
            DeclareLaunchArgument(
                "debug_image_topic", default_value="/safety/depth_debug_image"
            ),
            DeclareLaunchArgument("roi", default_value="0.25,0.2,0.75,0.85"),
            DeclareLaunchArgument("near_threshold", default_value="0.6"),
            DeclareLaunchArgument("near_margin", default_value="0.15"),
            DeclareLaunchArgument("min_area_ratio", default_value="0.12"),
            DeclareLaunchArgument("monitor_hz", default_value="10.0"),
            DeclareLaunchArgument("publish_debug_image", default_value="false"),
            DeclareLaunchArgument(
                "pause_service", default_value="/follower/servo_node/pause_servo"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            monitor,
            safety_bridge,
        ]
    )
