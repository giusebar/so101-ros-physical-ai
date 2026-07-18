"""Person-safety monitor + safety_pause_bridge (MoveIt Servo path).

Consumes an already-published detections topic (from the ``yolodetect`` snap,
or ``so101_yolo_demo``'s ``yolo_detect_node`` if ``launch_depth:=true``-style
inline bring-up was used on the parent demo launch) and pauses/resumes MoveIt
Servo via ``safety_pause_bridge`` whenever a watched-class detection (person,
by default) overlaps the ROI.

This is the detection-based twin of ``safety_stop_servo.launch.py`` -- same
enforcement node, different perception source.

Run:
  ros2 launch so101_safety safety_stop_servo_yolo.launch.py
  ros2 launch so101_safety safety_stop_servo_yolo.launch.py detections_topic:=/perception/detections
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detections_topic = LaunchConfiguration("detections_topic")
    safety_stop_topic = LaunchConfiguration("safety_stop_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")

    monitor = Node(
        package="so101_safety",
        executable="person_safety_monitor",
        name="person_safety_monitor",
        output="screen",
        parameters=[
            {
                "detections_topic": detections_topic,
                "stop_topic": safety_stop_topic,
                "debug_image_topic": debug_image_topic,
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "roi": LaunchConfiguration("roi"),
                "watched_classes": LaunchConfiguration("watched_classes"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "min_overlap_ratio": LaunchConfiguration("min_overlap_ratio"),
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
                "detections_topic",
                default_value="/perception/detections",
                description="vision_msgs/Detection2DArray topic to consume.",
            ),
            DeclareLaunchArgument(
                "safety_stop_topic", default_value="/safety/protective_stop"
            ),
            DeclareLaunchArgument(
                "debug_image_topic", default_value="/safety/detection_debug_image"
            ),
            DeclareLaunchArgument(
                "image_width",
                default_value="640",
                description="Must match the perception node's input frame width.",
            ),
            DeclareLaunchArgument("image_height", default_value="480"),
            DeclareLaunchArgument("roi", default_value="0.25,0.2,0.75,0.85"),
            DeclareLaunchArgument("watched_classes", default_value="person"),
            DeclareLaunchArgument("conf_threshold", default_value="0.4"),
            DeclareLaunchArgument("min_overlap_ratio", default_value="0.2"),
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
