"""Person-safety monitor + trajectory_safety_gate (JointTrajectory/JTC path).

Consumes an already-published detections topic (from the ``yolodetect`` snap,
or ``so101_yolo_demo``'s ``yolo_detect_node`` if brought up inline) and
freezes the follower's JointTrajectory stream via ``trajectory_safety_gate``
whenever a watched-class detection (person, by default) overlaps the ROI.

This is the detection-based twin of ``safety_stop_jtc.launch.py`` -- same
enforcement node, different perception source.

Run:
  ros2 launch so101_safety safety_stop_jtc_yolo.launch.py
  ros2 launch so101_safety safety_stop_jtc_yolo.launch.py detections_topic:=/perception/detections
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

    safety_gate = Node(
        package="so101_safety",
        executable="trajectory_safety_gate",
        name="trajectory_safety_gate",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("gate_input_topic"),
                "output_topic": LaunchConfiguration("gate_output_topic"),
                "safety_stop_topic": safety_stop_topic,
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
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
                "gate_input_topic", default_value="/safety/follower/arm_trajectory_in"
            ),
            DeclareLaunchArgument(
                "gate_output_topic",
                default_value="/follower/arm_trajectory_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "joint_states_topic", default_value="/follower/joint_states"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            monitor,
            safety_gate,
        ]
    )
