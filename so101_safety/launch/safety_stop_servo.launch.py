"""safety_pause_bridge (MoveIt Servo path).

Pauses/resumes MoveIt Servo whenever /safety/protective_stop is asserted.
This node is perception-agnostic -- it doesn't care what published the stop
signal or why. For the ai-vision-ros2 single-snap demo, that's the currently
installed AI backend itself (so101_depth_demo's depth_anything_node or
so101_yolo_demo's yolo_detect_node, each computing and publishing this topic
directly) -- so swapping `snap refresh ai-vision-ros2 --channel=...` swaps
the whole trigger logic with ZERO restart needed here.

Run:
  ros2 launch so101_safety safety_stop_servo.launch.py
  ros2 launch so101_safety safety_stop_servo.launch.py safety_stop_topic:=/safety/protective_stop
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    safety_bridge = Node(
        package="so101_safety",
        executable="safety_pause_bridge",
        name="safety_pause_bridge",
        output="screen",
        parameters=[
            {
                "safety_stop_topic": LaunchConfiguration("safety_stop_topic"),
                "pause_service": LaunchConfiguration("pause_service"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "safety_stop_topic", default_value="/safety/protective_stop"
            ),
            DeclareLaunchArgument(
                "pause_service", default_value="/follower/servo_node/pause_servo"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            safety_bridge,
        ]
    )
