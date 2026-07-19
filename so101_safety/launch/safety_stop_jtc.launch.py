"""trajectory_safety_gate (JointTrajectory/JTC path).

Freezes the follower's JointTrajectory stream whenever
/safety/protective_stop is asserted. This node is perception-agnostic -- it
doesn't care what published the stop signal or why. For the ai-vision-ros2
single-snap demo, that's the currently installed AI backend itself
(so101_depth_demo's depth_anything_node or so101_yolo_demo's
yolo_detect_node, each computing and publishing this topic directly) -- so
swapping `snap refresh ai-vision-ros2 --channel=...` swaps the whole trigger
logic with ZERO restart needed here.

Run:
  ros2 launch so101_safety safety_stop_jtc.launch.py
  ros2 launch so101_safety safety_stop_jtc.launch.py safety_stop_topic:=/safety/protective_stop
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    safety_gate = Node(
        package="so101_safety",
        executable="trajectory_safety_gate",
        name="trajectory_safety_gate",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("gate_input_topic"),
                "output_topic": LaunchConfiguration("gate_output_topic"),
                "safety_stop_topic": LaunchConfiguration("safety_stop_topic"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
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
            safety_gate,
        ]
    )
