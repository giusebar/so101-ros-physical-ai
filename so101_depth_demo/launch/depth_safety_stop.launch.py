"""Camera + Depth proximity + safety gate + viewer (CPU only).

Starts:
  - the V4L2 camera (default /dev/video5),
    - the CPU Depth Anything proximity node -> /safety/protective_stop + debug overlay,
    - the trajectory safety gate that freezes the follower when protective_stop,
  - rqt_image_view showing the camera + ROI overlay (red = STOP, green = clear).

This is the "safety" half of the demo. It expects the simulation and a teleop
bridge that publishes to /safety/follower/arm_trajectory_in to be running
already (see so101_bringup gazebo_teleop_sim.launch.py and so101_teleop
sim_teleop.launch.py), or just use so101_depth_demo full_demo.launch.py to start
everything at once.

Run:
  ros2 launch so101_depth_demo depth_safety_stop.launch.py
  ros2 launch so101_depth_demo depth_safety_stop.launch.py camera_device:=/dev/video0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_device = LaunchConfiguration("camera_device")
    image_topic = LaunchConfiguration("image_topic")
    model_path = LaunchConfiguration("model_path")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    safety_stop_topic = LaunchConfiguration("safety_stop_topic")

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="cam_overhead",
        namespace="static_camera",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("so101_bringup"),
                    "config",
                    "cameras",
                    "so101_v4l2_cam.yaml",
                ]
            ),
            {
                "video_device": camera_device,
                "camera_name": "cam_overhead",
                "frame_id": "cam_overhead",
                "use_sim_time": False,
            },
        ],
    )

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
                "near_margin": LaunchConfiguration("near_margin"),
                "min_area_ratio": LaunchConfiguration("min_area_ratio"),
                "inference_hz": LaunchConfiguration("inference_hz"),
                "publish_debug_image": True,
            }
        ],
    )

    safety_gate = Node(
        package="so101_teleop",
        executable="trajectory_safety_gate",
        name="trajectory_safety_gate",
        output="screen",
        parameters=[
            {
                "input_topic": "/safety/follower/arm_trajectory_in",
                "output_topic": "/follower/arm_trajectory_controller/joint_trajectory",
                "safety_stop_topic": safety_stop_topic,
                "joint_states_topic": "/follower/joint_states",
                "use_sim_time": True,
            }
        ],
    )

    viewer = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="depth_proximity_viewer",
        output="screen",
        arguments=[debug_image_topic],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_device", default_value="/dev/video5"),
            DeclareLaunchArgument("image_topic", default_value="/static_camera/image_raw"),
            DeclareLaunchArgument(
                "model_path",
                default_value=os.path.expanduser("~/models/depth_anything_v2_small.onnx"),
            ),
            DeclareLaunchArgument(
                "debug_image_topic", default_value="/safety/depth_debug_image"
            ),
            DeclareLaunchArgument(
                "safety_stop_topic", default_value="/safety/protective_stop"
            ),
            DeclareLaunchArgument("near_margin", default_value="0.15"),
            DeclareLaunchArgument("min_area_ratio", default_value="0.12"),
            DeclareLaunchArgument("inference_hz", default_value="10.0"),
            camera,
            depth_stop,
            safety_gate,
            viewer,
        ]
    )
