"""Launch the CPU Depth Anything V2 demo (depth node + optional display)."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = os.path.expanduser("~/models/depth_anything_v2_small.onnx")
    args = [
        DeclareLaunchArgument(
            "model_path",
            default_value=default_model,
            description="Path to the Depth Anything V2 Small ONNX model.",
        ),
        DeclareLaunchArgument(
            "input_image_topic",
            default_value="/follower/image_raw",
            description="Camera image topic to run depth inference on.",
        ),
        DeclareLaunchArgument(
            "output_image_topic",
            default_value="/camera/depth/visualization",
            description="Colorised depth visualisation topic to publish.",
        ),
        DeclareLaunchArgument("model_input_size", default_value="518"),
        DeclareLaunchArgument("publish_width", default_value="518"),
        DeclareLaunchArgument("publish_height", default_value="518"),
        DeclareLaunchArgument(
            "min_period_s",
            default_value="0.0",
            description="Minimum seconds between inferences (CPU throttle).",
        ),
        DeclareLaunchArgument("intra_op_threads", default_value="0"),
        DeclareLaunchArgument(
            "use_display",
            default_value="false",
            description="Also start an OpenCV display window (needs X11).",
        ),
    ]

    depth_node = Node(
        package="so101_depth_demo",
        executable="depth_anything_node",
        name="depth_anything_node",
        output="screen",
        parameters=[
            {
                "model_path": LaunchConfiguration("model_path"),
                "input_image_topic": LaunchConfiguration("input_image_topic"),
                "output_image_topic": LaunchConfiguration("output_image_topic"),
                "model_input_size": LaunchConfiguration("model_input_size"),
                "publish_width": LaunchConfiguration("publish_width"),
                "publish_height": LaunchConfiguration("publish_height"),
                "min_period_s": LaunchConfiguration("min_period_s"),
                "intra_op_threads": LaunchConfiguration("intra_op_threads"),
            }
        ],
    )

    display_node = Node(
        package="so101_depth_demo",
        executable="depth_display_node",
        name="depth_display_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_display")),
        parameters=[{"image_topic": LaunchConfiguration("output_image_topic")}],
    )

    return LaunchDescription(args + [depth_node, display_node])
