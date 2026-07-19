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
        DeclareLaunchArgument(
            "output_depth_topic",
            default_value="/perception/depth",
            description="Raw normalised depth (32FC1) topic for other "
            "machine consumers.",
        ),
        # Multiple of 14 (ViT patch size); 308 runs ~2.8x faster than native 518.
        DeclareLaunchArgument("model_input_size", default_value="308"),
        DeclareLaunchArgument("publish_width", default_value="256"),
        DeclareLaunchArgument("publish_height", default_value="256"),
        DeclareLaunchArgument(
            "min_period_s",
            default_value="0.0",
            description="Minimum seconds between inferences (CPU throttle).",
        ),
        DeclareLaunchArgument("intra_op_threads", default_value="0"),
        DeclareLaunchArgument(
            "stop_topic",
            default_value="/safety/protective_stop",
            description="Bool protective-stop output, computed directly by "
            "this node from ROI proximity + hysteresis debounce.",
        ),
        DeclareLaunchArgument("roi", default_value="0.25,0.2,0.75,0.85"),
        DeclareLaunchArgument("near_threshold", default_value="0.6"),
        DeclareLaunchArgument("near_margin", default_value="0.15"),
        DeclareLaunchArgument("min_area_ratio", default_value="0.12"),
        DeclareLaunchArgument("frames_to_block", default_value="2"),
        DeclareLaunchArgument("frames_to_clear", default_value="3"),
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
                "output_depth_topic": LaunchConfiguration("output_depth_topic"),
                "model_input_size": LaunchConfiguration("model_input_size"),
                "publish_width": LaunchConfiguration("publish_width"),
                "publish_height": LaunchConfiguration("publish_height"),
                "min_period_s": LaunchConfiguration("min_period_s"),
                "intra_op_threads": LaunchConfiguration("intra_op_threads"),
                "stop_topic": LaunchConfiguration("stop_topic"),
                "roi": LaunchConfiguration("roi"),
                "near_threshold": LaunchConfiguration("near_threshold"),
                "near_margin": LaunchConfiguration("near_margin"),
                "min_area_ratio": LaunchConfiguration("min_area_ratio"),
                "frames_to_block": LaunchConfiguration("frames_to_block"),
                "frames_to_clear": LaunchConfiguration("frames_to_clear"),
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
