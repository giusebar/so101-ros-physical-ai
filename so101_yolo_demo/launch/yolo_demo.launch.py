"""Launch the CPU YOLOv8n object/person detection demo (detect node only)."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = os.path.expanduser("~/models/yolov8n.onnx")
    args = [
        DeclareLaunchArgument(
            "model_path",
            default_value=default_model,
            description="Path to the yolov8n .onnx file (NMS baked in).",
        ),
        DeclareLaunchArgument(
            "input_image_topic",
            default_value="/static_camera/image_raw",
            description="Camera image topic to run detection on.",
        ),
        DeclareLaunchArgument(
            "output_image_topic",
            default_value="/camera/detections/visualization",
            description="Annotated detection visualisation topic to publish.",
        ),
        DeclareLaunchArgument(
            "output_detections_topic",
            default_value="/perception/detections",
            description="vision_msgs/Detection2DArray topic for other "
            "machine consumers.",
        ),
        DeclareLaunchArgument("input_size", default_value="640"),
        DeclareLaunchArgument("conf_threshold", default_value="0.4"),
        DeclareLaunchArgument(
            "class_filter",
            default_value="person",
            description="Comma-separated COCO class ids or names to keep "
            "(empty = keep all 80 classes).",
        ),
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
            "this node from ROI overlap + hysteresis debounce over the "
            "already class/confidence-filtered detections.",
        ),
        DeclareLaunchArgument("roi", default_value="0.25,0.2,0.75,0.85"),
        DeclareLaunchArgument("min_overlap_ratio", default_value="0.2"),
        DeclareLaunchArgument("frames_to_block", default_value="2"),
        DeclareLaunchArgument("frames_to_clear", default_value="3"),
    ]

    detect_node = Node(
        package="so101_yolo_demo",
        executable="yolo_detect_node",
        name="yolo_detect_node",
        output="screen",
        parameters=[
            {
                "model_path": LaunchConfiguration("model_path"),
                "input_image_topic": LaunchConfiguration("input_image_topic"),
                "output_image_topic": LaunchConfiguration("output_image_topic"),
                "output_detections_topic": LaunchConfiguration("output_detections_topic"),
                "input_size": LaunchConfiguration("input_size"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "class_filter": LaunchConfiguration("class_filter"),
                "min_period_s": LaunchConfiguration("min_period_s"),
                "intra_op_threads": LaunchConfiguration("intra_op_threads"),
                "stop_topic": LaunchConfiguration("stop_topic"),
                "roi": LaunchConfiguration("roi"),
                "min_overlap_ratio": LaunchConfiguration("min_overlap_ratio"),
                "frames_to_block": LaunchConfiguration("frames_to_block"),
                "frames_to_clear": LaunchConfiguration("frames_to_clear"),
            }
        ],
    )

    return LaunchDescription(args + [detect_node])
