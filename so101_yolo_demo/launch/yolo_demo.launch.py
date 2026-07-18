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
            description="vision_msgs/Detection2DArray topic for downstream "
            "consumers, e.g. so101_safety's person_safety_monitor.",
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
            }
        ],
    )

    return LaunchDescription(args + [detect_node])
