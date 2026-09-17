"""Launch the OWL-ViT open-vocabulary detection demo (detect node only).

Defaults target the Qualcomm Hexagon NPU (QNN HTP execution provider). Override
``execution_providers`` to ``CPUExecutionProvider`` to run without QNN.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = os.path.expanduser("~/models/owl_vit/owl_vit.onnx")
    default_tok = os.path.expanduser("~/models/owl_vit/tokenizer")
    args = [
        DeclareLaunchArgument(
            "model_path",
            default_value=default_model,
            description="Path to the OWL-ViT .onnx file (w8a16, static shapes).",
        ),
        DeclareLaunchArgument(
            "input_image_topic",
            default_value="/static_camera/image_raw",
            description="Camera image topic to run detection on.",
        ),
        DeclareLaunchArgument(
            "use_compressed",
            default_value="true",
            description="Subscribe to <input_image_topic>/compressed "
            "(sensor_msgs/CompressedImage, JPEG/PNG) instead of the raw "
            "sensor_msgs/Image. Less DDS/transport load than raw frames.",
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
        DeclareLaunchArgument("input_size", default_value="768"),
        DeclareLaunchArgument(
            "prompt",
            default_value="a hand",
            description="Free-form text prompt to detect (open-vocabulary).",
        ),
        DeclareLaunchArgument("prompt_topic", default_value="/perception/prompt"),
        DeclareLaunchArgument("prompt_file", default_value=""),
        DeclareLaunchArgument("score_threshold", default_value="0.1"),
        DeclareLaunchArgument("iou_threshold", default_value="0.3"),
        DeclareLaunchArgument("max_detections", default_value="20"),
        DeclareLaunchArgument("tokenizer_dir", default_value=default_tok),
        DeclareLaunchArgument("max_text_len", default_value="16"),
        DeclareLaunchArgument(
            "min_period_s",
            default_value="0.0",
            description="Minimum seconds between inferences (throttle).",
        ),
        DeclareLaunchArgument("intra_op_threads", default_value="4"),
        DeclareLaunchArgument(
            "execution_providers",
            default_value="['QNNExecutionProvider', 'CPUExecutionProvider']",
        ),
        DeclareLaunchArgument("qnn_backend_path", default_value="htp"),
        DeclareLaunchArgument(
            "qnn_context_cache_path",
            default_value="",
            description="EPContext cache file ('' = disabled). Persists the "
            "compiled QNN context so restarts skip the ~40 s graph compile.",
        ),
        DeclareLaunchArgument("log_severity_level", default_value="2"),
        DeclareLaunchArgument(
            "stop_topic",
            default_value="/safety/protective_stop",
            description="Bool protective-stop output, computed directly by "
            "this node from ROI overlap + hysteresis debounce over the "
            "prompt-matched detections.",
        ),
        DeclareLaunchArgument("roi", default_value="0.25,0.2,0.75,0.85"),
        DeclareLaunchArgument("min_overlap_ratio", default_value="0.2"),
        DeclareLaunchArgument("frames_to_block", default_value="2"),
        DeclareLaunchArgument("frames_to_clear", default_value="3"),
    ]

    detect_node = Node(
        package="so101_owl_demo",
        executable="owl_detect_node",
        name="owl_detect_node",
        output="screen",
        parameters=[
            {
                "model_path": LaunchConfiguration("model_path"),
                "input_image_topic": LaunchConfiguration("input_image_topic"),
                "use_compressed": LaunchConfiguration("use_compressed"),
                "output_image_topic": LaunchConfiguration("output_image_topic"),
                "output_detections_topic": LaunchConfiguration("output_detections_topic"),
                "input_size": LaunchConfiguration("input_size"),
                "prompt": LaunchConfiguration("prompt"),
                "prompt_topic": LaunchConfiguration("prompt_topic"),
                "prompt_file": LaunchConfiguration("prompt_file"),
                "score_threshold": LaunchConfiguration("score_threshold"),
                "iou_threshold": LaunchConfiguration("iou_threshold"),
                "max_detections": LaunchConfiguration("max_detections"),
                "tokenizer_dir": LaunchConfiguration("tokenizer_dir"),
                "max_text_len": LaunchConfiguration("max_text_len"),
                "min_period_s": LaunchConfiguration("min_period_s"),
                "intra_op_threads": LaunchConfiguration("intra_op_threads"),
                "qnn_backend_path": LaunchConfiguration("qnn_backend_path"),
                "qnn_context_cache_path": LaunchConfiguration("qnn_context_cache_path"),
                "log_severity_level": LaunchConfiguration("log_severity_level"),
                "stop_topic": LaunchConfiguration("stop_topic"),
                "roi": LaunchConfiguration("roi"),
                "min_overlap_ratio": LaunchConfiguration("min_overlap_ratio"),
                "frames_to_block": LaunchConfiguration("frames_to_block"),
                "frames_to_clear": LaunchConfiguration("frames_to_clear"),
            }
        ],
    )

    return LaunchDescription(args + [detect_node])
