# so101_owl_demo

Open-vocabulary object detection for the SO-101 on the **Qualcomm Hexagon NPU**.

`owl_detect_node` subscribes to a camera image topic, runs Qualcomm AI-Hub's
**OWL-ViT** (open-vocabulary detector, ViT-B/32 CLIP backbone) with ONNX
Runtime's **QNN execution provider** on the Hexagon NPU (HTP backend), and
publishes:

- `<output_image_topic>` (`sensor_msgs/Image`, rgb8) — annotated boxes + the
  safety ROI/trigger overlay, for viewing.
- `<output_detections_topic>` (`vision_msgs/Detection2DArray`) — for machine
  consumers.
- `<stop_topic>` (`std_msgs/Bool`, default `/safety/protective_stop`) — computed
  directly by this node from ROI overlap + hysteresis debounce over the
  prompt-matched detections.

Unlike the fixed-class `so101_yolo_demo`, OWL-ViT detects whatever **free-form
text prompt** you give it (default `"a hand"`). The prompt is a live runtime
input (tokenised CLIP text) fed to the model with the image, so it can be
swapped without reloading the model — via the `/perception/prompt`
(`std_msgs/String`) topic or a polled `prompt_file` (`snap set prompt=...`).

This is the **Qualcomm/NPU counterpart** of the Jetson-only NanoOWL variant:
same camera input, same detection + `/safety/protective_stop` contract, same
ROI-overlap + hysteresis trigger — different accelerator (QNN HTP instead of
TensorRT/torch2trt). It is the `latest/qualcomm/edge` channel of the
`ai-vision-ros2` snap (the depth variant is `latest/qualcomm/stable`). See
`docs/ai_vision_ros2_qualcomm.md`.

## Model

Qualcomm AI-Hub **OWL-ViT**, w8a16-quantised, static shapes (image
`1x3x768x768` uint16, text `1x16` int32), quantised uint16 box/score outputs
(the node dequantises). Runs **fully on the Hexagon NPU** (~23 ms/inference on
the IQ-9075). NMS is **not** in the graph — the node runs it. The model +
the CLIP tokenizer are bundled in the snap at build time (no AI-Hub account,
no separate model snap). Apache-2.0.

> **QNN context caching:** the first-ever start compiles the QNN graph (~40 s)
> and writes an EPContext binary to `qnn_context_cache_path` (the snap points it
> at `$SNAP_COMMON`). Every subsequent start reloads that cache in ~1 s. A cache
> left over from an incompatible QNN version is auto-detected and regenerated.

## Run (native, outside the snap)

```bash
# model at ~/models/owl_vit/{owl_vit.onnx,owl_vit.data,metadata.json}
# tokenizer at ~/models/owl_vit/tokenizer/{vocab.json,merges.txt,...}
ros2 launch so101_owl_demo owl_demo.launch.py prompt:="a hand"
```

## Key parameters

| Param | Default | Meaning |
| --- | --- | --- |
| `prompt` | `a hand` | free-form text prompt to detect |
| `prompt_topic` | `/perception/prompt` | live prompt update (String) |
| `prompt_file` | `""` | polled file for live prompt update |
| `score_threshold` | `0.1` | min sigmoid score to keep |
| `iou_threshold` | `0.3` | NMS IoU threshold |
| `execution_providers` | `[QNN, CPU]` | ORT providers (priority order) |
| `qnn_backend_path` | `htp` | QNN accelerator (`htp`/`gpu`/`cpu`) |
| `qnn_context_cache_path` | `""` | EPContext cache file (snap: `$SNAP_COMMON`) |
| `roi` | `0.25,0.2,0.75,0.85` | normalised centre ROI |
| `min_overlap_ratio` | `0.2` | box-in-ROI fraction to trigger |
| `frames_to_block` / `frames_to_clear` | `2` / `3` | hysteresis windows |

The uint16 quant/dequant params (`input_quant_scale`, `box_dequant_scale`,
`score_dequant_scale`, …) default to the bundled model's `metadata.json` values.
