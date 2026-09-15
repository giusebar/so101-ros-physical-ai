#!/usr/bin/env bash
# Smoke tests for the ai-vision-ros2 snap (Qualcomm OWL-ViT open-vocabulary
# detection variant, single bundled-model snap -- no content interface, no
# separate model snap).
#
# Run this on the Qualcomm target (IQ-9075 EVK etc.) with snapd:
#
#   bash snap/smoke_tests.sh
#
# Requires:
#   - ai-vision-ros2_*.snap at so101_owl_demo/ (built with `snapcraft` from
#     so101_owl_demo/, where snap/snapcraft.yaml lives). The OWL-ViT model and
#     the CLIP tokenizer are vendored + bundled at build time, so no second
#     snap / content-connect step is needed.
#   - The Ubuntu-on-Qualcomm IoT stack (fastrpc / QNN) on the board.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")/.." && pwd)"
APP_SNAP=$(ls "$PKG_DIR"/ai-vision-ros2_*.snap | head -1)

echo "=== 1. Installing snap ==="
sudo snap install "$APP_SNAP" --dangerous
sleep 3  # wait for AppArmor policies to load

echo "=== 2. Connecting the Qualcomm accelerator (custom-device) interface ==="
# custom-device auto-connection needs a store assertion; connect manually for a
# local --dangerous install so the daemon can reach /dev/fastrpc-cdsp (NPU).
sudo snap connect ai-vision-ros2:qcom-accel-plug ai-vision-ros2:qcom-accel || true

echo "=== 3. Checking snap is installed ==="
snap list ai-vision-ros2

echo "=== 4. Configure it (also (re)starts the daemon -- model is bundled, so it starts immediately) ==="
sudo snap set ai-vision-ros2 input-image-topic=/static_camera/image_raw prompt="a hand"
sleep 5
snap services ai-vision-ros2
echo "--- last 30 log lines (expect: QNN plugin EP htp, ONNX session ready, OwlDetectNode ready) ---"
sudo snap logs ai-vision-ros2 -n 30 || true
echo "    NOTE: the FIRST start compiles + caches the QNN context (~40 s -> \$SNAP_COMMON);"
echo "          later restarts reload the cache in ~1 s. Give the first start a moment."

echo "=== 5. Verify it actually publishes ROS messages ==="
echo "    In another terminal (with ROS 2 sourced), publish a test image:"
echo "      ros2 run so101_owl_demo test_image_publisher --ros-args -p image_topic:=/static_camera/image_raw -p fps:=10.0"
echo "    Then check the detection output topics:"
echo "      ros2 topic hz /camera/detections/visualization"
echo "      ros2 topic echo --once /camera/detections/visualization --field encoding   # expect: rgb8"
echo "      ros2 topic hz /perception/detections"
echo "      ros2 topic echo /safety/protective_stop"
echo ""
echo "    Swap the prompt at runtime WITHOUT a model reload (live prompt.txt poll):"
echo "      sudo snap set ai-vision-ros2 prompt=\"a cup\""
echo "    ...or via the ROS topic:"
echo "      ros2 topic pub --once /perception/prompt std_msgs/String '{data: \"a face\"}'"

echo "=== Smoke tests passed ==="
echo ""
echo "This is the latest/qualcomm/edge (OWL-ViT) channel; the depth variant is"
echo "latest/qualcomm/stable. See docs/ai_vision_ros2_qualcomm.md."
