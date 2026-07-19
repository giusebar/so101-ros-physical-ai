#!/usr/bin/env bash
# Smoke tests for the ai-vision-ros2 snap (detection variant, single
# bundled-model snap -- no content interface, no separate model snap).
#
# Run this on a machine with snapd:
#
#   bash snap/smoke_tests.sh
#
# Requires:
#   - ai-vision-ros2_*.snap  at so101_yolo_demo/ (built with `snapcraft`
#                            from so101_yolo_demo/, where snap/snapcraft.yaml
#                            lives). The model (models/yolov8n.onnx) is
#                            vendored + bundled at build time, so no second
#                            snap/content-connect step is needed.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")/.." && pwd)"
APP_SNAP=$(ls "$PKG_DIR"/ai-vision-ros2_*.snap | head -1)

echo "=== 1. Installing snap ==="
sudo snap install "$APP_SNAP" --dangerous
sleep 3  # wait for AppArmor policies to load

echo "=== 2. Checking snap is installed ==="
snap list ai-vision-ros2

echo "=== 3. Configure it (also (re)starts the daemon -- the model is bundled, so it starts immediately, no content-interface connect step needed) ==="
sudo snap set ai-vision-ros2 input-image-topic=/static_camera/image_raw
sleep 2
snap services ai-vision-ros2
echo "--- last 20 log lines (expect: ONNX session ready, YoloDetectNode ready) ---"
sudo snap logs ai-vision-ros2 -n 20 || true

echo "=== 4. Verify it actually publishes ROS messages ==="
echo "    In another terminal (with ROS 2 sourced), publish a test image:"
echo "      ros2 run so101_yolo_demo test_image_publisher --ros-args -p image_topic:=/static_camera/image_raw -p fps:=15.0"
echo "    Then check the detection output topics:"
echo "      ros2 topic hz /camera/detections/visualization"
echo "      ros2 topic echo --once /camera/detections/visualization --field encoding   # expect: rgb8"
echo "      ros2 topic hz /perception/detections"
echo ""
echo "    Reconfigure at runtime, e.g. to change classes or add a namespace:"
echo "      sudo snap set ai-vision-ros2 class-filter=person,dog conf-threshold=0.5 namespace=/perception"

echo "=== Smoke tests passed ==="
echo ""
echo "To test the channel-swap (depth) variant instead, see:"
echo "  docs/ai_vision_ros2_channel_demo.md"
