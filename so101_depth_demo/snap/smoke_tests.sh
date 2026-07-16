#!/usr/bin/env bash
# Smoke tests for the depthanything (+ depthanything-model) snap pair.
#
# Run this on a machine with snapd:
#
#   bash snap/smoke_tests.sh
#
# Requires:
#   - depthanything_*.snap        at so101_depth_demo/ (built with `snapcraft`
#                                  from so101_depth_demo/, where
#                                  snap/snapcraft.yaml lives)
#   - depthanything-model_*.snap  at <repo-root>/snaps/depthanything-model/
#                                  (built with `snapcraft` from that directory)
#
# For --dangerous local installs the content interface does NOT auto-connect
# (that only happens for store installs via `default-provider`), so this
# script connects it manually.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")/.." && pwd)"
REPO_ROOT="$(cd "$PKG_DIR/.." && pwd)"
MODEL_SNAP=$(ls "$REPO_ROOT"/snaps/depthanything-model/*.snap | head -1)
APP_SNAP=$(ls "$PKG_DIR"/depthanything_*.snap | head -1)

echo "=== 1. Installing snaps ==="
sudo snap install "$MODEL_SNAP" --dangerous
sudo snap install "$APP_SNAP" --dangerous
sleep 3  # wait for AppArmor policies to load

echo "=== 2. Checking snaps are installed ==="
snap list depthanything-model
snap list depthanything

echo "=== 3. Before connecting the content interface: daemon should be blocked ==="
snap connections depthanything
echo "--- health (expect: blocked, model not found) ---"
snap changes | tail -3 || true

echo "=== 4. Connect the content interface (mounts the ONNX weights) ==="
sudo snap connect depthanything:depthanything-model depthanything-model:depthanything-model
sleep 2

echo "=== 5. Re-run configure so the daemon picks up the now-mounted model ==="
sudo snap set depthanything input-image-topic=/follower/image_raw
sleep 2
snap services depthanything
echo "--- last 20 log lines (expect: ONNX session ready, DepthAnythingNode ready) ---"
sudo snap logs depthanything -n 20 || true

echo "=== 6. Verify it actually publishes ROS messages ==="
echo "    In another terminal (with ROS 2 sourced), publish a test image:"
echo "      ros2 run so101_depth_demo test_image_publisher --ros-args -p image_topic:=/follower/image_raw -p fps:=15.0"
echo "    Then check the depth output topic:"
echo "      ros2 topic hz /camera/depth/visualization"
echo "      ros2 topic echo --once /camera/depth/visualization --field encoding   # expect: rgb8"
echo ""
echo "    Reconfigure at runtime, e.g. to change topics or add a namespace:"
echo "      sudo snap set depthanything output-image-topic=/camera/depth/vis namespace=/perception"

echo "=== Smoke tests (steps 1-5) passed ==="
