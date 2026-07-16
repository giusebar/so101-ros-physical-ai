#!/usr/bin/env bash
# Smoke tests for the usb-cam snap.
#
# Run this on a machine with snapd (the build container here has no
# passwordless sudo, so these must be run interactively by a human):
#
#   bash snap/smoke_tests.sh
#
# IMPORTANT: step 5+ actually opens a real camera device. Only point
# `device=` at real hardware (e.g. /dev/video4) once you're ready to use it;
# a non-existent path (e.g. /dev/video99) is safe for steps 1-4 and will just
# make the daemon fail to open the device (visible in `snap logs`) without
# touching any hardware.
set -euo pipefail

SNAP_FILE=$(ls "$(dirname "$0")"/../*.snap | head -1)
SNAP_NAME=usb-cam

echo "=== 1. Installing snap ==="
sudo snap install "$SNAP_FILE" --dangerous
sleep 3  # wait for AppArmor policies to load

echo "=== 2. Checking snap is installed ==="
snap list "$SNAP_NAME"

echo "=== 3. Checking confinement + interfaces (camera should be strict/manual) ==="
snap connections "$SNAP_NAME"

echo "=== 4. Configuring with a non-existent device path (safe, no hardware touched) ==="
sudo snap set "$SNAP_NAME" device=/dev/video99 frame-id=cam_test camera-name=cam_test
sleep 2
snap services "$SNAP_NAME"
echo "--- last 20 log lines (expected: node fails to open /dev/video99, then restarts) ---"
sudo snap logs "$SNAP_NAME" -n 20 || true

echo "=== 5. Connect the camera interface (grants access to real /dev/video* nodes) ==="
echo "    Not auto-connected in strict confinement -- do this only when ready to use real hardware:"
echo "    sudo snap connect $SNAP_NAME:camera"

echo "=== 6. Point at a real camera and verify it publishes (requires explicit go-ahead + camera interface connected) ==="
echo "    sudo snap set $SNAP_NAME device=/dev/video4 frame-id=cam_overhead camera-name=cam_overhead namespace=/static_camera"
echo "    ros2 topic hz /static_camera/cam_overhead/image_raw"

echo "=== Smoke tests (steps 1-4) passed ==="
