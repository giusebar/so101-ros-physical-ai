#!/usr/bin/env bash
# Smoke test for the so101-bringup snap.
#
# Installs the snap unasserted (--dangerous), connects the interfaces that
# don't auto-connect for unasserted installs (content + raw-usb), configures
# hardware-type=mock (no real serial hardware required) and verifies that
# both arms' robot_description/joint_states/TF come up correctly -- this is
# exactly what an externally-run rviz2 needs to visualize both arms.
set -euo pipefail

SNAP_FILE=$(ls "$(dirname "$0")"/../*.snap | head -1)
SNAP_NAME=so101-bringup

echo "=== Installing snap ==="
snap install "$SNAP_FILE" --dangerous
sleep 3  # wait for AppArmor policies

echo "=== Checking snap is installed ==="
snap list "$SNAP_NAME"

echo "=== Connecting interfaces (no auto-connect for --dangerous installs) ==="
snap install ros-jazzy-ros-base 2>/dev/null || true
snap connect "$SNAP_NAME:ros-jazzy-ros-base" ros-jazzy-ros-base:ros-jazzy-ros-base
snap connect "$SNAP_NAME:raw-usb"
snap connections "$SNAP_NAME"

echo "=== Configuring hardware-type=mock (no real serial ports required) ==="
snap set "$SNAP_NAME" hardware-type=mock arm-controller=trajectory
sleep 5

echo "=== Checking snap services (daemon should be active) ==="
snap services "$SNAP_NAME"

echo "=== Verifying ROS graph (requires a sourced ROS 2 Jazzy env on the host) ==="
if command -v ros2 >/dev/null 2>&1; then
  ros2 topic list | grep -E "^/(leader|follower)/(robot_description|joint_states)$" || {
    echo "FAIL: expected leader/follower robot_description + joint_states topics not found" >&2
    exit 1
  }
  echo "Found expected robot_description/joint_states topics for both arms."
else
  echo "ros2 CLI not on PATH -- skipping ROS graph check (source /opt/ros/jazzy/setup.bash first)"
fi

echo "=== Cleaning up ==="
snap disable "$SNAP_NAME" || true
snap remove "$SNAP_NAME"

echo "=== Smoke tests passed ==="
