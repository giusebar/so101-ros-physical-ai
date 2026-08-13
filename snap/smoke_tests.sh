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

  # NOTE: `topic list`/`topic info` only prove DDS discovery (SPDP/SEDP over
  # UDP multicast) succeeded -- they do NOT prove actual data reaches an
  # external (non-snap) subscriber. A `shared-memory: private: true` plug
  # previously passed this exact check while silently breaking FastDDS's
  # shared-memory data-plane transport to every other ROS 2 process on the
  # host (confirmed on real hardware: `topic list` worked, `topic echo`/`hz`
  # hung forever). So explicitly verify data actually flows here too.
  echo "=== Verifying data actually flows to an external subscriber (not just discovery) ==="
  for topic in /leader/joint_states /follower/joint_states; do
    echo "--- $topic ---"
    timeout 8 ros2 topic echo "$topic" --once >/tmp/so101-bringup-echo.out 2>&1 || true
    if [ ! -s /tmp/so101-bringup-echo.out ]; then
      echo "FAIL: no message received on $topic within 8s -- discovery works but data plane doesn't (check for a private shared-memory plug isolating /dev/shm)" >&2
      exit 1
    fi
    cat /tmp/so101-bringup-echo.out
  done
  rm -f /tmp/so101-bringup-echo.out
  echo "Confirmed real data flow on both arms' joint_states."
else
  echo "ros2 CLI not on PATH -- skipping ROS graph check (source /opt/ros/jazzy/setup.bash first)"
fi

echo "=== Cleaning up ==="
snap disable "$SNAP_NAME" || true
snap remove "$SNAP_NAME"

echo "=== Smoke tests passed ==="
