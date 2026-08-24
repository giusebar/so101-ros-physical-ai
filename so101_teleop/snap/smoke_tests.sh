#!/usr/bin/env bash
# Smoke test for the so101-teleop snap.
#
# Build first with (ros2-jazzy-ros-base is an experimental extension, and
# snapcraft silently exits 1 with no visible error if this isn't set):
#
#   SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS=1 snapcraft pack
#
# Installs the snap unasserted (--dangerous), configures it, publishes a fake
# leader JointState (so101-bringup is not required for this test), and
# verifies teleop_split actually republishes a JointTrajectory on the
# follower's trajectory-controller topic -- not just that the node starts.
#
# Requires a ROS 2 Jazzy `ros2` CLI on the host (either sourced from
# /opt/ros/jazzy/setup.bash, or the `ros2-cli` snap).
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SNAP_FILE=$(ls "$PKG_DIR"/so101-teleop_*.snap | head -1)
SNAP_NAME=so101-teleop

echo "=== Installing snap ==="
snap install "$SNAP_FILE" --dangerous
sleep 3  # wait for AppArmor policies

echo "=== Checking snap is installed ==="
snap list "$SNAP_NAME"

echo "=== Connecting content interface (no auto-connect for --dangerous installs) ==="
snap install ros-jazzy-ros-base 2>/dev/null || true
snap connect "$SNAP_NAME:ros-jazzy-ros-base" ros-jazzy-ros-base:ros-jazzy-ros-base || true
snap connections "$SNAP_NAME"

echo "=== Configuring (defaults are used; matches so101-bringup's defaults) ==="
snap set "$SNAP_NAME" arm-mode=joint_trajectory gripper-mode=parallel_action
sleep 3

echo "=== Checking snap services (daemon should be active) ==="
snap services "$SNAP_NAME"
echo "--- last 20 log lines ---"
snap logs "$SNAP_NAME" -n 20 || true

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI not on PATH -- skipping ROS graph check (source /opt/ros/jazzy/setup.bash first)"
  echo "=== Cleaning up ==="
  snap disable "$SNAP_NAME" || true
  snap remove "$SNAP_NAME"
  exit 0
fi

echo "=== Publishing a fake leader JointState ==="
ros2 topic pub -r 20 /leader/joint_states sensor_msgs/msg/JointState \
  '{name: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper], position: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]}' \
  >/tmp/so101-teleop-pub.log 2>&1 &
PUB_PID=$!
trap 'kill "$PUB_PID" 2>/dev/null || true' EXIT

echo "=== Verifying teleop_split republishes a JointTrajectory (data plane, not just discovery) ==="
# so101-safety is not installed here, so the `safety-route` content interface is
# disconnected and teleop auto-routes STRAIGHT at the controller topic below.
# (With so101-safety installed + connected it would instead publish to the
# gate's input topic, /safety/follower/arm_trajectory_in.)
timeout 8 ros2 topic echo /follower/arm_trajectory_controller/joint_trajectory --once \
  >/tmp/so101-teleop-echo.out 2>&1 || true
if [ ! -s /tmp/so101-teleop-echo.out ]; then
  echo "FAIL: no message received on /follower/arm_trajectory_controller/joint_trajectory within 8s" >&2
  snap logs "$SNAP_NAME" -n 40 || true
  exit 1
fi
cat /tmp/so101-teleop-echo.out
rm -f /tmp/so101-teleop-echo.out /tmp/so101-teleop-pub.log

kill "$PUB_PID" 2>/dev/null || true
trap - EXIT

echo "=== Cleaning up ==="
snap disable "$SNAP_NAME" || true
snap remove "$SNAP_NAME"

echo "=== Smoke tests passed ==="
