#!/usr/bin/env bash
# Smoke test for the so101-safety snap (mode=jtc / trajectory_safety_gate).
#
# Build first with (ros2-jazzy-ros-base is an experimental extension, and
# snapcraft silently exits 1 with no visible error if this isn't set):
#
#   SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS=1 snapcraft pack
#
# Installs the snap unasserted (--dangerous), configures mode=jtc, publishes
# a fake follower JointState + an input JointTrajectory, and verifies:
#   1. while clear (safety-stop-topic=false / no message yet), the gate
#      passes the trajectory straight through to gate-output-topic.
#   2. once a protective_stop=true is published, the gate stops forwarding
#      new trajectories and instead holds the last follower pose.
#
# Requires a ROS 2 Jazzy `ros2` CLI on the host (either sourced from
# /opt/ros/jazzy/setup.bash, or the `ros2-cli` snap).
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SNAP_FILE=$(ls "$PKG_DIR"/so101-safety_*.snap | head -1)
SNAP_NAME=so101-safety

cleanup() {
  kill "${JOINT_STATE_PUB_PID:-}" 2>/dev/null || true
  kill "${TRAJ_PUB_PID:-}" 2>/dev/null || true
  snap disable "$SNAP_NAME" >/dev/null 2>&1 || true
  snap remove "$SNAP_NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "=== Installing snap ==="
snap install "$SNAP_FILE" --dangerous
sleep 3  # wait for AppArmor policies

echo "=== Checking snap is installed ==="
snap list "$SNAP_NAME"

echo "=== Connecting content interface (no auto-connect for --dangerous installs) ==="
snap install ros-jazzy-ros-base 2>/dev/null || true
snap connect "$SNAP_NAME:ros-jazzy-ros-base" ros-jazzy-ros-base:ros-jazzy-ros-base || true
snap connections "$SNAP_NAME"

echo "=== Configuring mode=jtc (default topics) ==="
snap set "$SNAP_NAME" mode=jtc
sleep 3

echo "=== Checking snap services (daemon should be active) ==="
snap services "$SNAP_NAME"
echo "--- last 20 log lines ---"
snap logs "$SNAP_NAME" -n 20 || true

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI not on PATH -- skipping ROS graph check"
  exit 0
fi

echo "=== Publishing a fake follower JointState (needed for the hold trajectory) ==="
# trajectory_safety_gate subscribes to joint_states with SensorDataQoS
# (best-effort) -- ros2 topic pub defaults to reliable, which is
# incompatible and silently drops all messages, so force best_effort here.
ros2 topic pub -r 20 --qos-reliability best_effort /follower/joint_states sensor_msgs/msg/JointState \
  '{name: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], position: [0.0, 0.0, 0.0, 0.0, 0.0]}' \
  >/tmp/so101-safety-jointstate.log 2>&1 &
JOINT_STATE_PUB_PID=$!

echo "=== Publishing a fake input JointTrajectory ==="
ros2 topic pub -r 20 /safety/follower/arm_trajectory_in trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], points: [{positions: [0.1, 0.2, 0.3, 0.4, 0.5], time_from_start: {sec: 0, nanosec: 80000000}}]}' \
  >/tmp/so101-safety-traj.log 2>&1 &
TRAJ_PUB_PID=$!

echo "=== 1. Verifying the gate passes the trajectory through while clear ==="
timeout 8 ros2 topic echo /follower/arm_trajectory_controller/joint_trajectory --once \
  >/tmp/so101-safety-echo1.out 2>&1 || true
if [ ! -s /tmp/so101-safety-echo1.out ]; then
  echo "FAIL: no message received on /follower/arm_trajectory_controller/joint_trajectory within 8s (gate should pass through while clear)" >&2
  snap logs "$SNAP_NAME" -n 40 || true
  exit 1
fi
cat /tmp/so101-safety-echo1.out
echo "Confirmed: trajectory passes through while clear."

echo "=== 2. Publishing protective_stop=true and verifying the gate holds ==="
# Stop the continuous input-trajectory publisher first -- otherwise the
# passthrough messages keep flowing at 20Hz and `ros2 topic echo --once`
# will very likely grab one of those instead of the (single, edge-triggered)
# hold trajectory, giving a false pass.
kill "$TRAJ_PUB_PID" 2>/dev/null || true
unset TRAJ_PUB_PID
sleep 1

# trajectory_safety_gate only publishes the hold trajectory ONCE, at the
# moment it transitions to blocked (not continuously) -- so start listening
# *before* triggering the stop, or `ros2 topic echo --once` can easily race
# past the single message and hang until timeout.
rm -f /tmp/so101-safety-echo2.out
timeout 8 ros2 topic echo /follower/arm_trajectory_controller/joint_trajectory --once \
  >/tmp/so101-safety-echo2.out 2>&1 &
ECHO_PID=$!
sleep 1
ros2 topic pub --once /safety/protective_stop std_msgs/msg/Bool '{data: true}' >/dev/null
wait "$ECHO_PID" || true

if [ ! -s /tmp/so101-safety-echo2.out ]; then
  echo "FAIL: no hold trajectory received after protective_stop=true (expected a held-pose trajectory)" >&2
  snap logs "$SNAP_NAME" -n 40 || true
  exit 1
fi
echo "--- hold trajectory (positions should be [0,0,0,0,0], the fake follower pose -- NOT [0.1,0.2,0.3,0.4,0.5]) ---"
cat /tmp/so101-safety-echo2.out
if grep -q -- "- 0.1" /tmp/so101-safety-echo2.out; then
  echo "FAIL: received the moving input trajectory, not a held pose -- the gate did not block" >&2
  exit 1
fi
echo "Confirmed: gate published a hold trajectory (follower's last pose) after protective_stop=true, not the blocked input."

rm -f /tmp/so101-safety-echo1.out /tmp/so101-safety-echo2.out /tmp/so101-safety-jointstate.log /tmp/so101-safety-traj.log

echo "=== Smoke tests passed ==="
