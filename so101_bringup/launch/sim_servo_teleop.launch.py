"""One-command SO-101 sim teleop through MoveIt Servo (JointJog).

Bundles the three terminals of the Servo-jog sim recipe into a single launch:

  1. ``gazebo_teleop_sim.launch.py`` - Gazebo with leader + follower arms. The
     follower runs ``arm_forward_controller`` (position streaming, what Servo
     writes to) and the built-in ``teleop_split`` bridge is disabled so it does
     not fight Servo for the follower position interface.
  2. ``so101_moveit_config servo.launch.py`` (use_sim_time:=true) - the Servo
     node in the /follower namespace, driven off /clock.
  3. ``so101_teleop teleop_servo.launch.py`` - the leader_servo_jog adapter
     (leader/follower JointState -> JointJog), the safety_pause_bridge, and
     (optionally) the leader keyboard in an xterm.

The stages are separated by TimerActions because Servo needs the follower
joint_state_broadcaster + arm_forward_controller up first, and the adapter needs
Servo up. Both the adapter and the safety bridge retry their Servo service calls,
so the exact timing is forgiving.

Usage:

  ros2 launch so101_bringup sim_servo_teleop.launch.py

  # headless / no xterm - run the leader keyboard yourself in a real TTY:
  ros2 launch so101_bringup sim_servo_teleop.launch.py launch_keyboard:=false

Requires use_sim_time everywhere: Servo drops commands as stale if it is not on
the Gazebo /clock (incoming_command_timeout ~0.1 s).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_keyboard = LaunchConfiguration("launch_keyboard")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    launch_safety = LaunchConfiguration("launch_safety")
    kp = LaunchConfiguration("kp")
    servo_start_delay = LaunchConfiguration("servo_start_delay")
    teleop_start_delay = LaunchConfiguration("teleop_start_delay")

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "launch", "gazebo_teleop_sim.launch.py"]
            )
        ),
        launch_arguments={
            # Servo owns the follower position interface; do not run the
            # direct-copy teleop bridge or the built-in keyboard here.
            "launch_teleop": "false",
            "launch_keyboard": "false",
            "follower_arm_controller": "arm_forward_controller",
        }.items(),
    )

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_moveit_config"), "launch", "servo.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_teleop"), "launch", "teleop_servo.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "launch_keyboard": launch_keyboard,
            "keyboard_prefix": keyboard_prefix,
            "launch_safety": launch_safety,
            "kp": kp,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_keyboard", default_value="true"),
            DeclareLaunchArgument("keyboard_prefix", default_value="xterm -e"),
            DeclareLaunchArgument("launch_safety", default_value="true"),
            DeclareLaunchArgument(
                "kp",
                default_value="10.0",
                description="leader_servo_jog proportional gain (live-tunable).",
            ),
            DeclareLaunchArgument(
                "servo_start_delay",
                default_value="8.0",
                description="Seconds to wait before starting Servo (follower "
                "controllers spawn ~4 s into the Gazebo launch).",
            ),
            DeclareLaunchArgument(
                "teleop_start_delay",
                default_value="11.0",
                description="Seconds to wait before starting the teleop adapter "
                "(needs Servo up; it retries the switch_command_type service).",
            ),
            gazebo_sim,
            TimerAction(period=servo_start_delay, actions=[servo]),
            TimerAction(period=teleop_start_delay, actions=[teleop]),
        ]
    )
