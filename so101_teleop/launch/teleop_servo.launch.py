"""Leader -> MoveIt Servo (JointJog) teleop.

Drop-in replacement for ``sim_teleop.launch.py`` that routes the leader arm
through MoveIt Servo instead of the direct-copy ``teleop_split`` bridge. Starts:

  1. ``leader_servo_jog`` - reads /leader/joint_states and /follower/joint_states
     and publishes control_msgs/JointJog to the Servo node (JOINT_JOG mode),
     which streams position setpoints to the follower arm_forward_controller.
  2. ``leader_sim_keyboard`` (optional) - drives the simulated leader arm.
  3. ``safety_pause_bridge`` (optional) - bridges /safety/protective_stop to the
     Servo pause_servo service so a perception protective-stop halts teleop.

Prerequisites (start these first, in separate terminals):
  * a follower running ``arm_forward_controller`` (real or sim),
  * the Servo node: ``ros2 launch so101_moveit_config servo.launch.py`` (add
    ``use_sim_time:=true`` in simulation),
  * a leader publishing /leader/joint_states (real leader, or the Gazebo
    leader+follower sim: ``gazebo_teleop_sim.launch.py launch_teleop:=false
    follower_arm_controller:=arm_forward_controller``).

Only one process may command the follower position interface, so this runs
INSTEAD of ``teleop_split`` / ``sim_teleop.launch.py``.

The leader keyboard needs an interactive TTY, which ``ros2 launch`` does not
provide, so by default it opens in its own xterm (keyboard_prefix:="xterm -e").
Without xterm, set launch_keyboard:=false and run it yourself:

  ros2 run so101_teleop leader_sim_keyboard --ros-args \
    --params-file $(ros2 pkg prefix so101_teleop)/share/so101_teleop/config/leader_sim_keyboard.yaml \
    -p use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    leader_topic = LaunchConfiguration("leader_topic")
    follower_topic = LaunchConfiguration("follower_topic")
    jog_topic = LaunchConfiguration("jog_topic")
    switch_service = LaunchConfiguration("switch_service")
    forward_gripper = LaunchConfiguration("forward_gripper")
    kp = LaunchConfiguration("kp")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_keyboard = LaunchConfiguration("launch_keyboard")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    launch_safety = LaunchConfiguration("launch_safety")
    safety_stop_topic = LaunchConfiguration("safety_stop_topic")
    pause_service = LaunchConfiguration("pause_service")

    adapter = Node(
        package="so101_teleop",
        executable="leader_servo_jog",
        name="leader_servo_jog",
        output="screen",
        parameters=[
            {
                "leader_topic": leader_topic,
                "follower_topic": follower_topic,
                "jog_topic": jog_topic,
                "switch_service": switch_service,
                "forward_gripper": forward_gripper,
                "kp": kp,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    keyboard = Node(
        package="so101_teleop",
        executable="leader_sim_keyboard",
        name="leader_sim_keyboard",
        output="screen",
        prefix=keyboard_prefix,
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("so101_teleop"), "config", "leader_sim_keyboard.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_keyboard),
    )

    safety = Node(
        package="so101_teleop",
        executable="safety_pause_bridge",
        name="safety_pause_bridge",
        output="screen",
        parameters=[
            {
                "safety_stop_topic": safety_stop_topic,
                "pause_service": pause_service,
                "use_sim_time": use_sim_time,
            },
        ],
        condition=IfCondition(launch_safety),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("leader_topic", default_value="/leader/joint_states"),
            DeclareLaunchArgument("follower_topic", default_value="/follower/joint_states"),
            DeclareLaunchArgument(
                "jog_topic", default_value="/follower/servo_node/delta_joint_cmds"
            ),
            DeclareLaunchArgument(
                "switch_service",
                default_value="/follower/servo_node/switch_command_type",
            ),
            DeclareLaunchArgument("forward_gripper", default_value="true"),
            DeclareLaunchArgument(
                "kp",
                default_value="10.0",
                description="Proportional gain: leader-follower position error -> unitless jog. "
                "Higher = snappier follow (saturates command at smaller error); "
                "too high can overshoot/jitter near target.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_keyboard", default_value="true"),
            DeclareLaunchArgument("keyboard_prefix", default_value="xterm -e"),
            DeclareLaunchArgument(
                "launch_safety",
                default_value="true",
                description="Start safety_pause_bridge to halt Servo on /safety/protective_stop.",
            ),
            DeclareLaunchArgument(
                "safety_stop_topic", default_value="/safety/protective_stop"
            ),
            DeclareLaunchArgument(
                "pause_service", default_value="/follower/servo_node/pause_servo"
            ),
            adapter,
            keyboard,
            safety,
        ]
    )
