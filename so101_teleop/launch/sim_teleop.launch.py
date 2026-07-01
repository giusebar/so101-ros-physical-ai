"""Simulated SO-101 teleoperation.

Starts the teleop bridge (leader joint_states -> follower trajectory) together
with the leader keyboard so you can drive the simulated leader arm.

The leader keyboard needs an interactive terminal (stdin must be a TTY), which
`ros2 launch` does not provide. By default it is therefore launched inside its
own xterm window (keyboard_prefix:="xterm -e"). If you do not have xterm, set
launch_keyboard:=false and run it yourself in a spare terminal:

  ros2 run so101_teleop leader_sim_keyboard --ros-args \
    --params-file $(ros2 pkg prefix so101_teleop)/share/so101_teleop/config/leader_sim_keyboard.yaml \
    -p use_sim_time:=true

Arguments:
  jtc_topic         where the follower trajectory is published. Default is the
                    controller topic (plain teleop). For the safety-stop demo
                    set it to /safety/follower/arm_trajectory_in so a safety
                    gate can intercept it.
  launch_keyboard   start the leader keyboard (default true).
  keyboard_prefix   process prefix for the keyboard, default "xterm -e" so it
                    opens a window with a real TTY. Use "" to run inline.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    jtc_topic = LaunchConfiguration("jtc_topic")
    launch_keyboard = LaunchConfiguration("launch_keyboard")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")

    teleop = Node(
        package="so101_teleop",
        executable="teleop_split",
        name="arm_gripper_teleop",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("so101_teleop"), "config", "teleop_split.yaml"]
            ),
            {
                "arm_mode": "joint_trajectory",
                "leader_topic": "/leader/joint_states",
                "jtc_topic": jtc_topic,
                "gripper_mode": "forward_position",
                "gripper_fwd_topic": "/follower/gripper_controller/commands",
                "use_sim_time": True,
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
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_keyboard),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "jtc_topic",
                default_value="/follower/arm_trajectory_controller/joint_trajectory",
            ),
            DeclareLaunchArgument("launch_keyboard", default_value="true"),
            DeclareLaunchArgument("keyboard_prefix", default_value="xterm -e"),
            teleop,
            keyboard,
        ]
    )
