import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _robot_description_command(variant, namespace, controller_config_file):
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101_arm.urdf.xacro"]
    )
    return Command(
        [
            "xacro ",
            xacro_file,
            " variant:=",
            variant,
            " use_ros2_control:=true",
            " hardware_type:=gazebo",
            " controller_config_file:=",
            controller_config_file,
            " ros_namespace:=/",
            namespace,
        ]
    )


def _rsp(namespace, robot_description):
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[
            {
                "robot_description": robot_description,
                "frame_prefix": namespace + "/",
                "use_sim_time": True,
            }
        ],
        output="screen",
    )


def _spawn_entity(name, robot_description, x, y):
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "so101_empty",
            "-string",
            robot_description,
            "-name",
            name,
            "-x",
            x,
            "-y",
            y,
            "-z",
            "0.02",
        ],
    )


def _spawn_controller(namespace, controller):
    return Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[controller, "--controller-manager", ["/", namespace, "/controller_manager"]],
    )


def _clock_bridge():
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )


def generate_launch_description():
    launch_keyboard = LaunchConfiguration("launch_keyboard")
    launch_teleop = LaunchConfiguration("launch_teleop")
    follower_arm_controller = LaunchConfiguration("follower_arm_controller")

    bringup_share = get_package_share_directory("so101_bringup")
    world_file = os.path.join(bringup_share, "worlds", "so101_empty.sdf")
    leader_controllers = os.path.join(
        bringup_share, "config", "ros2_control", "leader_sim_controllers.yaml"
    )
    follower_controllers = os.path.join(
        bringup_share, "config", "ros2_control", "follower_sim_controllers.yaml"
    )

    leader_description = _robot_description_command("leader", "leader", leader_controllers)
    follower_description = _robot_description_command("follower", "follower", follower_controllers)
    leader_description_param = ParameterValue(leader_description, value_type=str)
    follower_description_param = ParameterValue(follower_description, value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", world_file]}.items(),
    )

    leader_keyboard = Node(
        package="so101_teleop",
        executable="leader_sim_keyboard",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("so101_teleop"), "config", "leader_sim_keyboard.yaml"]
            ),
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_keyboard),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_teleop"), "launch", "teleop_split.launch.py"])
        ),
        launch_arguments={
            "leader_namespace": "leader",
            "follower_namespace": "follower",
            "arm_controller": "arm_trajectory_controller",
            "gripper_mode": "forward_position",
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(launch_teleop),
    )

    controller_start = TimerAction(
        period=4.0,
        actions=[
            _spawn_controller("leader", "joint_state_broadcaster"),
            _spawn_controller("leader", "arm_forward_controller"),
            _spawn_controller("leader", "gripper_forward_controller"),
            _spawn_controller("follower", "joint_state_broadcaster"),
            _spawn_controller("follower", follower_arm_controller),
            _spawn_controller("follower", "gripper_controller"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_keyboard", default_value="false"),
            DeclareLaunchArgument("launch_teleop", default_value="true"),
            DeclareLaunchArgument(
                "follower_arm_controller",
                default_value="arm_trajectory_controller",
                description=(
                    "Follower arm controller to spawn: arm_trajectory_controller "
                    "(JTC, for the built-in leader->follower teleop bridge) or "
                    "arm_forward_controller (position streaming, required when "
                    "driving the follower through MoveIt Servo). Set launch_teleop "
                    "to false when using Servo so the bridge does not fight it."
                ),
            ),
            gz_sim,
            _clock_bridge(),
            _rsp("leader", leader_description_param),
            _rsp("follower", follower_description_param),
            TimerAction(period=2.0, actions=[_spawn_entity("leader", leader_description, "0", "0.35")]),
            TimerAction(period=2.0, actions=[_spawn_entity("follower", follower_description, "0", "-0.35")]),
            controller_start,
            TimerAction(period=6.0, actions=[leader_keyboard, teleop]),
        ]
    )