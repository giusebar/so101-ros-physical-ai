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


def _spawn_controller(controller):
    return Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[controller, "--controller-manager", "/follower/controller_manager"],
    )


def _clock_bridge():
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    bringup_share = get_package_share_directory("so101_bringup")
    world_file = os.path.join(bringup_share, "worlds", "so101_empty.sdf")
    follower_controllers = os.path.join(
        bringup_share, "config", "ros2_control", "follower_sim_controllers.yaml"
    )
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101_arm.urdf.xacro"]
    )
    robot_description = Command(
        [
            "xacro ",
            xacro_file,
            " variant:=follower",
            " use_ros2_control:=true",
            " hardware_type:=gazebo",
            " controller_config_file:=",
            follower_controllers,
            " ros_namespace:=/follower",
        ]
    )
    robot_description_param = ParameterValue(robot_description, value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", world_file]}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="follower",
        parameters=[
            {
                "robot_description": robot_description_param,
                "frame_prefix": "follower/",
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "so101_empty",
            "-string",
            robot_description,
            "-name",
            "follower",
            "-z",
            "0.02",
        ],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_moveit_config"), "launch", "move_group.launch.py"])
        ),
        launch_arguments={"namespace": "follower", "variant": "follower", "use_sim_time": "true"}.items(),
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_moveit_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
        launch_arguments={"namespace": "follower", "variant": "follower"}.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            gz_sim,
            _clock_bridge(),
            rsp,
            TimerAction(period=2.0, actions=[spawn]),
            TimerAction(
                period=4.0,
                actions=[
                    _spawn_controller("joint_state_broadcaster"),
                    _spawn_controller("arm_trajectory_controller"),
                    _spawn_controller("gripper_controller"),
                ],
            ),
            TimerAction(period=6.0, actions=[move_group, moveit_rviz]),
        ]
    )