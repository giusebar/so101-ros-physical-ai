from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Launch arguments ---
    hardware_type = LaunchConfiguration("hardware_type")  # real | mock

    leader_namespace = LaunchConfiguration("leader_namespace")
    follower_namespace = LaunchConfiguration("follower_namespace")
    leader_frame_prefix = LaunchConfiguration("leader_frame_prefix")
    follower_frame_prefix = LaunchConfiguration("follower_frame_prefix")

    leader_usb_port = LaunchConfiguration("leader_usb_port")
    follower_usb_port = LaunchConfiguration("follower_usb_port")

    leader_joint_config_file = LaunchConfiguration("leader_joint_config_file")
    follower_joint_config_file = LaunchConfiguration("follower_joint_config_file")

    leader_controller_config_file = LaunchConfiguration("leader_controller_config_file")
    follower_controller_config_file = LaunchConfiguration("follower_controller_config_file")

    arm_controller = LaunchConfiguration("arm_controller")

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # --- Include leader bringup (state-only, no command interfaces) ---
    leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "leader.launch.py"])
        ),
        launch_arguments={
            "namespace": leader_namespace,
            "hardware_type": hardware_type,
            "usb_port": leader_usb_port,
            "frame_prefix": leader_frame_prefix,
            "joint_config_file": leader_joint_config_file,
            "controller_config_file": leader_controller_config_file,
            "use_rviz": "false",
        }.items(),
    )

    # --- Include follower bringup (split controllers: forward or trajectory,
    # runtime-selectable via arm_controller) ---
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "follower_split.launch.py"])
        ),
        launch_arguments={
            "namespace": follower_namespace,
            "hardware_type": hardware_type,
            "usb_port": follower_usb_port,
            "frame_prefix": follower_frame_prefix,
            "joint_config_file": follower_joint_config_file,
            "controller_config_file": follower_controller_config_file,
            "arm_controller": arm_controller,
            "use_rviz": "false",
        }.items(),
    )

    # --- Static TF layout (positions the two arms side-by-side in 'world',
    # matching the Gazebo sim layout, so they don't overlap in RViz) ---
    layout_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "layout_tf.launch.py"])
        ),
    )

    # --- Optional combined RViz (both arms). Off by default -- this launch
    # file is meant to publish robot_description/TF/joint_states for arms so
    # an externally-run rviz2 can visualize them; use_rviz:=true is only a
    # convenience for local/dev use (e.g. outside of the so101-bringup snap,
    # which never enables it). ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="hardware_bringup_rviz",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    default_leader_ctrl_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "ros2_control", "leader_controllers.yaml"]
    )
    default_follower_ctrl_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "ros2_control", "follower_split_controllers.yaml"]
    )
    default_rviz_config = PathJoinSubstitution([FindPackageShare("so101_bringup"), "rviz", "teleop.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real", description="real | mock"),
            DeclareLaunchArgument("leader_namespace", default_value="leader"),
            DeclareLaunchArgument("follower_namespace", default_value="follower"),
            DeclareLaunchArgument("leader_frame_prefix", default_value="leader/"),
            DeclareLaunchArgument("follower_frame_prefix", default_value="follower/"),
            DeclareLaunchArgument("leader_usb_port", default_value="/dev/so101_leader"),
            DeclareLaunchArgument("follower_usb_port", default_value="/dev/so101_follower"),
            DeclareLaunchArgument("leader_joint_config_file", default_value=""),
            DeclareLaunchArgument("follower_joint_config_file", default_value=""),
            DeclareLaunchArgument("leader_controller_config_file", default_value=default_leader_ctrl_cfg),
            DeclareLaunchArgument("follower_controller_config_file", default_value=default_follower_ctrl_cfg),
            DeclareLaunchArgument(
                "arm_controller",
                default_value="arm_trajectory_controller",
                description="arm_trajectory_controller | arm_forward_controller",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            leader_launch,
            follower_launch,
            layout_tf_launch,
            rviz_node,
        ]
    )
