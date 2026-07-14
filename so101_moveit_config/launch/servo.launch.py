"""Launch MoveIt Servo for the SO-101 follower arm (Cartesian jogging).

Phase 1 of the "Cartesian Teleop -> MoveIt Servo" roadmap. Brings up a Servo
node in the ``/follower`` namespace that:

  * subscribes to Twist / JointJog commands on
      ``/follower/servo_node/delta_twist_cmds``
      ``/follower/servo_node/delta_joint_cmds``
  * streams position setpoints (std_msgs/Float64MultiArray) to
      ``/follower/arm_forward_controller/commands``

This launch assumes the follower is already up (e.g. via
``so101_bringup follower_split.launch.py`` with
``arm_controller:=arm_forward_controller``), which provides robot_state_publisher,
the controller_manager, joint_states and the arm_forward_controller. Servo only
needs the robot model + kinematics + joint limits, passed here as parameters.

Validates on ``hardware_type:=mock`` — no physical arm required.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    variant = LaunchConfiguration("variant")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # Run Servo as a standalone node (easier to debug) or as a composable
    # node component (lower-latency intra-process comms with the scene monitor).
    launch_as_standalone_node = LaunchConfiguration("launch_as_standalone_node")

    xacro_path = os.path.join(
        get_package_share_directory("so101_description"),
        "urdf",
        "so101_arm.urdf.xacro",
    )

    # Reuse the same MoveIt assets as move_group: robot_description, SRDF
    # (manipulator group), kinematics.yaml and joint_limits.yaml.
    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "variant": variant,
                "use_ros2_control": "false",
            },
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .to_moveit_configs()
    )

    # Servo parameters (top-level keys in so101_servo.yaml) wrapped under the
    # "moveit_servo" namespace, matching the upstream moveit_servo launch API.
    servo_params = {
        "moveit_servo": ParameterBuilder("so101_moveit_config")
        .yaml("config/so101_servo.yaml")
        .to_dict()
    }

    common_parameters = [
        servo_params,
        {"use_sim_time": use_sim_time},
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    # Standalone Servo node.
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        namespace=namespace,
        parameters=common_parameters,
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    # Composable Servo node inside a multi-threaded container.
    servo_container = ComposableNodeContainer(
        name="servo_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                namespace=namespace,
                parameters=common_parameters,
            ),
        ],
        output="screen",
        condition=UnlessCondition(launch_as_standalone_node),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="follower"),
            DeclareLaunchArgument("variant", default_value="follower"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "launch_as_standalone_node",
                default_value="true",
                description="true: standalone servo_node; false: composable component",
            ),
            servo_node,
            servo_container,
        ]
    )
