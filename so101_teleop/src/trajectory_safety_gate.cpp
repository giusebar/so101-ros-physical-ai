#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

class TrajectorySafetyGate : public rclcpp::Node
{
public:
  TrajectorySafetyGate() : Node("trajectory_safety_gate")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/safety/follower/arm_trajectory_in");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/follower/arm_trajectory_controller/joint_trajectory");
    const auto legacy_stop_topic = declare_parameter<std::string>(
      "obstacle_topic", "/safety/protective_stop");
    safety_stop_topic_ = declare_parameter<std::string>("safety_stop_topic", legacy_stop_topic);
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/follower/joint_states");
    hold_duration_s_ = declare_parameter<double>("hold_duration_s", 0.2);
    arm_joints_ = declare_parameter<std::vector<std::string>>(
      "arm_joints", std::vector<std::string>{"shoulder_pan", "shoulder_lift", "elbow_flex",
                                             "wrist_flex", "wrist_roll"});

    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      output_topic_, rclcpp::QoS(10).reliable());

    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      input_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&TrajectorySafetyGate::trajectory_callback, this, std::placeholders::_1));

    obstacle_sub_ = create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&TrajectorySafetyGate::obstacle_callback, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrajectorySafetyGate::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Safety gate input: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Safety gate output: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Protective stop topic: %s", safety_stop_topic_.c_str());
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  std::string safety_stop_topic_;
  std::string joint_states_topic_;
  double hold_duration_s_{0.2};
  std::vector<std::string> arm_joints_;

  bool blocked_{false};
  std::optional<sensor_msgs::msg::JointState> latest_joint_state_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (blocked_) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000, "Blocking follower trajectory while obstacle is present");
      return;
    }

    trajectory_pub_->publish(*msg);
  }

  void obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool was_blocked = blocked_;
    blocked_ = msg->data;

    if (blocked_ && !was_blocked) {
      RCLCPP_WARN(get_logger(), "Obstacle present: holding follower arm");
      publish_hold_trajectory();
    } else if (!blocked_ && was_blocked) {
      RCLCPP_INFO(get_logger(), "Obstacle cleared: resuming follower arm commands");
    }
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_joint_state_ = *msg;
  }

  void publish_hold_trajectory()
  {
    const auto positions = current_arm_positions();
    if (!positions) {
      RCLCPP_WARN(get_logger(), "Cannot publish hold trajectory yet: follower joint state unavailable");
      return;
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = arm_joints_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = *positions;
    const int sec = static_cast<int>(hold_duration_s_);
    const int nsec = static_cast<int>((hold_duration_s_ - sec) * 1e9);
    point.time_from_start.sec = sec;
    point.time_from_start.nanosec = nsec;
    trajectory.points.push_back(point);

    trajectory_pub_->publish(trajectory);
  }

  std::optional<std::vector<double>> current_arm_positions() const
  {
    if (!latest_joint_state_) return std::nullopt;

    const auto & joint_state = *latest_joint_state_;
    std::unordered_map<std::string, double> positions;
    positions.reserve(joint_state.name.size());

    const auto count = std::min(joint_state.name.size(), joint_state.position.size());
    for (size_t index = 0; index < count; ++index) {
      positions[joint_state.name[index]] = joint_state.position[index];
    }

    std::vector<double> arm_positions;
    arm_positions.reserve(arm_joints_.size());
    for (const auto & joint_name : arm_joints_) {
      const auto position = positions.find(joint_name);
      if (position == positions.end()) {
        RCLCPP_WARN(get_logger(), "Follower joint '%s' not found in latest joint state", joint_name.c_str());
        return std::nullopt;
      }
      arm_positions.push_back(position->second);
    }

    return arm_positions;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectorySafetyGate>());
  rclcpp::shutdown();
  return 0;
}