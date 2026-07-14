// Leader -> MoveIt Servo JointJog adapter (Phase 2, "supervised mirror").
//
// Replaces the direct joint-copy done by teleop_split on the arm. Instead of
// forwarding leader joint positions straight to the follower controller, this
// node drives the follower *through* MoveIt Servo so the mirror inherits
// Servo's joint-limit, singularity and (optionally) collision safety.
//
// Data flow:
//   /leader/joint_states  (sensor_msgs/JointState)  --.
//                                                      |-> per-joint position
//   /follower/joint_states (sensor_msgs/JointState) --'   error (P controller)
//                                                      |
//                                                      v
//   /follower/servo_node/delta_joint_cmds (control_msgs/JointJog, unitless)
//                                                      |
//                                     MoveIt Servo -> arm_forward_controller
//
// The follower chases the leader: for each arm joint the commanded (unitless)
// velocity is clamp(kp * (leader - follower), -max, max). Because it is closed
// loop on the follower's measured state, the command naturally decays to zero
// as the follower catches up, so a settled arm holds still.
//
// This works identically on real hardware (real leader publishing
// /leader/joint_states) and in the Gazebo leader+follower sim (leader driven by
// leader_sim_keyboard). Only one writer may command the follower's position
// interface at a time, so run this INSTEAD of teleop_split.
//
// IMPORTANT: moveit_msgs/srv/ServoCommandType enum is JOINT_JOG=0, TWIST=1,
// POSE=2. Servo must be switched to JOINT_JOG before it will act on JointJog
// messages; this node calls that service (with retry) on startup.
//
// The gripper is not part of the Servo move group, so it is optionally
// forwarded directly to the follower gripper controller (leader-position copy),
// preserving the previous teleop_split behaviour for that joint.

#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

class LeaderServoJog : public rclcpp::Node
{
public:
  using ServoCommandType = moveit_msgs::srv::ServoCommandType;

  LeaderServoJog() : Node("leader_servo_jog")
  {
    // --- Topics / services ---
    leader_topic_ = declare_parameter<std::string>("leader_topic", "/leader/joint_states");
    follower_topic_ = declare_parameter<std::string>("follower_topic", "/follower/joint_states");
    jog_topic_ =
        declare_parameter<std::string>("jog_topic", "/follower/servo_node/delta_joint_cmds");
    switch_service_ = declare_parameter<std::string>(
        "switch_service", "/follower/servo_node/switch_command_type");

    // --- Joints ---
    arm_joints_ = declare_parameter<std::vector<std::string>>(
        "arm_joints", std::vector<std::string>{"shoulder_pan", "shoulder_lift", "elbow_flex",
                                               "wrist_flex", "wrist_roll"});

    // --- Control law ---
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    kp_ = declare_parameter<double>("kp", 10.0);
    deadband_rad_ = declare_parameter<double>("deadband_rad", 0.01);
    max_command_ = declare_parameter<double>("max_command", 1.0);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.5);

    // --- Gripper passthrough (Servo does not own the gripper) ---
    forward_gripper_ = declare_parameter<bool>("forward_gripper", true);
    gripper_joint_ = declare_parameter<std::string>("gripper_joint", "gripper");
    gripper_topic_ =
        declare_parameter<std::string>("gripper_topic", "/follower/gripper_controller/commands");
    gripper_deadband_ = declare_parameter<double>("gripper_deadband", 0.005);
    gripper_min_interval_s_ = declare_parameter<double>("gripper_min_interval_s", 0.05);

    arm_cmd_.resize(arm_joints_.size(), 0.0);

    // --- ROS interfaces ---
    leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        leader_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LeaderServoJog::leader_cb, this, std::placeholders::_1));
    follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        follower_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LeaderServoJog::follower_cb, this, std::placeholders::_1));

    jog_pub_ = create_publisher<control_msgs::msg::JointJog>(jog_topic_, rclcpp::QoS(10));
    if (forward_gripper_) {
      gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(gripper_topic_,
                                                                        rclcpp::QoS(10).reliable());
    }

    switch_client_ = create_client<ServoCommandType>(switch_service_);

    control_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz_),
                                       std::bind(&LeaderServoJog::control_loop, this));
    // Try to put Servo into JOINT_JOG mode until it succeeds.
    switch_timer_ = create_wall_timer(std::chrono::seconds(1),
                                      std::bind(&LeaderServoJog::try_switch_command_type, this));

    RCLCPP_INFO(get_logger(), "leader_servo_jog: %s + %s -> %s (kp=%.2f, deadband=%.3f rad)",
                leader_topic_.c_str(), follower_topic_.c_str(), jog_topic_.c_str(), kp_,
                deadband_rad_);
  }

private:
  // --- Parameters ---
  std::string leader_topic_, follower_topic_, jog_topic_, switch_service_;
  std::vector<std::string> arm_joints_;
  double publish_rate_hz_{100.0};
  double kp_{4.0};
  double deadband_rad_{0.01};
  double max_command_{1.0};
  double stale_timeout_s_{0.5};
  bool forward_gripper_{true};
  std::string gripper_joint_;
  std::string gripper_topic_;
  double gripper_deadband_{0.005};
  double gripper_min_interval_s_{0.05};

  // --- ROS interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_sub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr jog_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Client<ServoCommandType>::SharedPtr switch_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr switch_timer_;

  // --- State ---
  std::vector<double> leader_arm_;
  std::vector<double> follower_arm_;
  std::optional<double> leader_gripper_;
  std::vector<double> arm_cmd_;
  bool have_leader_{false};
  bool have_follower_{false};
  bool switch_pending_{false};
  bool switched_{false};
  rclcpp::Time last_leader_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_follower_stamp_{0, 0, RCL_ROS_TIME};
  double last_gripper_goal_{0.0};
  rclcpp::Time last_gripper_goal_time_{0, 0, RCL_ROS_TIME};

  // Extract the configured arm joints (and gripper) from a JointState message.
  // Returns false if any arm joint is missing.
  bool extract(const sensor_msgs::msg::JointState &msg, std::vector<double> &arm_out,
               std::optional<double> *gripper_out) const
  {
    std::unordered_map<std::string, size_t> idx;
    idx.reserve(msg.name.size());
    for (size_t i = 0; i < msg.name.size(); ++i) idx[msg.name[i]] = i;

    arm_out.assign(arm_joints_.size(), 0.0);
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      auto it = idx.find(arm_joints_[i]);
      if (it == idx.end() || it->second >= msg.position.size()) return false;
      arm_out[i] = msg.position[it->second];
    }
    if (gripper_out) {
      auto it = idx.find(gripper_joint_);
      if (it != idx.end() && it->second < msg.position.size())
        *gripper_out = msg.position[it->second];
    }
    return true;
  }

  void leader_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::optional<double> grip;
    if (!extract(*msg, leader_arm_, &grip)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Leader joint_states missing configured arm joints");
      return;
    }
    leader_gripper_ = grip;
    have_leader_ = true;
    last_leader_stamp_ = now();
  }

  void follower_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!extract(*msg, follower_arm_, nullptr)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Follower joint_states missing configured arm joints");
      return;
    }
    have_follower_ = true;
    last_follower_stamp_ = now();
  }

  void try_switch_command_type()
  {
    if (switched_ || switch_pending_) return;
    if (!switch_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Waiting for Servo switch_command_type service (%s)...",
                           switch_service_.c_str());
      return;
    }
    auto req = std::make_shared<ServoCommandType::Request>();
    req->command_type = ServoCommandType::Request::JOINT_JOG;  // = 0
    switch_pending_ = true;
    switch_client_->async_send_request(
        req, [this](rclcpp::Client<ServoCommandType>::SharedFuture future) {
          switch_pending_ = false;
          if (future.get()->success) {
            switched_ = true;
            RCLCPP_INFO(get_logger(), "Servo switched to JOINT_JOG mode.");
          } else {
            RCLCPP_WARN(get_logger(), "Servo rejected JOINT_JOG switch; will retry.");
          }
        });
  }

  void control_loop()
  {
    forward_gripper_command();

    if (!switched_ || !have_leader_ || !have_follower_) return;

    const auto t = now();
    if ((t - last_leader_stamp_).seconds() > stale_timeout_s_) return;
    if ((t - last_follower_stamp_).seconds() > stale_timeout_s_) return;
    if (leader_arm_.size() != arm_joints_.size() ||
        follower_arm_.size() != arm_joints_.size())
      return;

    // Proportional position-error controller -> unitless [-1, 1] velocities.
    bool any_motion = false;
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      const double error = leader_arm_[i] - follower_arm_[i];
      double v = 0.0;
      if (std::abs(error) > deadband_rad_) {
        v = std::clamp(kp_ * error, -max_command_, max_command_);
        any_motion = true;
      }
      arm_cmd_[i] = v;
    }
    (void)any_motion;  // We publish every tick (incl. zeros) to keep Servo fresh.

    control_msgs::msg::JointJog jog;
    jog.header.stamp = t;              // MUST be now(): Servo drops stale commands.
    jog.joint_names = arm_joints_;
    jog.velocities = arm_cmd_;         // unitless, scaled by Servo scale.joint
    jog_pub_->publish(jog);
  }

  void forward_gripper_command()
  {
    if (!forward_gripper_ || !gripper_pub_ || !leader_gripper_) return;
    const auto t = now();
    if ((t - last_leader_stamp_).seconds() > stale_timeout_s_) return;
    if (std::abs(*leader_gripper_ - last_gripper_goal_) <= gripper_deadband_) return;
    if ((t - last_gripper_goal_time_).seconds() < gripper_min_interval_s_) return;

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {*leader_gripper_};
    gripper_pub_->publish(cmd);
    last_gripper_goal_ = *leader_gripper_;
    last_gripper_goal_time_ = t;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderServoJog>());
  rclcpp::shutdown();
  return 0;
}
