#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace
{
class TerminalRawMode
{
public:
  TerminalRawMode() = default;
  ~TerminalRawMode() { restore(); }

  TerminalRawMode(const TerminalRawMode &) = delete;
  TerminalRawMode &operator=(const TerminalRawMode &) = delete;

  bool enable()
  {
    if (enabled_) return true;
    if (!::isatty(STDIN_FILENO)) return false;
    if (::tcgetattr(STDIN_FILENO, &original_) < 0) return false;

    termios raw = original_;
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | IEXTEN);
    raw.c_iflag &= ~static_cast<tcflag_t>(IXON | ICRNL);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0) return false;
    enabled_ = true;
    return true;
  }

  void restore()
  {
    if (!enabled_) return;
    (void)::tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    enabled_ = false;
  }

  int read_byte() const
  {
    unsigned char ch = 0;
    const ssize_t n = ::read(STDIN_FILENO, &ch, 1);
    return (n == 1) ? static_cast<int>(ch) : -1;
  }

private:
  termios original_{};
  bool enabled_{false};
};
}  // namespace

class LeaderSimKeyboard : public rclcpp::Node
{
public:
  explicit LeaderSimKeyboard(TerminalRawMode &terminal)
      : Node("leader_sim_keyboard"), terminal_(terminal)
  {
    arm_topic_ = declare_parameter<std::string>("arm_topic", "/leader/arm_forward_controller/commands");
    gripper_topic_ = declare_parameter<std::string>("gripper_topic", "/leader/gripper_forward_controller/commands");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/leader/joint_states");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    step_rad_ = declare_parameter<double>("step_rad", 0.035);
    fast_step_multiplier_ = declare_parameter<double>("fast_step_multiplier", 3.0);
    stale_seed_timeout_s_ = declare_parameter<double>("stale_seed_timeout_s", 2.0);
    arm_joints_ = declare_parameter<std::vector<std::string>>(
      "arm_joints", std::vector<std::string>{"shoulder_pan", "shoulder_lift", "elbow_flex",
                          "wrist_flex", "wrist_roll"});
    gripper_joint_ = declare_parameter<std::string>("gripper_joint", "gripper");
    lower_limits_ = declare_parameter<std::vector<double>>(
      "lower_limits", std::vector<double>{-1.91986, -1.74533, -1.69, -1.65806, -2.74385});
    upper_limits_ = declare_parameter<std::vector<double>>(
      "upper_limits", std::vector<double>{1.91986, 1.74533, 1.69, 1.65806, 2.84121});
    gripper_lower_limit_ = declare_parameter<double>("gripper_lower_limit", -0.174533);
    gripper_upper_limit_ = declare_parameter<double>("gripper_upper_limit", 1.74533);

    if (lower_limits_.size() != arm_joints_.size() || upper_limits_.size() != arm_joints_.size()) {
      throw std::runtime_error("leader_sim_keyboard limits must match arm_joints length");
    }

    arm_targets_.assign(arm_joints_.size(), 0.0);
    gripper_target_ = 0.0;

    arm_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(arm_topic_, rclcpp::QoS(10).reliable());
    gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(gripper_topic_, rclcpp::QoS(10).reliable());
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LeaderSimKeyboard::joint_state_callback, this, std::placeholders::_1));

    key_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                   std::bind(&LeaderSimKeyboard::poll_keyboard, this));
    publish_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz_),
                                       std::bind(&LeaderSimKeyboard::publish_targets, this));

    print_help();
  }

private:
  TerminalRawMode &terminal_;
  std::string arm_topic_;
  std::string gripper_topic_;
  std::string joint_state_topic_;
  std::vector<std::string> arm_joints_;
  std::string gripper_joint_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> arm_targets_;
  double gripper_target_{0.0};
  double publish_rate_hz_{50.0};
  double step_rad_{0.035};
  double fast_step_multiplier_{3.0};
  double stale_seed_timeout_s_{2.0};
  double gripper_lower_limit_{-0.174533};
  double gripper_upper_limit_{1.74533};
  bool seeded_from_joint_state_{false};
  rclcpp::Time last_seed_time_{0, 0, RCL_ROS_TIME};
  int esc_state_{0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr key_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  void print_help() const
  {
    std::printf("\nSO-101 simulated leader keyboard\n");
    std::printf("  1/q shoulder_pan   +/-\n");
    std::printf("  2/w shoulder_lift  +/-\n");
    std::printf("  3/e elbow_flex     +/-\n");
    std::printf("  4/r wrist_flex     +/-\n");
    std::printf("  5/t wrist_roll     +/-\n");
    std::printf("  o/p gripper        close/open\n");
    std::printf("  Shift = fast step, space = zero arm, h = help, x = quit\n\n");
    std::fflush(stdout);
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (seeded_from_joint_state_ && (now() - last_seed_time_).seconds() < stale_seed_timeout_s_) return;

    std::unordered_map<std::string, size_t> index_by_name;
    index_by_name.reserve(msg->name.size());
    for (size_t index = 0; index < msg->name.size(); ++index) index_by_name[msg->name[index]] = index;

    bool complete = true;
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      const auto it = index_by_name.find(arm_joints_[i]);
      if (it == index_by_name.end() || it->second >= msg->position.size()) {
        complete = false;
        break;
      }
      arm_targets_[i] = clamp_arm(i, msg->position[it->second]);
    }

    if (const auto it = index_by_name.find(gripper_joint_);
        it != index_by_name.end() && it->second < msg->position.size()) {
      gripper_target_ = clamp_gripper(msg->position[it->second]);
    }

    if (complete) {
      seeded_from_joint_state_ = true;
      last_seed_time_ = now();
    }
  }

  void poll_keyboard()
  {
    int ch = terminal_.read_byte();
    if (ch < 0) return;

    if (esc_state_ == 0 && ch == 27) {
      esc_state_ = 1;
      return;
    }
    if (esc_state_ == 1) {
      esc_state_ = (ch == '[') ? 2 : 0;
      return;
    }
    if (esc_state_ == 2) {
      esc_state_ = 0;
      return;
    }

    const bool fast = ch >= 'A' && ch <= 'Z';
    const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    const double step = step_rad_ * (fast ? fast_step_multiplier_ : 1.0);

    switch (key) {
      case '1': increment_joint(0, step); break;
      case 'q': increment_joint(0, -step); break;
      case '2': increment_joint(1, step); break;
      case 'w': increment_joint(1, -step); break;
      case '3': increment_joint(2, step); break;
      case 'e': increment_joint(2, -step); break;
      case '4': increment_joint(3, step); break;
      case 'r': increment_joint(3, -step); break;
      case '5': increment_joint(4, step); break;
      case 't': increment_joint(4, -step); break;
      case 'o': gripper_target_ = clamp_gripper(gripper_target_ - step); break;
      case 'p': gripper_target_ = clamp_gripper(gripper_target_ + step); break;
      case ' ': zero_arm(); break;
      case 'h': print_help(); break;
      case 'x': rclcpp::shutdown(); break;
      default: break;
    }
  }

  void increment_joint(size_t index, double delta)
  {
    if (index >= arm_targets_.size()) return;
    arm_targets_[index] = clamp_arm(index, arm_targets_[index] + delta);
  }

  void zero_arm()
  {
    std::fill(arm_targets_.begin(), arm_targets_.end(), 0.0);
  }

  double clamp_arm(size_t index, double value) const
  {
    return std::clamp(value, lower_limits_[index], upper_limits_[index]);
  }

  double clamp_gripper(double value) const
  {
    return std::clamp(value, gripper_lower_limit_, gripper_upper_limit_);
  }

  void publish_targets()
  {
    std_msgs::msg::Float64MultiArray arm_msg;
    arm_msg.data = arm_targets_;
    arm_pub_->publish(arm_msg);

    std_msgs::msg::Float64MultiArray gripper_msg;
    gripper_msg.data = {gripper_target_};
    gripper_pub_->publish(gripper_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  TerminalRawMode terminal;
  if (!terminal.enable()) {
    std::fprintf(stderr, "leader_sim_keyboard requires an interactive terminal.\n");
    return 1;
  }

  auto node = std::make_shared<LeaderSimKeyboard>(terminal);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}