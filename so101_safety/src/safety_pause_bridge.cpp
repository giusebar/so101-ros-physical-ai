// Protective-stop -> MoveIt Servo pause bridge.
//
// Bridges the perception safety signal to MoveIt Servo on the forward-controller
// teleop path (the Servo-native replacement for trajectory_safety_gate, which
// only works on the JointTrajectory/JTC path).
//
//   /safety/protective_stop (std_msgs/Bool)
//        true  -> call pause_servo(true)   Servo smooth-halts all output
//        false -> call pause_servo(false)  Servo resumes
//
// Uses the /follower/servo_node/pause_servo service (std_srvs/SetBool). The
// desired pause state is reconciled against the last successfully-applied state
// by a timer, so a transient (service not ready, failed call, or a stop that
// arrives before Servo is up) is retried until it takes effect. Edge-triggered:
// the service is only called when the desired state actually changes.
//
// NOTE: pause_servo only affects Servo. Motion that does not flow through Servo
// (e.g. planned MoveIt execution on the JTC) is NOT halted by this bridge and
// needs its own protective-stop handling.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <optional>
#include <string>

class SafetyPauseBridge : public rclcpp::Node
{
public:
  using SetBool = std_srvs::srv::SetBool;

  SafetyPauseBridge() : Node("safety_pause_bridge")
  {
    safety_stop_topic_ =
        declare_parameter<std::string>("safety_stop_topic", "/safety/protective_stop");
    pause_service_ =
        declare_parameter<std::string>("pause_service", "/follower/servo_node/pause_servo");
    // Assume clear on startup and actively resume Servo once the service is up,
    // so a stale paused state from a previous run does not linger.
    resume_on_start_ = declare_parameter<bool>("resume_on_start", true);
    reconcile_period_s_ = declare_parameter<double>("reconcile_period_s", 0.5);

    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        safety_stop_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&SafetyPauseBridge::stop_cb, this, std::placeholders::_1));

    pause_client_ = create_client<SetBool>(pause_service_);

    if (resume_on_start_) desired_pause_ = false;

    reconcile_timer_ = create_wall_timer(std::chrono::duration<double>(reconcile_period_s_),
                                         std::bind(&SafetyPauseBridge::reconcile, this));

    RCLCPP_INFO(get_logger(), "safety_pause_bridge: %s -> %s (pause on true)",
                safety_stop_topic_.c_str(), pause_service_.c_str());
  }

private:
  std::string safety_stop_topic_;
  std::string pause_service_;
  bool resume_on_start_{true};
  double reconcile_period_s_{0.5};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Client<SetBool>::SharedPtr pause_client_;
  rclcpp::TimerBase::SharedPtr reconcile_timer_;

  std::optional<bool> desired_pause_;  // what we want Servo to be
  std::optional<bool> applied_pause_;  // last state Servo confirmed
  bool call_in_flight_{false};

  void stop_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool want_pause = msg->data;
    if (desired_pause_ != want_pause) {
      RCLCPP_WARN(get_logger(), "%s", want_pause ? "PROTECTIVE STOP -> pausing Servo"
                                                 : "Clear -> resuming Servo");
    }
    desired_pause_ = want_pause;
    reconcile();  // react immediately, don't wait for the timer
  }

  void reconcile()
  {
    if (!desired_pause_.has_value()) return;           // nothing to do yet
    if (call_in_flight_) return;                       // wait for the pending call
    if (applied_pause_ == desired_pause_) return;      // already in desired state

    if (!pause_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Waiting for Servo pause service (%s)...", pause_service_.c_str());
      return;
    }

    const bool target = *desired_pause_;
    auto req = std::make_shared<SetBool::Request>();
    req->data = target;
    call_in_flight_ = true;
    pause_client_->async_send_request(
        req, [this, target](rclcpp::Client<SetBool>::SharedFuture future) {
          call_in_flight_ = false;
          const auto resp = future.get();
          if (resp->success) {
            applied_pause_ = target;
            RCLCPP_INFO(get_logger(), "Servo %s.", target ? "paused" : "resumed");
          } else {
            RCLCPP_WARN(get_logger(), "pause_servo(%s) rejected: %s (will retry)",
                        target ? "true" : "false", resp->message.c_str());
          }
        });
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyPauseBridge>());
  rclcpp::shutdown();
  return 0;
}
