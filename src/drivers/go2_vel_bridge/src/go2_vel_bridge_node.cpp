#include "go2_vel_bridge/sport_request.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

namespace go2_vel_bridge
{

enum class StopMode
{
  ZeroVelocity,
  StopMove,
  None,
};

enum class RotateGatePhase
{
  Idle,
  Rotating,
  Waiting,
};

class Go2VelBridgeNode : public rclcpp::Node
{
public:
  Go2VelBridgeNode()
  : Node("go2_vel_bridge")
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cmd_out_topic_ = declare_parameter<std::string>("cmd_out_topic", "/go2_vel_bridge/cmd_out");
    sport_request_topic_ =
      declare_parameter<std::string>("sport_request_topic", "/api/sport/request");
    max_vx_ = declare_parameter<double>("max_vx", 0.6);
    max_vy_ = declare_parameter<double>("max_vy", 0.0);
    max_vyaw_ = declare_parameter<double>("max_vyaw", 1.0);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 1.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    max_linear_accel_ = declare_parameter<double>("max_linear_accel", 1.5);
    max_angular_accel_ = declare_parameter<double>("max_angular_accel", 3.0);
    forward_linear_threshold_ =
      declare_parameter<double>("forward_linear_threshold", 0.15);
    forward_angular_deadband_ =
      declare_parameter<double>("forward_angular_deadband", 0.12);
    wait_for_turn_settle_ = declare_parameter<bool>("wait_for_turn_settle", true);
    turning_vyaw_threshold_ =
      declare_parameter<double>("turning_vyaw_threshold", 0.12);
    rotating_linear_threshold_ =
      declare_parameter<double>("rotating_linear_threshold", 0.08);
    settle_hold_time_ = declare_parameter<double>("settle_hold_time", 0.25);
    invert_angular_z_ = declare_parameter<bool>("invert_angular_z", false);
    stop_mode_ = parse_stop_mode(declare_parameter<std::string>("stop_mode", "zero_velocity"));

    if (publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be positive");
    }
    if (max_linear_accel_ <= 0.0 || max_angular_accel_ <= 0.0) {
      throw std::invalid_argument("max_linear_accel and max_angular_accel must be positive");
    }

    dt_ = 1.0 / publish_rate_;

    sport_pub_ = create_publisher<unitree_api::msg::Request>(sport_request_topic_, 10);
    cmd_out_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic_, 10);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&Go2VelBridgeNode::on_cmd_vel, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(dt_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Go2VelBridgeNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "go2_vel_bridge v2: %s -> %s + debug %s (rate %.1f Hz, stop_mode %s, "
      "turn_settle %s hold %.2fs)",
      cmd_vel_topic_.c_str(), sport_request_topic_.c_str(), cmd_out_topic_.c_str(),
      publish_rate_, stop_mode_name(stop_mode_).c_str(),
      wait_for_turn_settle_ ? "on" : "off", settle_hold_time_);
  }

private:
  static StopMode parse_stop_mode(const std::string & mode)
  {
    if (mode == "zero_velocity") {
      return StopMode::ZeroVelocity;
    }
    if (mode == "stop_move") {
      return StopMode::StopMove;
    }
    if (mode == "none") {
      return StopMode::None;
    }
    throw std::invalid_argument(
            "stop_mode must be one of: zero_velocity, stop_move, none");
  }

  static std::string stop_mode_name(StopMode mode)
  {
    switch (mode) {
      case StopMode::ZeroVelocity: return "zero_velocity";
      case StopMode::StopMove: return "stop_move";
      case StopMode::None: return "none";
    }
    return "unknown";
  }

  static double clamp_symmetric(double value, double limit)
  {
    if (limit <= 0.0) {
      return 0.0;
    }
    return std::clamp(value, -limit, limit);
  }

  static double approach(double current, double target, double max_delta)
  {
    const double delta = target - current;
    if (delta > max_delta) {
      return current + max_delta;
    }
    if (delta < -max_delta) {
      return current - max_delta;
    }
    return target;
  }

  double apply_vyaw_filter(double vyaw, double vx)
  {
    const double abs_vyaw = std::abs(vyaw);
    const double abs_vx = std::abs(vx);

    // Forward: zero small vyaw corrections to prevent left-right weaving on straight paths.
    if (abs_vx >= forward_linear_threshold_ && abs_vyaw < forward_angular_deadband_) {
      return 0.0;
    }

    return vyaw;
  }

  void reset_rotate_settle_state()
  {
    rotate_gate_phase_ = RotateGatePhase::Idle;
    post_rotate_wait_elapsed_ = 0.0;
  }

  bool is_rotate_in_place(double vx, double target_vyaw) const
  {
    return std::abs(vx) < rotating_linear_threshold_ &&
           std::abs(target_vyaw) >= turning_vyaw_threshold_;
  }

  // Idle -> Rotating (vx=0, vyaw passes) -> Waiting (full stop) -> Idle.
  void gate_velocity_after_rotate(double & vx, double & vyaw)
  {
    if (!wait_for_turn_settle_) {
      return;
    }

    const bool rotate_cmd = is_rotate_in_place(vx, vyaw);

    if (rotate_gate_phase_ == RotateGatePhase::Idle) {
      if (rotate_cmd) {
        rotate_gate_phase_ = RotateGatePhase::Rotating;
      }
      if (rotate_gate_phase_ == RotateGatePhase::Rotating) {
        vx = 0.0;
      }
      return;
    }

    if (rotate_gate_phase_ == RotateGatePhase::Rotating) {
      vx = 0.0;
      if (!rotate_cmd) {
        rotate_gate_phase_ = RotateGatePhase::Waiting;
        post_rotate_wait_elapsed_ = 0.0;
        vx = 0.0;
        vyaw = 0.0;
      }
      return;
    }

    // Waiting: hold full stop so RPP cannot keep commanding spin during settle.
    vx = 0.0;
    vyaw = 0.0;
    post_rotate_wait_elapsed_ += dt_;

    if (rotate_cmd) {
      rotate_gate_phase_ = RotateGatePhase::Rotating;
      post_rotate_wait_elapsed_ = 0.0;
      return;
    }

    if (post_rotate_wait_elapsed_ >= settle_hold_time_) {
      rotate_gate_phase_ = RotateGatePhase::Idle;
    }
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;
    has_cmd_ = true;
    last_cmd_time_ = now();
    stopped_sent_ = false;
    timeout_logged_ = false;
  }

  void compute_target(double & vx, double & vy, double & vyaw, bool cmd_fresh)
  {
    if (!cmd_fresh) {
      reset_rotate_settle_state();
      vx = 0.0;
      vy = 0.0;
      vyaw = 0.0;
      return;
    }

    vyaw = invert_angular_z_ ? -latest_cmd_.angular.z : latest_cmd_.angular.z;
    vx = clamp_symmetric(latest_cmd_.linear.x, max_vx_);
    vy = clamp_symmetric(latest_cmd_.linear.y, max_vy_);
    vyaw = clamp_symmetric(vyaw, max_vyaw_);
    gate_velocity_after_rotate(vx, vyaw);
    vyaw = apply_vyaw_filter(vyaw, vx);
  }

  void publish_cmd_out()
  {
    geometry_msgs::msg::Twist out;
    out.linear.x = out_vx_;
    out.linear.y = out_vy_;
    out.angular.z = out_vyaw_;
    cmd_out_pub_->publish(out);
  }

  void on_timer()
  {
    const bool cmd_fresh =
      has_cmd_ && (now() - last_cmd_time_).seconds() <= cmd_vel_timeout_;

    double target_vx = 0.0;
    double target_vy = 0.0;
    double target_vyaw = 0.0;
    compute_target(target_vx, target_vy, target_vyaw, cmd_fresh);

    if (stop_mode_ == StopMode::ZeroVelocity || cmd_fresh) {
      out_vx_ = approach(out_vx_, target_vx, max_linear_accel_ * dt_);
      out_vy_ = approach(out_vy_, target_vy, max_linear_accel_ * dt_);
      out_vyaw_ = approach(out_vyaw_, target_vyaw, max_angular_accel_ * dt_);

      unitree_api::msg::Request req;
      make_move_request(
        req,
        static_cast<float>(out_vx_),
        static_cast<float>(out_vy_),
        static_cast<float>(out_vyaw_));
      sport_pub_->publish(req);
      publish_cmd_out();

      if (!cmd_fresh && !timeout_logged_) {
        RCLCPP_WARN(
          get_logger(),
          "cmd_vel timeout (%.2f s), ramping to zero velocity", cmd_vel_timeout_);
        timeout_logged_ = true;
      }
      return;
    }

    if (stop_mode_ == StopMode::None || stopped_sent_) {
      return;
    }

    unitree_api::msg::Request req;
    make_stop_move_request(req);
    sport_pub_->publish(req);
    stopped_sent_ = true;
    RCLCPP_WARN(
      get_logger(),
      "cmd_vel timeout (%.2f s), sending StopMove", cmd_vel_timeout_);
  }

  std::string cmd_vel_topic_;
  std::string cmd_out_topic_;
  std::string sport_request_topic_;
  double max_vx_;
  double max_vy_;
  double max_vyaw_;
  double cmd_vel_timeout_;
  double publish_rate_;
  double max_linear_accel_;
  double max_angular_accel_;
  double forward_linear_threshold_;
  double forward_angular_deadband_;
  bool wait_for_turn_settle_;
  double turning_vyaw_threshold_;
  double rotating_linear_threshold_;
  double settle_hold_time_;
  RotateGatePhase rotate_gate_phase_{RotateGatePhase::Idle};
  double post_rotate_wait_elapsed_{0.0};
  double dt_;
  StopMode stop_mode_;
  bool invert_angular_z_;

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_out_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist latest_cmd_;
  bool has_cmd_{false};
  bool stopped_sent_{true};
  bool timeout_logged_{false};
  rclcpp::Time last_cmd_time_;

  double out_vx_{0.0};
  double out_vy_{0.0};
  double out_vyaw_{0.0};
};

}  // namespace go2_vel_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_vel_bridge::Go2VelBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
