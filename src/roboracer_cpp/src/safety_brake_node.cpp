#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"


class SafetyBrakeNode : public rclcpp::Node
{
public:
  SafetyBrakeNode()
  : Node("safety_brake_node")
  {
    // Input freshness.
    this->declare_parameter<double>("command_timeout_sec", 0.30);
    this->declare_parameter<double>("distance_timeout_sec", 0.50);
    this->declare_parameter<double>("publish_period_sec", 0.05);

    // Distance-based AEB.
    this->declare_parameter<bool>("distance_safety_enabled", true);
    this->declare_parameter<bool>("require_distance_data", true);
    this->declare_parameter<double>("minimum_clearance_m", 0.35);
    this->declare_parameter<double>("reaction_time_sec", 0.25);
    this->declare_parameter<double>("braking_deceleration_mps2", 1.00);
    this->declare_parameter<double>("slowdown_margin_m", 0.50);
    this->declare_parameter<double>("estop_release_margin_m", 0.10);

    // Normalized brake output: 0.0 = none, 1.0 = maximum requested brake.
    this->declare_parameter<double>("slowdown_brake_max_request", 0.50);
    this->declare_parameter<double>("emergency_brake_request", 1.00);

    // Issue a short brake request when the command changes from forward motion to stop.
    this->declare_parameter<bool>("command_stop_brake_enabled", true);
    this->declare_parameter<double>("command_stop_brake_request", 0.60);
    this->declare_parameter<double>("command_stop_brake_hold_sec", 0.50);
    this->declare_parameter<double>("forward_speed_epsilon_mps", 0.03);

    command_timeout_sec_ =
      std::max(0.05, this->get_parameter("command_timeout_sec").as_double());
    distance_timeout_sec_ =
      std::max(0.05, this->get_parameter("distance_timeout_sec").as_double());
    publish_period_sec_ =
      std::max(0.01, this->get_parameter("publish_period_sec").as_double());

    distance_safety_enabled_ =
      this->get_parameter("distance_safety_enabled").as_bool();
    require_distance_data_ =
      this->get_parameter("require_distance_data").as_bool();

    minimum_clearance_m_ =
      std::max(0.0, this->get_parameter("minimum_clearance_m").as_double());
    reaction_time_sec_ =
      std::max(0.0, this->get_parameter("reaction_time_sec").as_double());
    braking_deceleration_mps2_ =
      std::max(0.05, this->get_parameter("braking_deceleration_mps2").as_double());
    slowdown_margin_m_ =
      std::max(0.01, this->get_parameter("slowdown_margin_m").as_double());
    estop_release_margin_m_ =
      std::max(0.0, this->get_parameter("estop_release_margin_m").as_double());

    slowdown_brake_max_request_ = clamp01(
      this->get_parameter("slowdown_brake_max_request").as_double());
    emergency_brake_request_ = clamp01(
      this->get_parameter("emergency_brake_request").as_double());

    command_stop_brake_enabled_ =
      this->get_parameter("command_stop_brake_enabled").as_bool();
    command_stop_brake_request_ = clamp01(
      this->get_parameter("command_stop_brake_request").as_double());
    command_stop_brake_hold_sec_ =
      std::max(0.0, this->get_parameter("command_stop_brake_hold_sec").as_double());
    forward_speed_epsilon_mps_ =
      std::max(0.0, this->get_parameter("forward_speed_epsilon_mps").as_double());

    requested_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_requested",
      10,
      std::bind(&SafetyBrakeNode::requested_cmd_callback, this, std::placeholders::_1));

    front_distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/front_distance",
      10,
      std::bind(&SafetyBrakeNode::front_distance_callback, this, std::placeholders::_1));

    safe_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    brake_request_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/brake/request", 10);

    safety_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/safety/state", 10);

    stop_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/safety/dynamic_stop_distance", 10);

    slowdown_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/safety/dynamic_slowdown_distance", 10);

    speed_scale_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/safety/speed_scale", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(publish_period_sec_),
      std::bind(&SafetyBrakeNode::control_loop, this));

    command_stop_brake_until_ = this->now();

    RCLCPP_INFO(this->get_logger(), "safety_brake_node started.");
    RCLCPP_INFO(
      this->get_logger(),
      "Input /cmd_vel_requested -> safe output /cmd_vel;"
      "final topic may be changed by launch remapping; "
      "brake output /brake/request.");
    RCLCPP_INFO(
      this->get_logger(),
      "Distance AEB enabled=%s, require_distance_data=%s.",
      distance_safety_enabled_ ? "true" : "false",
      require_distance_data_ ? "true" : "false");
  }

private:
  static double clamp01(double value)
  {
    return std::max(0.0, std::min(1.0, value));
  }

  void requested_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double new_forward_speed = std::max(0.0, msg->linear.x);

    // Detect a commanded forward-to-stop transition.
    if (
      command_stop_brake_enabled_ &&
      previous_requested_forward_speed_mps_ > forward_speed_epsilon_mps_ &&
      new_forward_speed <= forward_speed_epsilon_mps_)
    {
      command_stop_brake_until_ =
        this->now() + rclcpp::Duration::from_seconds(command_stop_brake_hold_sec_);
      command_stop_source_speed_mps_ = previous_requested_forward_speed_mps_;
    }

    // Reverse is treated as an escape command and cancels the timed front brake hold.
    if (msg->linear.x < -forward_speed_epsilon_mps_) {
      command_stop_brake_until_ = this->now();
    }

    previous_requested_forward_speed_mps_ = new_forward_speed;
    last_requested_cmd_ = *msg;
    last_requested_cmd_time_ = this->now();
    received_requested_cmd_ = true;
  }

  void front_distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (std::isfinite(msg->data) && msg->data >= 0.0F) {
      front_distance_m_ = msg->data;
      last_front_distance_time_ = this->now();
      received_front_distance_ = true;
    }
  }

  bool command_is_fresh() const
  {
    if (!received_requested_cmd_) {
      return false;
    }
    return (this->now() - last_requested_cmd_time_).seconds() <= command_timeout_sec_;
  }

  bool distance_is_fresh() const
  {
    if (!received_front_distance_) {
      return false;
    }
    return (this->now() - last_front_distance_time_).seconds() <= distance_timeout_sec_;
  }

  double calculate_stop_distance(double forward_speed_mps) const
  {
    const double speed = std::max(0.0, forward_speed_mps);
    return minimum_clearance_m_ +
      speed * reaction_time_sec_ +
      (speed * speed) / (2.0 * braking_deceleration_mps2_);
  }

  geometry_msgs::msg::Twist make_stop_cmd() const
  {
    return geometry_msgs::msg::Twist();
  }

  void publish_outputs(
    const geometry_msgs::msg::Twist & cmd,
    double brake_request,
    const std::string & state,
    double stop_distance,
    double slowdown_distance,
    double speed_scale)
  {
    safe_cmd_pub_->publish(cmd);

    std_msgs::msg::Float32 brake_msg;
    brake_msg.data = static_cast<float>(clamp01(brake_request));
    brake_request_pub_->publish(brake_msg);

    std_msgs::msg::String state_msg;
    state_msg.data = state;
    safety_state_pub_->publish(state_msg);

    std_msgs::msg::Float32 stop_msg;
    stop_msg.data = static_cast<float>(stop_distance);
    stop_distance_pub_->publish(stop_msg);

    std_msgs::msg::Float32 slowdown_msg;
    slowdown_msg.data = static_cast<float>(slowdown_distance);
    slowdown_distance_pub_->publish(slowdown_msg);

    std_msgs::msg::Float32 scale_msg;
    scale_msg.data = static_cast<float>(clamp01(speed_scale));
    speed_scale_pub_->publish(scale_msg);
  }

  void control_loop()
  {
    // If the selected command source or drive_switch_node disappears while moving,
    // request a full brake instead of only coasting.
    if (!command_is_fresh()) {
      const double brake =
        previous_requested_forward_speed_mps_ > forward_speed_epsilon_mps_ ?
        emergency_brake_request_ : 0.0;

      publish_outputs(
        make_stop_cmd(), brake, "command_timeout", 0.0, 0.0, 0.0);

      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "SAFETY: /cmd_vel_requested missing or timed out. Stopping.");
      return;
    }

    geometry_msgs::msg::Twist output_cmd = last_requested_cmd_;
    const double requested_forward_speed = std::max(0.0, output_cmd.linear.x);

    // Reverse commands are not blocked by a front-facing distance measurement.
    if (output_cmd.linear.x < -forward_speed_epsilon_mps_) {
      estop_latched_ = false;
      publish_outputs(
        output_cmd, 0.0, "reverse_allowed",
        minimum_clearance_m_,
        minimum_clearance_m_ + slowdown_margin_m_,
        1.0);
      return;
    }

    // A zero command after forward motion requests a short normalized brake pulse.
    if (requested_forward_speed <= forward_speed_epsilon_mps_) {
      if (
        command_stop_brake_enabled_ &&
        this->now() < command_stop_brake_until_)
      {
        publish_outputs(
          make_stop_cmd(),
          command_stop_brake_request_,
          "command_stop_brake",
          minimum_clearance_m_,
          minimum_clearance_m_ + slowdown_margin_m_,
          0.0);

        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "BRAKE: Forward command %.2f m/s changed to stop. "
          "Holding brake request %.2f.",
          command_stop_source_speed_mps_,
          command_stop_brake_request_);
      } else {
        publish_outputs(
          make_stop_cmd(),
          0.0,
          "stopped",
          minimum_clearance_m_,
          minimum_clearance_m_ + slowdown_margin_m_,
          0.0);
      }
      return;
    }

    // Distance AEB can be disabled when a reliable distance source is unavailable.
    if (!distance_safety_enabled_) {
      estop_latched_ = false;
      publish_outputs(
        output_cmd, 0.0, "distance_safety_disabled",
        calculate_stop_distance(requested_forward_speed),
        calculate_stop_distance(requested_forward_speed) + slowdown_margin_m_,
        1.0);
      return;
    }

    if (!distance_is_fresh()) {
      if (require_distance_data_) {
        estop_latched_ = true;
        publish_outputs(
          make_stop_cmd(),
          emergency_brake_request_,
          received_front_distance_ ? "distance_timeout" : "waiting_distance_data",
          calculate_stop_distance(requested_forward_speed),
          calculate_stop_distance(requested_forward_speed) + slowdown_margin_m_,
          0.0);

        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "SAFETY: No fresh /front_distance. Stopping and requesting brake.");
      } else {
        estop_latched_ = false;
        publish_outputs(
          output_cmd, 0.0, "distance_bypass",
          calculate_stop_distance(requested_forward_speed),
          calculate_stop_distance(requested_forward_speed) + slowdown_margin_m_,
          1.0);

        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "SAFETY WARNING: No fresh /front_distance; passing command because "
          "require_distance_data=false.");
      }
      return;
    }

    const double stop_distance =
      calculate_stop_distance(requested_forward_speed);
    const double slowdown_distance =
      stop_distance + slowdown_margin_m_;

    if (
      estop_latched_ &&
      front_distance_m_ <= stop_distance + estop_release_margin_m_)
    {
      publish_outputs(
        make_stop_cmd(),
        emergency_brake_request_,
        "emergency_hold",
        stop_distance,
        slowdown_distance,
        0.0);
      return;
    }

    if (front_distance_m_ <= stop_distance) {
      estop_latched_ = true;
      publish_outputs(
        make_stop_cmd(),
        emergency_brake_request_,
        "emergency_stop",
        stop_distance,
        slowdown_distance,
        0.0);

      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "AEB: front=%.2f m <= stop_distance=%.2f m at requested speed %.2f m/s.",
        front_distance_m_, stop_distance, requested_forward_speed);
      return;
    }

    estop_latched_ = false;

    if (front_distance_m_ < slowdown_distance) {
      const double scale = clamp01(
        (front_distance_m_ - stop_distance) / slowdown_margin_m_);

      // Scale linear and angular command together to preserve approximate curvature.
      output_cmd.linear.x *= scale;
      output_cmd.angular.z *= scale;

      const double brake_request =
        (1.0 - scale) * slowdown_brake_max_request_;

      publish_outputs(
        output_cmd,
        brake_request,
        "slowdown",
        stop_distance,
        slowdown_distance,
        scale);

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "AEB slowdown: front=%.2f m, stop=%.2f m, start=%.2f m, "
        "speed_scale=%.2f, brake_request=%.2f.",
        front_distance_m_, stop_distance, slowdown_distance,
        scale, brake_request);
      return;
    }

    publish_outputs(
      output_cmd,
      0.0,
      "clear",
      stop_distance,
      slowdown_distance,
      1.0);
  }

  double command_timeout_sec_;
  double distance_timeout_sec_;
  double publish_period_sec_;

  bool distance_safety_enabled_;
  bool require_distance_data_;
  double minimum_clearance_m_;
  double reaction_time_sec_;
  double braking_deceleration_mps2_;
  double slowdown_margin_m_;
  double estop_release_margin_m_;

  double slowdown_brake_max_request_;
  double emergency_brake_request_;

  bool command_stop_brake_enabled_;
  double command_stop_brake_request_;
  double command_stop_brake_hold_sec_;
  double forward_speed_epsilon_mps_;

  geometry_msgs::msg::Twist last_requested_cmd_;
  rclcpp::Time last_requested_cmd_time_;
  bool received_requested_cmd_{false};

  double front_distance_m_{0.0};
  rclcpp::Time last_front_distance_time_;
  bool received_front_distance_{false};

  double previous_requested_forward_speed_mps_{0.0};
  double command_stop_source_speed_mps_{0.0};
  rclcpp::Time command_stop_brake_until_;
  bool estop_latched_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr requested_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr front_distance_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr stop_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slowdown_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_scale_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyBrakeNode>());
  rclcpp::shutdown();
  return 0;
}
