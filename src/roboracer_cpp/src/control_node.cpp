#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"


class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node")
    {
        // Distance-safety parameters.
        this->declare_parameter<double>("safe_distance", 1.7);
        this->declare_parameter<double>("emergency_stop_distance", 0.6);
        this->declare_parameter<double>("data_timeout", 0.5);

        // Autonomous startup defaults.
        // Retained /terminal_settings/* messages replace these values after
        // terminal_command_node starts. The parameters remain useful as
        // safe startup defaults and preserve existing launch compatibility.
        this->declare_parameter<double>("forward_speed", 0.5);
        this->declare_parameter<double>("turn_forward_speed", 0.2);
        this->declare_parameter<double>("turn_speed", 0.6);

        // Intervention compatibility parameters.
        this->declare_parameter<double>("manual_forward_speed", 0.6);
        this->declare_parameter<double>("manual_back_speed", -0.4);
        this->declare_parameter<double>("manual_turn_forward_speed", 0.35);
        this->declare_parameter<double>("manual_turn_speed", 0.7);
        this->declare_parameter<double>("manual_move_duration", 1.0);
        this->declare_parameter<double>("manual_back_duration", 1.0);
        // Declared for launch-file compatibility. Auto left/right
        // no longer use a fixed manual_turn_duration.
        this->declare_parameter<double>("manual_turn_duration", 1.2);

        // Shared bicycle-model settings.
        this->declare_parameter<double>("wheelbase_m", 0.33);
        this->declare_parameter<double>("max_steering_angle_rad", 0.65);
        this->declare_parameter<double>("minimum_turn_speed_mps", 0.05);

        // Time/model-based Auto turn intervention.
        this->declare_parameter<double>("auto_turn_target_deg", 90.0);
        this->declare_parameter<double>("auto_turn_force_angle_deg", 25.0);

        // Distance-scaled normal Auto steering.
        // steering_angle_rad_ remains the maximum steering setting. Normal
        // autonomous obstacle avoidance now starts with a fraction of that
        // angle and increases smoothly as the front obstacle gets closer.
        this->declare_parameter<double>("auto_min_steering_scale", 0.00);
        this->declare_parameter<double>("auto_full_steering_distance_m", 0.90);
        this->declare_parameter<double>("auto_steering_curve_exponent", 1.0);
        // Low-pass LiDAR values for steering only. The downstream AEB still
        // receives the raw /front_distance topic and remains fully responsive.
        this->declare_parameter<double>("auto_distance_filter_alpha", 0.25);
        // Limit how quickly the commanded front-wheel angle may change.
        this->declare_parameter<double>("auto_max_steering_rate_rad_s", 1.20);
        // Require a meaningful left/right advantage before reversing direction.
        this->declare_parameter<double>("auto_direction_switch_deadband_m", 0.25);
        this->declare_parameter<double>("control_period_sec", 0.02);

        this->declare_parameter<double>("safe_cmd_feedback_timeout_sec", 0.60);
        // No IMU/odometry is required. The preferred progress clock integrates
        // the Safety-filtered yaw command. A wall-time formula remains as a
        // fallback when that feedback topic is temporarily unavailable.
        this->declare_parameter<bool>("allow_wall_time_turn_fallback", true);
        this->declare_parameter<std::string>(
            "safe_cmd_feedback_topic", "/cmd_vel_safety_filtered");

        safe_distance_ = this->get_parameter("safe_distance").as_double();
        emergency_stop_distance_ =
            this->get_parameter("emergency_stop_distance").as_double();
        data_timeout_ = this->get_parameter("data_timeout").as_double();

        forward_speed_mps_ = std::abs(
            this->get_parameter("forward_speed").as_double());
        reverse_speed_mps_ = -std::abs(
            this->get_parameter("manual_back_speed").as_double());
        turn_linear_speed_mps_ = std::abs(
            this->get_parameter("turn_forward_speed").as_double());
        turn_speed_follows_forward_ = false;

        const double legacy_turn_yaw_rate = std::abs(
            this->get_parameter("turn_speed").as_double());

        manual_move_duration_ = std::max(
            0.0, this->get_parameter("manual_move_duration").as_double());
        manual_back_duration_ = std::max(
            0.0, this->get_parameter("manual_back_duration").as_double());

        wheelbase_m_ = std::max(
            kEpsilon, std::abs(this->get_parameter("wheelbase_m").as_double()));
        max_steering_angle_rad_ = std::max(
            0.0,
            std::abs(
                this->get_parameter("max_steering_angle_rad").as_double()));
        minimum_turn_speed_mps_ = std::max(
            0.0,
            std::abs(this->get_parameter("minimum_turn_speed_mps").as_double()));

        const double initial_turn_speed = std::max(
            std::abs(turn_linear_speed_mps_), minimum_turn_speed_mps_);
        steering_angle_rad_ = clamp(
            std::atan(wheelbase_m_ * legacy_turn_yaw_rate / initial_turn_speed),
            0.0,
            max_steering_angle_rad_);

        auto_turn_target_rad_ = degrees_to_radians(
            std::abs(this->get_parameter("auto_turn_target_deg").as_double()));
        auto_turn_force_angle_rad_ = clamp(
            degrees_to_radians(std::abs(
                this->get_parameter("auto_turn_force_angle_deg").as_double())),
            0.0,
            auto_turn_target_rad_);

        auto_min_steering_scale_ = clamp(
            this->get_parameter("auto_min_steering_scale").as_double(),
            0.0,
            1.0);
        auto_full_steering_distance_m_ = std::max(
            0.0,
            this->get_parameter("auto_full_steering_distance_m").as_double());
        auto_steering_curve_exponent_ = std::max(
            0.10,
            this->get_parameter("auto_steering_curve_exponent").as_double());
        auto_distance_filter_alpha_ = clamp(
            this->get_parameter("auto_distance_filter_alpha").as_double(),
            0.01,
            1.0);
        auto_max_steering_rate_rad_s_ = std::max(
            0.05,
            this->get_parameter("auto_max_steering_rate_rad_s").as_double());
        auto_direction_switch_deadband_m_ = std::max(
            0.0,
            this->get_parameter("auto_direction_switch_deadband_m").as_double());
        control_period_sec_ = std::max(
            0.01,
            this->get_parameter("control_period_sec").as_double());

        safe_cmd_feedback_timeout_sec_ = std::max(
            0.05,
            this->get_parameter("safe_cmd_feedback_timeout_sec").as_double());
        allow_wall_time_turn_fallback_ =
            this->get_parameter("allow_wall_time_turn_fallback").as_bool();

        front_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/front_distance",
            10,
            std::bind(&ControlNode::front_callback, this, std::placeholders::_1));
        left_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/left_distance",
            10,
            std::bind(&ControlNode::left_callback, this, std::placeholders::_1));
        right_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/right_distance",
            10,
            std::bind(&ControlNode::right_callback, this, std::placeholders::_1));

        manual_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/manual_command",
            10,
            std::bind(&ControlNode::manual_callback, this, std::placeholders::_1));

        // Terminal settings are transient-local so control_node also receives
        // the most recently saved values when nodes start in a different order.
        auto settings_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        settings_qos.reliable();
        settings_qos.transient_local();

        forward_setting_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/terminal_settings/forward_speed_mps",
            settings_qos,
            std::bind(
                &ControlNode::forward_setting_callback,
                this,
                std::placeholders::_1));
        reverse_setting_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/terminal_settings/reverse_speed_mps",
            settings_qos,
            std::bind(
                &ControlNode::reverse_setting_callback,
                this,
                std::placeholders::_1));
        turn_speed_setting_sub_ =
            this->create_subscription<std_msgs::msg::Float32>(
                "/terminal_settings/turn_speed_mps",
                settings_qos,
                std::bind(
                    &ControlNode::turn_speed_setting_callback,
                    this,
                    std::placeholders::_1));
        steering_setting_sub_ =
            this->create_subscription<std_msgs::msg::Float32>(
                "/terminal_settings/steering_angle_rad",
                settings_qos,
                std::bind(
                    &ControlNode::steering_setting_callback,
                    this,
                    std::placeholders::_1));
        turn_follow_setting_sub_ =
            this->create_subscription<std_msgs::msg::Bool>(
                "/terminal_settings/turn_speed_follows_forward",
                settings_qos,
                std::bind(
                    &ControlNode::turn_follow_setting_callback,
                    this,
                    std::placeholders::_1));

        const std::string safe_cmd_topic =
            this->get_parameter("safe_cmd_feedback_topic").as_string();
        safe_cmd_feedback_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                safe_cmd_topic,
                10,
                std::bind(
                    &ControlNode::safe_cmd_feedback_callback,
                    this,
                    std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/auto_cmd_vel", 10);
        intervention_state_pub_ =
            this->create_publisher<std_msgs::msg::String>(
                "/auto_intervention/state", 10);
        intervention_progress_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/auto_intervention/progress_deg", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(control_period_sec_),
            std::bind(&ControlNode::control_loop, this));

        publish_intervention_state("auto");

        RCLCPP_INFO(this->get_logger(), "Control node started.");
        RCLCPP_INFO(
            this->get_logger(),
            "Auto interventions stay inside control_node and publish /auto_cmd_vel.");
        RCLCPP_INFO(
            this->get_logger(),
            "Auto turn: force phase %.1f deg, then continue to the %.1f deg "
            "time/model target. LiDAR road-release is intentionally disabled. "
            "Safety remains active.",
            radians_to_degrees(auto_turn_force_angle_rad_),
            radians_to_degrees(auto_turn_target_rad_));
        RCLCPP_INFO(
            this->get_logger(),
            "Normal Auto steering is distance-scaled: minimum %.0f%% of the "
            "configured steering angle at front=%.2f m, reaching 100%% by "
            "front=%.2f m; curve exponent=%.2f.",
            auto_min_steering_scale_ * 100.0,
            safe_distance_,
            auto_full_steering_distance_m_,
            auto_steering_curve_exponent_);
        RCLCPP_INFO(
            this->get_logger(),
            "Auto smoothing: distance alpha=%.2f, steering-rate limit=%.2f "
            "rad/s, direction deadband=%.2f m, control rate=%.0f Hz.",
            auto_distance_filter_alpha_,
            auto_max_steering_rate_rad_s_,
            auto_direction_switch_deadband_m_,
            1.0 / control_period_sec_);
        RCLCPP_INFO(
            this->get_logger(),
            "No heading sensor is required. Turn progress uses Safety-filtered "
            "command feedback: %s; formula-based wall-time fallback=%s.",
            safe_cmd_topic.c_str(),
            allow_wall_time_turn_fallback_ ? "true" : "false");
    }

private:
    static constexpr double kEpsilon = 1e-6;
    static constexpr double kPi = 3.14159265358979323846;

    static double clamp(double value, double minimum, double maximum)
    {
        return std::max(minimum, std::min(value, maximum));
    }

    static double degrees_to_radians(double degrees)
    {
        return degrees * kPi / 180.0;
    }

    static double radians_to_degrees(double radians)
    {
        return radians * 180.0 / kPi;
    }


    void update_auto_filtered_distance(
        double raw_value,
        double & filtered_value,
        bool & initialized)
    {
        if (!initialized || !std::isfinite(filtered_value))
        {
            filtered_value = raw_value;
            initialized = true;
            return;
        }

        filtered_value +=
            auto_distance_filter_alpha_ * (raw_value - filtered_value);
    }

    void front_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        front_distance_ = msg->data;
        update_auto_filtered_distance(
            front_distance_, auto_front_distance_, auto_front_filter_initialized_);
        received_front_ = true;
        last_front_time_ = this->now();
    }

    void left_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        left_distance_ = msg->data;
        update_auto_filtered_distance(
            left_distance_, auto_left_distance_, auto_left_filter_initialized_);
        received_left_ = true;
        last_left_time_ = this->now();
    }

    void right_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        right_distance_ = msg->data;
        update_auto_filtered_distance(
            right_distance_, auto_right_distance_, auto_right_filter_initialized_);
        received_right_ = true;
        last_right_time_ = this->now();
    }

    void forward_setting_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        forward_speed_mps_ = std::max(0.0, static_cast<double>(msg->data));
        log_shared_settings("forward speed");
    }

    void reverse_setting_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        reverse_speed_mps_ = -std::abs(static_cast<double>(msg->data));
        log_shared_settings("reverse speed");
    }

    void turn_speed_setting_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        turn_linear_speed_mps_ = std::max(0.0, static_cast<double>(msg->data));
        turn_speed_follows_forward_ = false;
        log_shared_settings("turn speed");
    }

    void steering_setting_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        steering_angle_rad_ = clamp(
            std::abs(static_cast<double>(msg->data)),
            0.0,
            max_steering_angle_rad_);
        log_shared_settings("steering angle");
    }

    void turn_follow_setting_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        turn_speed_follows_forward_ = msg->data;
        log_shared_settings("turn-speed follow mode");
    }

    void log_shared_settings(const std::string & changed)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Shared %s updated: forward=%.3f, reverse=%.3f, turn=%.3f%s, "
            "steering=%.3f rad (%.1f deg), yaw_rate=%.3f rad/s",
            changed.c_str(),
            forward_speed_mps_,
            reverse_speed_mps_,
            effective_turn_speed_mps(),
            turn_speed_follows_forward_ ? " (follows forward)" : "",
            steering_angle_rad_,
            radians_to_degrees(steering_angle_rad_),
            effective_turn_yaw_rate());
    }

    void safe_cmd_feedback_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const rclcpp::Time now = this->now();
        const double current_yaw_rate = msg->angular.z;

        if (received_safe_cmd_feedback_)
        {
            const double dt = (now - last_safe_cmd_feedback_time_).seconds();
            if (dt > 0.0 && dt < 1.0)
            {
                const double average_yaw_rate =
                    0.5 * (last_safe_yaw_rate_ + current_yaw_rate);
                safe_cmd_integrated_yaw_rad_ += average_yaw_rate * dt;
            }
        }

        last_safe_yaw_rate_ = current_yaw_rate;
        received_safe_cmd_feedback_ = true;
        last_safe_cmd_feedback_time_ = now;
    }


    bool safe_cmd_feedback_fresh() const
    {
        if (!received_safe_cmd_feedback_)
        {
            return false;
        }
        return (this->now() - last_safe_cmd_feedback_time_).seconds() <=
               safe_cmd_feedback_timeout_sec_;
    }

    double effective_turn_speed_mps() const
    {
        return turn_speed_follows_forward_
                   ? forward_speed_mps_
                   : turn_linear_speed_mps_;
    }

    double yaw_rate_for_steering(
        double linear_speed_mps,
        double steering_angle_rad) const
    {
        const double speed = std::abs(linear_speed_mps);
        const double steering = std::abs(steering_angle_rad);

        if (speed < minimum_turn_speed_mps_ || steering <= kEpsilon)
        {
            return 0.0;
        }

        return speed / wheelbase_m_ * std::tan(steering);
    }

    double effective_turn_yaw_rate() const
    {
        return yaw_rate_for_steering(
            effective_turn_speed_mps(),
            steering_angle_rad_);
    }

    double auto_steering_urgency() const
    {
        if (auto_front_distance_ >= safe_distance_)
        {
            return 0.0;
        }

        const double scaling_span =
            safe_distance_ - auto_full_steering_distance_m_;

        if (scaling_span <= kEpsilon)
        {
            return 1.0;
        }

        const double linear_urgency =
            (safe_distance_ - auto_front_distance_) / scaling_span;
        return clamp(linear_urgency, 0.0, 1.0);
    }

    double adaptive_auto_steering_scale() const
    {
        const double urgency = auto_steering_urgency();
        // Smoothstep has zero slope at both ends, avoiding a sharp change when
        // entering the avoidance range or reaching full steering.
        const double smooth_urgency =
            urgency * urgency * (3.0 - 2.0 * urgency);
        const double shaped_urgency = std::pow(
            smooth_urgency, auto_steering_curve_exponent_);

        return auto_min_steering_scale_ +
               (1.0 - auto_min_steering_scale_) * shaped_urgency;
    }

    double adaptive_auto_steering_angle_rad() const
    {
        return clamp(
            steering_angle_rad_ * adaptive_auto_steering_scale(),
            0.0,
            max_steering_angle_rad_);
    }

    int choose_auto_turn_direction()
    {
        const double side_difference =
            auto_left_distance_ - auto_right_distance_;

        if (auto_turn_direction_sign_ == 0)
        {
            auto_turn_direction_sign_ = side_difference >= 0.0 ? 1 : -1;
        }
        else if (
            auto_turn_direction_sign_ > 0 &&
            side_difference < -auto_direction_switch_deadband_m_)
        {
            auto_turn_direction_sign_ = -1;
        }
        else if (
            auto_turn_direction_sign_ < 0 &&
            side_difference > auto_direction_switch_deadband_m_)
        {
            auto_turn_direction_sign_ = 1;
        }

        return auto_turn_direction_sign_;
    }

    double rate_limit_auto_steering(double target_signed_angle_rad)
    {
        const rclcpp::Time now = this->now();
        if (!auto_steering_rate_initialized_)
        {
            last_auto_steering_update_time_ = now;
            current_auto_signed_steering_rad_ = 0.0;
            auto_steering_rate_initialized_ = true;
            return current_auto_signed_steering_rad_;
        }

        const double dt = clamp(
            (now - last_auto_steering_update_time_).seconds(),
            0.0,
            0.20);
        last_auto_steering_update_time_ = now;

        const double maximum_change = auto_max_steering_rate_rad_s_ * dt;
        const double requested_change =
            target_signed_angle_rad - current_auto_signed_steering_rad_;
        current_auto_signed_steering_rad_ += clamp(
            requested_change, -maximum_change, maximum_change);

        if (std::abs(current_auto_signed_steering_rad_) < 1e-4)
        {
            current_auto_signed_steering_rad_ = 0.0;
        }
        return current_auto_signed_steering_rad_;
    }

    void reset_auto_steering_smoother()
    {
        current_auto_signed_steering_rad_ = 0.0;
        auto_steering_rate_initialized_ = false;
    }

    void make_stop_command(geometry_msgs::msg::Twist & cmd) const
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }

    void publish_intervention_state(const std::string & state)
    {
        if (state == last_published_intervention_state_)
        {
            return;
        }
        last_published_intervention_state_ = state;
        std_msgs::msg::String msg;
        msg.data = state;
        intervention_state_pub_->publish(msg);
    }

    void publish_turn_progress(double progress_rad)
    {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(radians_to_degrees(progress_rad));
        intervention_progress_pub_->publish(msg);
    }

    void clear_turn_intervention()
    {
        turn_intervention_active_ = false;
        turn_direction_sign_ = 0;
        turn_start_safe_integral_rad_ = safe_cmd_integrated_yaw_rad_;
        turn_nominal_yaw_rate_rad_s_ = 0.0;
        turn_nominal_force_duration_sec_ = 0.0;
        turn_nominal_target_duration_sec_ = 0.0;
    }

    void return_to_auto(const std::string & reason)
    {
        manual_mode_ = "auto";
        manual_has_end_time_ = false;
        clear_turn_intervention();
        reset_auto_steering_smoother();
        publish_intervention_state("auto");
        RCLCPP_INFO(this->get_logger(), "%s Returning to autonomous control.", reason.c_str());
    }

    void start_temporary_intervention(
        const std::string & mode,
        double duration_sec)
    {
        clear_turn_intervention();
        manual_mode_ = mode;
        manual_end_time_ =
            this->now() + rclcpp::Duration::from_seconds(duration_sec);
        manual_has_end_time_ = true;
        publish_intervention_state(mode);

        RCLCPP_INFO(
            this->get_logger(),
            "Temporary Auto intervention: %s for %.2f seconds",
            mode.c_str(),
            duration_sec);
    }

    void start_turn_intervention(const std::string & mode)
    {
        const int direction_sign = mode == "left" ? 1 : -1;
        const double nominal_yaw_rate = effective_turn_yaw_rate();

        if (nominal_yaw_rate <= kEpsilon)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Cannot start Auto %s intervention: ts#/tsauto and sa#/sad# "
                "produce zero turn rate.",
                mode.c_str());
            return_to_auto("Turn intervention rejected.");
            return;
        }

        manual_mode_ = mode;
        manual_has_end_time_ = false;
        turn_intervention_active_ = true;
        turn_direction_sign_ = direction_sign;
        turn_start_time_ = this->now();
        turn_start_safe_integral_rad_ = safe_cmd_integrated_yaw_rad_;
        turn_nominal_yaw_rate_rad_s_ = nominal_yaw_rate;
        turn_nominal_force_duration_sec_ =
            auto_turn_force_angle_rad_ / nominal_yaw_rate;
        turn_nominal_target_duration_sec_ =
            auto_turn_target_rad_ / nominal_yaw_rate;

        publish_intervention_state(mode + "_forced");

        RCLCPP_WARN(
            this->get_logger(),
            "Auto %s intervention started: v=%.3f m/s, steering=%.3f rad "
            "(%.1f deg), yaw_rate=%.3f rad/s, force=%.1f deg / %.2fs nominal, "
            "target=%.1f deg / %.2fs nominal. No LiDAR early release is used. "
            "A new 2/3 command immediately replaces this intervention. Safety "
            "always retains priority; slowdown stretches effective time and "
            "E-stop pauses it.",
            mode.c_str(),
            effective_turn_speed_mps(),
            steering_angle_rad_,
            radians_to_degrees(steering_angle_rad_),
            nominal_yaw_rate,
            radians_to_degrees(auto_turn_force_angle_rad_),
            turn_nominal_force_duration_sec_,
            radians_to_degrees(auto_turn_target_rad_),
            turn_nominal_target_duration_sec_);
    }

    void manual_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string command = msg->data;

        if (command == "auto")
        {
            return_to_auto("Auto command received.");
        }
        else if (command == "move")
        {
            start_temporary_intervention("move", manual_move_duration_);
        }
        else if (command == "back")
        {
            start_temporary_intervention("back", manual_back_duration_);
        }
        else if (command == "stop")
        {
            clear_turn_intervention();
            manual_mode_ = "stop";
            manual_has_end_time_ = false;
            publish_intervention_state("stop");
            RCLCPP_INFO(this->get_logger(), "Auto-path intervention command: stop");
        }
        else if (command == "left" || command == "right")
        {
            // Calling this while another turn is active intentionally restarts
            // the intervention in the newly requested direction.
            start_turn_intervention(command);
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown /manual_command: %s",
                command.c_str());
        }
    }

    bool distance_data_ready() const
    {
        return received_front_ && received_left_ && received_right_;
    }

    bool distance_data_fresh() const
    {
        if (!distance_data_ready())
        {
            return false;
        }

        const auto now = this->now();
        return (
            (now - last_front_time_).seconds() <= data_timeout_ &&
            (now - last_left_time_).seconds() <= data_timeout_ &&
            (now - last_right_time_).seconds() <= data_timeout_);
    }

    bool manual_escape_allowed() const
    {
        return manual_mode_ == "back" || manual_mode_ == "stop";
    }

    bool apply_safety_check(geometry_msgs::msg::Twist & cmd)
    {
        if (!distance_data_ready())
        {
            make_stop_command(cmd);
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "SAFETY: Waiting for complete distance data. Stopping.");
            return true;
        }

        if (!distance_data_fresh())
        {
            make_stop_command(cmd);
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "SAFETY: Distance data timeout. Stopping.");
            return true;
        }

        if (front_distance_ <= emergency_stop_distance_)
        {
            if (manual_escape_allowed())
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "SAFETY: Obstacle very close, but escape command is allowed. "
                    "front=%.2f",
                    front_distance_);
                return false;
            }

            make_stop_command(cmd);
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "SAFETY: Emergency stop. front=%.2f <= %.2f",
                front_distance_,
                emergency_stop_distance_);
            return true;
        }

        return false;
    }

    std::optional<double> current_turn_progress_rad() const
    {
        if (!turn_intervention_active_ || turn_direction_sign_ == 0)
        {
            return std::nullopt;
        }

        if (safe_cmd_feedback_fresh())
        {
            // /cmd_vel_safety_filtered is downstream of Safety Brake but
            // upstream of the controller mux. Integrating it gives a
            // Safety-aware effective turn clock without adding an IMU:
            // slowdown counts proportionally and emergency stop counts zero.
            const double signed_progress =
                static_cast<double>(turn_direction_sign_) *
                (safe_cmd_integrated_yaw_rad_ -
                 turn_start_safe_integral_rad_);
            return std::max(0.0, signed_progress);
        }

        if (allow_wall_time_turn_fallback_ &&
            turn_nominal_yaw_rate_rad_s_ > kEpsilon)
        {
            // Formula-only fallback:
            // yaw_rate = v / L * tan(delta), progress = yaw_rate * time.
            // This cannot observe wheel slip, but it needs no new hardware.
            const double elapsed = (this->now() - turn_start_time_).seconds();
            return std::max(0.0, elapsed * turn_nominal_yaw_rate_rad_s_);
        }

        return std::nullopt;
    }

    bool update_turn_completion()
    {
        const std::optional<double> progress = current_turn_progress_rad();
        if (!progress.has_value())
        {
            publish_intervention_state(
                manual_mode_ + "_waiting_for_safe_feedback");
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Auto turn is waiting for Safety-filtered command feedback. "
                "Enable the formula fallback or verify the configured topic.");
            return false;
        }

        publish_turn_progress(*progress);

        if (*progress >= auto_turn_target_rad_)
        {
            return_to_auto(
                "Auto turn reached its maximum target angle.");
            return true;
        }

        const bool forced_phase_complete =
            *progress >= auto_turn_force_angle_rad_;

        if (!forced_phase_complete)
        {
            publish_intervention_state(manual_mode_ + "_forced");
            return false;
        }

        // Do not use LiDAR or road geometry for early intervention release.
        // After the minimum forced correction, continue the same requested
        // direction until the formula-derived target progress is reached.
        publish_intervention_state(manual_mode_ + "_targeting");
        return false;
    }

    bool timed_intervention_finished() const
    {
        return manual_has_end_time_ && this->now() >= manual_end_time_;
    }

    bool apply_manual_control(geometry_msgs::msg::Twist & cmd)
    {
        if (manual_mode_ == "auto")
        {
            return false;
        }

        if (turn_intervention_active_)
        {
            if (update_turn_completion())
            {
                return false;
            }

            const double turn_speed = effective_turn_speed_mps();
            const double yaw_rate = effective_turn_yaw_rate();
            if (turn_speed < minimum_turn_speed_mps_ || yaw_rate <= kEpsilon)
            {
                make_stop_command(cmd);
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Auto turn paused because ts#/tsauto or sa#/sad# now "
                    "produces zero effective turn rate.");
                return true;
            }

            cmd.linear.x = turn_speed;
            cmd.angular.z =
                static_cast<double>(turn_direction_sign_) * yaw_rate;
            return true;
        }

        if (timed_intervention_finished())
        {
            return_to_auto("Temporary intervention finished.");
            return false;
        }

        if (manual_mode_ == "move")
        {
            cmd.linear.x = forward_speed_mps_;
            cmd.angular.z = 0.0;
        }
        else if (manual_mode_ == "back")
        {
            cmd.linear.x = reverse_speed_mps_;
            cmd.angular.z = 0.0;
        }
        else if (manual_mode_ == "stop")
        {
            make_stop_command(cmd);
        }
        else
        {
            return false;
        }

        return true;
    }

    void apply_auto_control(geometry_msgs::msg::Twist & cmd)
    {
        const bool path_clear = auto_front_distance_ >= safe_distance_;
        const double steering_scale =
            path_clear ? 0.0 : adaptive_auto_steering_scale();
        const double target_angle_magnitude =
            path_clear ? 0.0 : adaptive_auto_steering_angle_rad();
        const int direction_sign =
            path_clear ? auto_turn_direction_sign_ : choose_auto_turn_direction();
        const double target_signed_angle =
            static_cast<double>(direction_sign) * target_angle_magnitude;
        const double smoothed_signed_angle =
            rate_limit_auto_steering(target_signed_angle);

        const double commanded_speed =
            path_clear ? forward_speed_mps_ : effective_turn_speed_mps();
        const double yaw_rate_magnitude = yaw_rate_for_steering(
            commanded_speed, smoothed_signed_angle);

        if (
            !path_clear &&
            (commanded_speed < minimum_turn_speed_mps_ ||
             adaptive_auto_steering_angle_rad() <= kEpsilon))
        {
            make_stop_command(cmd);
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "AUTO: Cannot turn because shared ts#/tsauto or sa#/sad# "
                "produces zero effective turn rate.");
            return;
        }

        cmd.linear.x = commanded_speed;
        cmd.angular.z =
            smoothed_signed_angle >= 0.0 ? yaw_rate_magnitude : -yaw_rate_magnitude;

        if (path_clear)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "AUTO: Path clear. raw_front=%.2f, filtered_front=%.2f. "
                "Steering returning smoothly to center: %.1f deg.",
                front_distance_,
                auto_front_distance_,
                radians_to_degrees(smoothed_signed_angle));
            return;
        }

        const char * direction_name = direction_sign > 0 ? "LEFT" : "RIGHT";
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "AUTO: raw_front=%.2f, filtered_front=%.2f, "
            "filtered_left=%.2f, filtered_right=%.2f. Turning %s with "
            "target=%.1f deg, smoothed=%.1f deg (%.0f%% of configured %.1f deg).",
            front_distance_,
            auto_front_distance_,
            auto_left_distance_,
            auto_right_distance_,
            direction_name,
            radians_to_degrees(target_signed_angle),
            radians_to_degrees(smoothed_signed_angle),
            steering_scale * 100.0,
            radians_to_degrees(steering_angle_rad_));
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        const bool intervention_used = apply_manual_control(cmd);
        if (!intervention_used)
        {
            apply_auto_control(cmd);
        }

        // Local fail-safe is applied after the desired command is generated.
        // The system-level safety_brake_node remains downstream and therefore
        // has priority over Auto/Terminal intervention commands. A blocked turn
        // remains selected, but its Safety-filtered effective clock pauses.
        apply_safety_check(cmd);
        cmd_pub_->publish(cmd);
    }

    // Distance safety.
    double safe_distance_ = 1.7;
    double emergency_stop_distance_ = 0.6;
    double data_timeout_ = 0.5;

    double front_distance_ = 0.0;
    double left_distance_ = 0.0;
    double right_distance_ = 0.0;

    // Filtered copies are used only for normal Auto steering decisions.
    double auto_front_distance_ = 0.0;
    double auto_left_distance_ = 0.0;
    double auto_right_distance_ = 0.0;
    bool auto_front_filter_initialized_ = false;
    bool auto_left_filter_initialized_ = false;
    bool auto_right_filter_initialized_ = false;

    bool received_front_ = false;
    bool received_left_ = false;
    bool received_right_ = false;
    rclcpp::Time last_front_time_;
    rclcpp::Time last_left_time_;
    rclcpp::Time last_right_time_;

    // Persistent shared vehicle command settings.
    double forward_speed_mps_ = 0.5;
    double reverse_speed_mps_ = -0.4;
    double turn_linear_speed_mps_ = 0.2;
    double steering_angle_rad_ = 0.3;
    bool turn_speed_follows_forward_ = false;
    double wheelbase_m_ = 0.33;
    double max_steering_angle_rad_ = 0.65;
    double minimum_turn_speed_mps_ = 0.05;

    // Auto intervention state.
    std::string manual_mode_ = "auto";
    rclcpp::Time manual_end_time_;
    bool manual_has_end_time_ = false;
    double manual_move_duration_ = 1.0;
    double manual_back_duration_ = 1.0;

    bool turn_intervention_active_ = false;
    int turn_direction_sign_ = 0;
    rclcpp::Time turn_start_time_;
    double turn_start_safe_integral_rad_ = 0.0;
    double turn_nominal_yaw_rate_rad_s_ = 0.0;
    double turn_nominal_force_duration_sec_ = 0.0;
    double turn_nominal_target_duration_sec_ = 0.0;

    double auto_turn_target_rad_ = kPi / 2.0;
    double auto_turn_force_angle_rad_ = degrees_to_radians(25.0);

    // Normal autonomous obstacle-avoidance steering profile.
    double auto_min_steering_scale_ = 0.00;
    double auto_full_steering_distance_m_ = 0.90;
    double auto_steering_curve_exponent_ = 1.0;
    double auto_distance_filter_alpha_ = 0.25;
    double auto_max_steering_rate_rad_s_ = 1.20;
    double auto_direction_switch_deadband_m_ = 0.25;
    double control_period_sec_ = 0.02;

    int auto_turn_direction_sign_ = 0;
    double current_auto_signed_steering_rad_ = 0.0;
    bool auto_steering_rate_initialized_ = false;
    rclcpp::Time last_auto_steering_update_time_;

    bool allow_wall_time_turn_fallback_ = true;

    // Safety-filtered yaw-rate integration. This automatically pauses
    // when Safety outputs zero and stretches when Safety slows the command.
    bool received_safe_cmd_feedback_ = false;
    double last_safe_yaw_rate_ = 0.0;
    double safe_cmd_integrated_yaw_rad_ = 0.0;
    rclcpp::Time last_safe_cmd_feedback_time_;
    double safe_cmd_feedback_timeout_sec_ = 0.60;

    std::string last_published_intervention_state_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr front_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_sub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        forward_setting_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        reverse_setting_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        turn_speed_setting_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        steering_setting_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        turn_follow_setting_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        safe_cmd_feedback_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr intervention_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        intervention_progress_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
