#include "smc/reaching_law.hpp"
#include "pid/pid_controller.hpp"
#include "mpc/mpc_controller.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double pi = 3.14159265358979323846;

double wrap(double angle) { return std::remainder(angle, 2.0 * pi); }

pid::PidGains pid_gains(double kp, double ki, double kd, double output_limit,
                       double integral_limit, double filter_time) {
  pid::PidGains gains;
  gains.kp = kp;
  gains.ki = ki;
  gains.kd = kd;
  gains.derivative_filter_time_constant = filter_time;
  gains.anti_windup_gain = 5.0;
  gains.output_min = -output_limit;
  gains.output_max = output_limit;
  gains.integral_min = -integral_limit;
  gains.integral_max = integral_limit;
  gains.max_dt = 0.25;
  return gains;
}

struct Sample {
  double x = 0, y = 0;
  double vx = 0, vy = 0;
  double ax = 0, ay = 0;
  double yaw = 0, yaw_rate = 0;
};

class TrajectoryFollower final : public rclcpp::Node {
 public:
  TrajectoryFollower() : Node("trajectory_follower") {
    shape_ = declare_parameter<std::string>("shape", "circle");
    control_law_name_ = declare_parameter<std::string>("control_law", "smc");
    period_ = declare_parameter<double>("period", 120.0);
    cycles_ = declare_parameter<double>("cycles", 1.0);
    start_delay_ = declare_parameter<double>("start_delay", 2.0);
    radius_ = declare_parameter<double>("radius", 1.5);
    semi_axis_x_ = declare_parameter<double>("semi_axis_x", 1.2);
    semi_axis_y_ = declare_parameter<double>("semi_axis_y", 0.75);
    max_speed_ = declare_parameter<double>("max_speed", 0.12);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.18);
    smc_velocity_filter_tau_ = declare_parameter<double>("smc_velocity_filter_tau", 0.25);
    smc_yaw_rate_filter_tau_ = declare_parameter<double>("smc_yaw_rate_filter_tau", 0.30);
    mpc_velocity_filter_tau_ = declare_parameter<double>("mpc_velocity_filter_tau", 0.25);
    mpc_yaw_rate_filter_tau_ = declare_parameter<double>("mpc_yaw_rate_filter_tau", 0.30);
    output_csv_ = declare_parameter<std::string>("output_csv", "");
    if (shape_ != "circle" && shape_ != "ellipse" && shape_ != "lemniscate")
      throw std::invalid_argument("shape must be circle, ellipse or lemniscate");
    if (control_law_name_ != "smc" && control_law_name_ != "pid" && control_law_name_ != "mpc")
      throw std::invalid_argument("control_law must be smc, pid or mpc");
    use_pid_ = control_law_name_ == "pid";
    use_mpc_ = control_law_name_ == "mpc";
    for (double value : {period_, cycles_, start_delay_, radius_, semi_axis_x_, semi_axis_y_, max_speed_, max_yaw_rate_,
                         smc_velocity_filter_tau_, smc_yaw_rate_filter_tau_})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Trajectory parameters must be finite and positive");
    for (double value : {mpc_velocity_filter_tau_, mpc_yaw_rate_filter_tau_})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("MPC output filter constants must be finite and positive");
    if (period_ < 2.0 || period_ > 300.0 || cycles_ > 10.0 || period_ * cycles_ > 600.0 ||
        radius_ > 2.0 || semi_axis_x_ > 2.0 || semi_axis_y_ > 2.0 || max_speed_ > 0.12 || max_yaw_rate_ > 0.25)
      throw std::invalid_argument("Trajectory exceeds the supported simulation limits");

    smc::ReachingGains position_gains;
    position_gains.lambda = declare_parameter<double>("lambda", 2.0);
    position_gains.linear_gain = declare_parameter<double>("linear_gain", 1.5);
    position_gains.switching_gain = declare_parameter<double>("switching_gain", 0.04);
    position_gains.boundary_layer = declare_parameter<double>("boundary_layer", 0.45);
    position_gains.acceleration_limit = declare_parameter<double>("acceleration_limit", 0.15);
    position_law_ = smc::ReachingLaw(position_gains);
    smc::ReachingGains yaw_gains;
    yaw_gains.lambda = declare_parameter<double>("yaw_lambda", 3.0);
    yaw_gains.linear_gain = declare_parameter<double>("yaw_linear_gain", 2.5);
    yaw_gains.switching_gain = declare_parameter<double>("yaw_switching_gain", 0.08);
    yaw_gains.boundary_layer = declare_parameter<double>("yaw_boundary_layer", 0.18);
    yaw_gains.acceleration_limit = declare_parameter<double>("yaw_acceleration_limit", 0.35);
    yaw_law_ = smc::ReachingLaw(yaw_gains);
    auto position_pid_gains = pid_gains(
        declare_parameter<double>("pid_kp", 1.6),
        declare_parameter<double>("pid_ki", 0.04),
        declare_parameter<double>("pid_kd", 0.18), max_speed_, 0.035,
        declare_parameter<double>("pid_derivative_filter", 0.15));
    for (auto& controller : position_pid_)
      controller = pid::PidController(position_pid_gains);
    auto yaw_pid_gains = pid_gains(
        declare_parameter<double>("pid_yaw_kp", 1.5),
        declare_parameter<double>("pid_yaw_ki", 0.02),
        declare_parameter<double>("pid_yaw_kd", 0.20), max_yaw_rate_, 0.04,
        declare_parameter<double>("pid_yaw_derivative_filter", 0.12));
    yaw_pid_ = pid::PidController(yaw_pid_gains);
    mpc_horizon_ = static_cast<std::size_t>(declare_parameter<int>("mpc_horizon", 15));
    mpc::MpcConfig position_mpc_config;
    position_mpc_config.horizon_steps = mpc_horizon_;
    position_mpc_config.position_weight = declare_parameter<double>("mpc_position_weight", 20.0);
    position_mpc_config.velocity_weight = declare_parameter<double>("mpc_velocity_weight", 3.0);
    position_mpc_config.acceleration_weight = declare_parameter<double>("mpc_acceleration_weight", 0.5);
    position_mpc_config.acceleration_change_weight = declare_parameter<double>("mpc_acceleration_change_weight", 2.0);
    position_mpc_config.acceleration_change_limit = declare_parameter<double>("mpc_acceleration_change_limit", 0.04);
    position_mpc_config.acceleration_min = -position_gains.acceleration_limit;
    position_mpc_config.acceleration_max = position_gains.acceleration_limit;
    for (auto& controller : position_mpc_)
      controller = mpc::MpcController(position_mpc_config);
    mpc::MpcConfig yaw_mpc_config;
    yaw_mpc_config.horizon_steps = mpc_horizon_;
    yaw_mpc_config.position_weight = declare_parameter<double>("mpc_yaw_position_weight", 8.0);
    yaw_mpc_config.velocity_weight = declare_parameter<double>("mpc_yaw_velocity_weight", 2.0);
    yaw_mpc_config.acceleration_weight = declare_parameter<double>("mpc_yaw_acceleration_weight", 3.0);
    yaw_mpc_config.acceleration_change_weight = declare_parameter<double>("mpc_yaw_acceleration_change_weight", 2.0);
    yaw_mpc_config.acceleration_change_limit = declare_parameter<double>("mpc_yaw_acceleration_change_limit", 0.06);
    yaw_mpc_config.acceleration_min = -yaw_gains.acceleration_limit;
    yaw_mpc_config.acceleration_max = yaw_gains.acceleration_limit;
    yaw_mpc_ = mpc::MpcController(yaw_mpc_config);
    double peak_reference_speed = 0.0;
    constexpr int speed_samples = 2048;
    for (int i = 0; i <= speed_samples; ++i) {
      const double t = period_ * cycles_ * i / speed_samples;
      const auto sample = trajectory(t);
      peak_reference_speed = std::max(peak_reference_speed, std::hypot(sample.vx, sample.vy));
    }
    if (peak_reference_speed > 0.90 * max_speed_)
      throw std::invalid_argument("Trajectory speed exceeds 90% of max_speed; increase period or max_speed to leave tracking headroom");

    const auto path_qos = rclcpp::QoS(1).reliable().transient_local();
    reference_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory/reference", path_qos);
    actual_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory/actual", path_qos);
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    telemetry_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("trajectory/telemetry", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { odom_ = *message; have_odom_ = true; });
    if (!output_csv_.empty()) {
      csv_.open(output_csv_);
      if (!csv_) throw std::runtime_error("Cannot open trajectory CSV: " + output_csv_);
      csv_ << "sim_time,reference_x,reference_y,reference_yaw,actual_x,actual_y,actual_yaw,cmd_vx,cmd_vy,cmd_yaw_rate,controller_x_output,controller_y_output,controller_yaw_output,trajectory_law\n";
      RCLCPP_INFO(get_logger(), "Saving every controller sample to %s", output_csv_.c_str());
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "%s %s trajectory: %.2f s period, %.2f cycles, max speed %.2f m/s",
                control_law_name_.c_str(), shape_.c_str(), period_, cycles_, max_speed_);
  }

 private:
  Sample trajectory(double elapsed) const {
    const double omega = 2.0 * pi / period_;
    const double t = std::clamp(elapsed, 0.0, period_ * cycles_);
    const double phase = omega * t;
    Sample result;
    if (shape_ == "circle") {
      result.x = radius_ * std::sin(phase);
      result.y = radius_ * (1.0 - std::cos(phase));
      result.vx = radius_ * omega * std::cos(phase);
      result.vy = radius_ * omega * std::sin(phase);
      result.ax = -radius_ * omega * omega * std::sin(phase);
      result.ay = radius_ * omega * omega * std::cos(phase);
    } else if (shape_ == "ellipse") {
      result.x = semi_axis_x_ * std::sin(phase);
      result.y = semi_axis_y_ * (1.0 - std::cos(phase));
      result.vx = semi_axis_x_ * omega * std::cos(phase);
      result.vy = semi_axis_y_ * omega * std::sin(phase);
      result.ax = -semi_axis_x_ * omega * omega * std::sin(phase);
      result.ay = semi_axis_y_ * omega * omega * std::cos(phase);
    } else {
      result.x = semi_axis_x_ * std::sin(phase);
      result.y = 0.5 * semi_axis_y_ * std::sin(2.0 * phase);
      result.vx = semi_axis_x_ * omega * std::cos(phase);
      result.vy = semi_axis_y_ * omega * std::cos(2.0 * phase);
      result.ax = -semi_axis_x_ * omega * omega * std::sin(phase);
      result.ay = -2.0 * semi_axis_y_ * omega * omega * std::sin(2.0 * phase);
    }
    const double speed_squared = result.vx * result.vx + result.vy * result.vy;
    result.yaw = std::atan2(result.vy, result.vx);
    result.yaw_rate = speed_squared > 1e-10
        ? (result.vx * result.ay - result.vy * result.ax) / speed_squared : 0.0;
    if (elapsed >= period_ * cycles_) result.vx = result.vy = result.ax = result.ay = result.yaw_rate = 0.0;
    return result;
  }

  geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw, const rclcpp::Time& stamp) const {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = "odom";
    result.header.stamp = stamp;
    result.pose.position.x = x;
    result.pose.position.y = y;
    result.pose.orientation.z = std::sin(0.5 * yaw);
    result.pose.orientation.w = std::cos(0.5 * yaw);
    return result;
  }

  void initialize(const rclcpp::Time& now) {
    start_time_ = now;
    start_x_ = odom_.pose.pose.position.x;
    start_y_ = odom_.pose.pose.position.y;
    start_yaw_ = std::atan2(2.0 * (odom_.pose.pose.orientation.w * odom_.pose.pose.orientation.z),
                             1.0 - 2.0 * std::pow(odom_.pose.pose.orientation.z, 2));
    start_time_ = now + rclcpp::Duration::from_seconds(start_delay_);
    started_ = true;
    const auto initial_reference = trajectory(0.0);
    const double c = std::cos(start_yaw_), s = std::sin(start_yaw_);
    const double initial_ref_x = start_x_ + c * initial_reference.x - s * initial_reference.y;
    const double initial_ref_y = start_y_ + s * initial_reference.x + c * initial_reference.y;
    const double initial_ref_vx = c * initial_reference.vx - s * initial_reference.vy;
    const double initial_ref_vy = s * initial_reference.vx + c * initial_reference.vy;
    const double initial_yaw = start_yaw_;
    const double initial_ref_yaw = wrap(start_yaw_ + initial_reference.yaw);
    position_pid_[0].reset(initial_ref_x, start_x_, initial_ref_vx, initial_ref_vx);
    position_pid_[1].reset(initial_ref_y, start_y_, initial_ref_vy, initial_ref_vy);
    yaw_pid_.reset(0.0, wrap(initial_yaw - initial_ref_yaw),
                   initial_reference.yaw_rate, initial_reference.yaw_rate);
    for (auto& controller : position_mpc_) controller.reset();
    yaw_mpc_.reset();
    mpc_command_filter_initialized_ = false;
    actual_path_.header.frame_id = "odom";
    actual_path_.header.stamp = now;

    reference_path_.header.frame_id = "odom";
    reference_path_.header.stamp = start_time_;
    const double duration = period_ * cycles_;
    const int count = static_cast<int>(std::ceil(duration / 0.1));
    reference_path_.poses.reserve(static_cast<std::size_t>(count + 1));
    for (int i = 0; i <= count; ++i) {
      const double t = std::min(duration, 0.1 * i);
      const auto sample = trajectory(t);
      const double x = start_x_ + std::cos(start_yaw_) * sample.x - std::sin(start_yaw_) * sample.y;
      const double y = start_y_ + std::sin(start_yaw_) * sample.x + std::cos(start_yaw_) * sample.y;
      const double yaw = start_yaw_ + sample.yaw;
      reference_path_.poses.push_back(pose(x, y, yaw, start_time_ + rclcpp::Duration::from_seconds(t)));
    }
    reference_pub_->publish(reference_path_);
    RCLCPP_INFO(get_logger(), "Trajectory starts after %.2f s at odom=(%.2f, %.2f, %.2f); pose timestamps span %.2f s",
                start_delay_, start_x_, start_y_, start_yaw_, duration);
  }

  void tick() {
    geometry_msgs::msg::Twist stop;
    if (!have_odom_) { command_pub_->publish(stop); return; }
    const rclcpp::Time now = get_clock()->now();
    if (!started_) { initialize(now); return; }
    if (now < start_time_) {
      command_pub_->publish(stop);
      last_update_ = now;
      return;
    }
    const double elapsed = std::max(0.0, (now - start_time_).seconds());
    const double dt = last_update_.nanoseconds() > 0
        ? std::clamp((now - last_update_).seconds(), 0.01, 0.20) : 0.05;
    last_update_ = now;
    const auto reference = trajectory(elapsed);
    const double c = std::cos(start_yaw_), s = std::sin(start_yaw_);
    const double ref_x = start_x_ + c * reference.x - s * reference.y;
    const double ref_y = start_y_ + s * reference.x + c * reference.y;
    const double ref_vx = c * reference.vx - s * reference.vy;
    const double ref_vy = s * reference.vx + c * reference.vy;
    const double ref_ax = c * reference.ax - s * reference.ay;
    const double ref_ay = s * reference.ax + c * reference.ay;
    const double ref_yaw = wrap(start_yaw_ + reference.yaw);
    const double yaw = std::atan2(2.0 * (odom_.pose.pose.orientation.w * odom_.pose.pose.orientation.z),
                                  1.0 - 2.0 * std::pow(odom_.pose.pose.orientation.z, 2));
    const double actual_x = odom_.pose.pose.position.x;
    const double actual_y = odom_.pose.pose.position.y;
    const double body_vx = odom_.twist.twist.linear.x;
    const double body_vy = odom_.twist.twist.linear.y;
    const double actual_vx = std::cos(yaw) * body_vx - std::sin(yaw) * body_vy;
    const double actual_vy = std::sin(yaw) * body_vx + std::cos(yaw) * body_vy;
    const double actual_yaw_rate = odom_.twist.twist.angular.z;

    double accel_x = 0.0, accel_y = 0.0, yaw_accel = 0.0;
    double command_vx = 0.0, command_vy = 0.0, command_yaw_rate = 0.0;
    if (use_pid_) {
      command_vx = position_pid_[0].update(ref_x, actual_x, dt, ref_vx);
      command_vy = position_pid_[1].update(ref_y, actual_y, dt, ref_vy);
      const double yaw_error = wrap(yaw - ref_yaw);
      command_yaw_rate = yaw_pid_.update(0.0, yaw_error, dt, reference.yaw_rate);
      accel_x = position_pid_[0].state().output;
      accel_y = position_pid_[1].state().output;
      yaw_accel = yaw_pid_.state().output;
    } else if (use_mpc_) {
      std::vector<mpc::Reference> x_refs, y_refs, yaw_refs;
      x_refs.reserve(mpc_horizon_);
      y_refs.reserve(mpc_horizon_);
      yaw_refs.reserve(mpc_horizon_);
      double unwrapped_yaw_ref = yaw + wrap(ref_yaw - yaw);
      for (std::size_t step = 1; step <= mpc_horizon_; ++step) {
        const auto future = trajectory(elapsed + static_cast<double>(step) * dt);
        const double future_x = start_x_ + c * future.x - s * future.y;
        const double future_y = start_y_ + s * future.x + c * future.y;
        const double future_vx = c * future.vx - s * future.vy;
        const double future_vy = s * future.vx + c * future.vy;
        x_refs.push_back({future_x, future_vx});
        y_refs.push_back({future_y, future_vy});

        const double wrapped_future_yaw = start_yaw_ + future.yaw;
        unwrapped_yaw_ref += wrap(wrapped_future_yaw - unwrapped_yaw_ref);
        yaw_refs.push_back({unwrapped_yaw_ref, future.yaw_rate});
      }
      accel_x = position_mpc_[0].update(actual_x, actual_vx, x_refs, dt);
      accel_y = position_mpc_[1].update(actual_y, actual_vy, y_refs, dt);
      yaw_accel = yaw_mpc_.update(yaw, actual_yaw_rate, yaw_refs, dt);
      command_vx = actual_vx + dt * accel_x;
      command_vy = actual_vy + dt * accel_y;
      command_yaw_rate = std::clamp(actual_yaw_rate + dt * yaw_accel,
                                    -max_yaw_rate_, max_yaw_rate_);
      if (!mpc_command_filter_initialized_) {
        mpc_filtered_vx_ = actual_vx;
        mpc_filtered_vy_ = actual_vy;
        mpc_filtered_yaw_rate_ = actual_yaw_rate;
        mpc_command_filter_initialized_ = true;
      }
      const double velocity_alpha = std::exp(-dt / mpc_velocity_filter_tau_);
      const double yaw_alpha = std::exp(-dt / mpc_yaw_rate_filter_tau_);
      mpc_filtered_vx_ = velocity_alpha * mpc_filtered_vx_ + (1.0 - velocity_alpha) * command_vx;
      mpc_filtered_vy_ = velocity_alpha * mpc_filtered_vy_ + (1.0 - velocity_alpha) * command_vy;
      mpc_filtered_yaw_rate_ = yaw_alpha * mpc_filtered_yaw_rate_ + (1.0 - yaw_alpha) * command_yaw_rate;
      command_vx = mpc_filtered_vx_;
      command_vy = mpc_filtered_vy_;
      command_yaw_rate = mpc_filtered_yaw_rate_;
    } else {
      accel_x = position_law_.acceleration(actual_x - ref_x, actual_vx - ref_vx, ref_ax);
      accel_y = position_law_.acceleration(actual_y - ref_y, actual_vy - ref_vy, ref_ay);
      command_vx = actual_vx + dt * accel_x;
      command_vy = actual_vy + dt * accel_y;
      const double desired_yaw_rate = reference.yaw_rate;
      yaw_accel = yaw_law_.acceleration(wrap(yaw - ref_yaw), actual_yaw_rate - desired_yaw_rate);
      command_yaw_rate = std::clamp(actual_yaw_rate + dt * yaw_accel, -max_yaw_rate_, max_yaw_rate_);

      // Odometry velocity contains gait-frequency ripple. Low-pass the SMC
      // velocity setpoint so that ripple does not become a high-frequency
      // body command; the slow reference motion passes with little lag.
      if (!smc_command_filter_initialized_) {
        smc_filtered_vx_ = actual_vx;
        smc_filtered_vy_ = actual_vy;
        smc_filtered_yaw_rate_ = actual_yaw_rate;
        smc_command_filter_initialized_ = true;
      }
      const double velocity_alpha = std::exp(-dt / smc_velocity_filter_tau_);
      const double yaw_alpha = std::exp(-dt / smc_yaw_rate_filter_tau_);
      smc_filtered_vx_ = velocity_alpha * smc_filtered_vx_ + (1.0 - velocity_alpha) * command_vx;
      smc_filtered_vy_ = velocity_alpha * smc_filtered_vy_ + (1.0 - velocity_alpha) * command_vy;
      smc_filtered_yaw_rate_ = yaw_alpha * smc_filtered_yaw_rate_ + (1.0 - yaw_alpha) * command_yaw_rate;
      command_vx = smc_filtered_vx_;
      command_vy = smc_filtered_vy_;
      command_yaw_rate = smc_filtered_yaw_rate_;
    }
    const double command_speed = std::hypot(command_vx, command_vy);
    if (command_speed > max_speed_) {
      command_vx *= max_speed_ / command_speed;
      command_vy *= max_speed_ / command_speed;
    }
    geometry_msgs::msg::Twist command;
    command.linear.x = std::cos(yaw) * command_vx + std::sin(yaw) * command_vy;
    command.linear.y = -std::sin(yaw) * command_vx + std::cos(yaw) * command_vy;
    command.angular.z = command_yaw_rate;
    command_pub_->publish(command);
    std_msgs::msg::Float64MultiArray telemetry;
    telemetry.data = {elapsed, ref_x, ref_y, actual_x, actual_y, command.linear.x, command.linear.y,
                      command_yaw_rate, accel_x, accel_y, yaw_accel};
    const bool record_sample = elapsed <= period_ * cycles_ + 2.0;
    if (record_sample) telemetry_pub_->publish(telemetry);

    if (record_sample) {
      actual_path_.poses.push_back(pose(actual_x, actual_y, yaw, rclcpp::Time(odom_.header.stamp)));
      if (last_path_publish_.nanoseconds() == 0 || (now - last_path_publish_).seconds() >= 0.2) {
        actual_path_.header.stamp = now;
        actual_pub_->publish(actual_path_);
        last_path_publish_ = now;
      }
    }
    if (csv_.is_open() && record_sample) {
      csv_ << elapsed << ',' << ref_x << ',' << ref_y << ',' << ref_yaw << ','
           << actual_x << ',' << actual_y << ',' << yaw << ',' << command.linear.x << ','
           << command.linear.y << ',' << command.angular.z << ',' << accel_x << ',' << accel_y << ','
           << yaw_accel << ',' << control_law_name_ << '\n';
      if (++csv_samples_ % 100 == 0) csv_.flush();
    }
    if (!completed_ && elapsed >= period_ * cycles_) {
      completed_ = true;
      RCLCPP_INFO(get_logger(), "Timed trajectory complete; returning to its final pose and holding position");
    }
  }

  std::string shape_, output_csv_;
  std::string control_law_name_ = "smc";
  double period_ = 120.0, cycles_ = 1.0, start_delay_ = 2.0, radius_ = 1.5, semi_axis_x_ = 1.2, semi_axis_y_ = 0.75;
  double max_speed_ = 0.12, max_yaw_rate_ = 0.18;
  double smc_velocity_filter_tau_ = 0.25, smc_yaw_rate_filter_tau_ = 0.30;
  double mpc_velocity_filter_tau_ = 0.25, mpc_yaw_rate_filter_tau_ = 0.30;
  double smc_filtered_vx_ = 0.0, smc_filtered_vy_ = 0.0, smc_filtered_yaw_rate_ = 0.0;
  double mpc_filtered_vx_ = 0.0, mpc_filtered_vy_ = 0.0, mpc_filtered_yaw_rate_ = 0.0;
  double start_x_ = 0, start_y_ = 0, start_yaw_ = 0;
  bool have_odom_ = false, started_ = false, completed_ = false;
  bool smc_command_filter_initialized_ = false, mpc_command_filter_initialized_ = false;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME}, last_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_path_publish_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Path reference_path_, actual_path_;
  smc::ReachingLaw position_law_, yaw_law_;
  bool use_pid_ = false, use_mpc_ = false;
  std::array<pid::PidController, 2> position_pid_;
  pid::PidController yaw_pid_;
  std::size_t mpc_horizon_ = 15;
  std::array<mpc::MpcController, 2> position_mpc_;
  mpc::MpcController yaw_mpc_;
  std::ofstream csv_;
  std::size_t csv_samples_ = 0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_, actual_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr telemetry_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TrajectoryFollower>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("trajectory_follower"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
