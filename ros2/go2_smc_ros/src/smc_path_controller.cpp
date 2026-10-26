#include "smc/reaching_law.hpp"
#include "pid/pid_controller.hpp"
#include <nav2_core/controller.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace go2_smc_ros {
namespace {
constexpr double pi = 3.14159265358979323846;
double wrap(double angle) { return std::remainder(angle, 2 * pi); }
}

// Nav2 supplies a global path. SMC tracks position AND heading, then aligns
// to the quaternion requested by the RViz goal, using real motor-driven turns.
class SmcPathController : public nav2_core::Controller {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_ = parent.lock();
    tf_ = std::move(tf);
    costmap_ = std::move(costmap);
    if (!node_) throw std::runtime_error("Missing controller node");
    auto parameter = [&](const std::string& key, double value) {
      const auto full = name + "." + key;
      if (!node_->has_parameter(full)) node_->declare_parameter(full, value);
      return node_->get_parameter(full).as_double();
    };
    const auto law_name = name + ".control_law";
    if (!node_->has_parameter(law_name)) node_->declare_parameter(law_name, std::string("smc"));
    const auto control_law = node_->get_parameter(law_name).as_string();
    if (control_law != "smc" && control_law != "pid")
      throw std::runtime_error("FollowPath.control_law must be smc or pid");
    use_pid_ = control_law == "pid";
    max_speed_ = parameter("max_speed", 0.10);
    lookahead_ = parameter("lookahead", 0.22);
    max_yaw_rate_ = parameter("max_yaw_rate", 0.18);
    rotate_threshold_ = parameter("rotate_threshold", 0.55);
    align_distance_ = parameter("align_goal_distance", 0.10);
    smc::ReachingGains gains;
    gains.lambda = parameter("lambda", 3);
    gains.linear_gain = parameter("linear_gain", 3);
    gains.switching_gain = parameter("switching_gain", 0.12);
    gains.boundary_layer = parameter("boundary_layer", 0.22);
    gains.acceleration_limit = parameter("acceleration_limit", 0.15);
    law_ = smc::ReachingLaw(gains);
    smc::ReachingGains yaw_gains;
    yaw_gains.lambda = parameter("yaw_lambda", 3);
    yaw_gains.linear_gain = parameter("yaw_linear_gain", 3);
    yaw_gains.switching_gain = parameter("yaw_switching_gain", 0.10);
    yaw_gains.boundary_layer = parameter("yaw_boundary_layer", 0.18);
    yaw_gains.acceleration_limit = parameter("yaw_acceleration_limit", 0.35);
    yaw_law_ = smc::ReachingLaw(yaw_gains);
    auto path_gains = pid::PidGains{};
    path_gains.kp = parameter("pid_kp", 16.0);
    path_gains.ki = parameter("pid_ki", 0.4);
    path_gains.kd = parameter("pid_kd", 8.0);
    path_gains.output_min = -gains.acceleration_limit;
    path_gains.output_max = gains.acceleration_limit;
    path_gains.integral_min = -0.10;
    path_gains.integral_max = 0.10;
    path_pid_[0] = pid::PidController(path_gains);
    path_pid_[1] = pid::PidController(path_gains);
    auto yaw_pid_gains = path_gains;
    yaw_pid_gains.kp = parameter("pid_yaw_kp", 16.0);
    yaw_pid_gains.ki = parameter("pid_yaw_ki", 0.4);
    yaw_pid_gains.kd = parameter("pid_yaw_kd", 8.0);
    yaw_pid_gains.output_min = -yaw_gains.acceleration_limit;
    yaw_pid_gains.output_max = yaw_gains.acceleration_limit;
    yaw_pid_[0] = pid::PidController(yaw_pid_gains);
    for (double value : {max_speed_, lookahead_, max_yaw_rate_, rotate_threshold_, align_distance_})
      if (!std::isfinite(value) || value <= 0) throw std::runtime_error("Invalid SMC path settings");
    if (max_speed_ > 0.12 || max_yaw_rate_ > 0.25 || rotate_threshold_ > pi)
      throw std::runtime_error("SMC path limits exceed supported gait");
    nominal_speed_ = max_speed_;
    RCLCPP_INFO(node_->get_logger(), "%s path/heading: %.2f m/s, %.2f rad/s",
                use_pid_ ? "PID" : "SMC", max_speed_, max_yaw_rate_);
  }

  void cleanup() override {
    path_.poses.clear();
    aligning_goal_ = turning_path_ = false;
    reset_pid();
  }
  void activate() override {}
  void deactivate() override {}
  void setPlan(const nav_msgs::msg::Path& path) override {
    bool new_goal = path_.poses.empty() || path.poses.empty();
    if (!new_goal) {
      const auto& previous = path_.poses.back().pose;
      const auto& next = path.poses.back().pose;
      new_goal = std::hypot(previous.position.x - next.position.x,
                            previous.position.y - next.position.y) > 1e-4 ||
                 std::abs(wrap(tf2::getYaw(previous.orientation) - tf2::getYaw(next.orientation))) > 1e-4;
    }
    if (new_goal) {
      aligning_goal_ = turning_path_ = false;
      reset_pid();
    }
    path_ = path;
  }
  void setSpeedLimit(const double& limit, const bool& percentage) override {
    max_speed_ = limit <= 0 ? nominal_speed_ :
        std::clamp(percentage ? nominal_speed_ * limit / 100.0 : limit, 0.0, nominal_speed_);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker*) override {
    if (path_.poses.empty()) throw nav2_core::PlannerException("Empty SMC path");
    const auto now = node_->now();
    double dt = last_update_.nanoseconds() > 0 ? (now - last_update_).seconds() : 0.1;
    if (!std::isfinite(dt) || dt <= 0 || dt > 0.25) {
      reset_pid();
      dt = 0.1;
    }
    last_update_ = now;
    geometry_msgs::msg::PoseStamped robot;
    try {
      robot = tf_->transform(pose, path_.header.frame_id, tf2::durationFromSec(0.3));
    } catch (const tf2::TransformException& error) {
      throw nav2_core::PlannerException(error.what());
    }
    const double x = robot.pose.position.x, y = robot.pose.position.y;
    const double yaw = tf2::getYaw(robot.pose.orientation);
    const auto& goal = path_.poses.back().pose;
    const double goal_distance = std::hypot(goal.position.x - x, goal.position.y - y);
    if (goal_distance <= align_distance_) aligning_goal_ = true;
    if (goal_distance > 1.8 * align_distance_) aligning_goal_ = false;
    if (aligning_goal_) {
      auto result = command();
      result.twist.angular.z = yaw_command(wrap(yaw - tf2::getYaw(goal.orientation)), velocity.angular.z, dt);
      if (safe(pose, velocity, result.twist)) return result;
      throw nav2_core::PlannerException("Goal rotation footprint is blocked");
    }

    std::size_t closest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path_.poses.size(); ++i) {
      const auto& point = path_.poses[i].pose.position;
      const double distance = std::hypot(point.x - x, point.y - y);
      if (distance < best) { best = distance; closest = i; }
    }
    const double vx = std::cos(yaw) * velocity.linear.x - std::sin(yaw) * velocity.linear.y;
    const double vy = std::sin(yaw) * velocity.linear.x + std::cos(yaw) * velocity.linear.y;
    for (double look = lookahead_; look >= 0.025; look *= 0.5) {
      auto target = path_.poses[closest].pose.position;
      double distance = 0;
      for (std::size_t i = closest + 1; i < path_.poses.size(); ++i) {
        const auto& point = path_.poses[i].pose.position;
        const double length = std::hypot(point.x - target.x, point.y - target.y);
        if (distance + length >= look && length > 1e-9) {
          const double u = (look - distance) / length;
          target.x += u * (point.x - target.x);
          target.y += u * (point.y - target.y);
          break;
        }
        distance += length;
        target = point;
      }
      const double bearing = std::atan2(target.y - y, target.x - x);
      const double heading_error = wrap(yaw - bearing);
      // Hysteresis: turn first at a sharp corner, then walk mainly forward.
      if (std::abs(heading_error) > rotate_threshold_) turning_path_ = true;
      if (std::abs(heading_error) < 0.20) turning_path_ = false;
      auto result = command();
      result.twist.angular.z = yaw_command(heading_error, velocity.angular.z, dt);
      if (!turning_path_) {
        // Acceleration output is integrated into a velocity command at the controller rate.
        const double ax = use_pid_ ? path_pid_[0].update(target.x, x, dt)
                                   : law_.acceleration(x - target.x, vx);
        const double ay = use_pid_ ? path_pid_[1].update(target.y, y, dt)
                                   : law_.acceleration(y - target.y, vy);
        double ux = vx + dt * ax;
        double uy = vy + dt * ay;
        const double norm = std::hypot(ux, uy);
        if (norm > max_speed_) { ux *= max_speed_ / norm; uy *= max_speed_ / norm; }
        const double scale = std::max(0.0, std::cos(heading_error));
        result.twist.linear.x = std::max(0.0, std::cos(yaw) * ux + std::sin(yaw) * uy) * scale;
        result.twist.linear.y = std::clamp(-std::sin(yaw) * ux + std::cos(yaw) * uy, -0.025, 0.025) * scale;
      }
      if (safe(pose, velocity, result.twist)) return result;
    }
    throw nav2_core::PlannerException("SMC footprint prediction blocked; awaiting global replan");
  }

 private:
  geometry_msgs::msg::TwistStamped command() const {
    geometry_msgs::msg::TwistStamped result;
    result.header.stamp = node_->now();
    result.header.frame_id = costmap_->getBaseFrameID();
    return result;
  }
  double yaw_command(double error, double measured_rate, double dt) {
    const double acceleration = use_pid_ ? yaw_pid_[0].update(0.0, error, dt)
                                         : yaw_law_.acceleration(error, measured_rate);
    return std::clamp(measured_rate + dt * acceleration,
                      -max_yaw_rate_, max_yaw_rate_);
  }
  void reset_pid() {
    for (auto& controller : path_pid_) controller.reset();
    for (auto& controller : yaw_pid_) controller.reset();
    last_update_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  bool safe(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
            const geometry_msgs::msg::Twist& requested) const {
    auto* map = costmap_->getCostmap();
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(map);
    const auto footprint = costmap_->getRobotFootprint();
    double x = pose.pose.position.x, y = pose.pose.position.y;
    double yaw = tf2::getYaw(pose.pose.orientation);
    double vx = velocity.linear.x, vy = velocity.linear.y, rate = velocity.angular.z;
    constexpr double dt = 1.0 / 12;
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    for (int k = 0; k <= 12; ++k) {
      const double cost = checker.footprintCostAtPose(x, y, yaw, footprint);
      unsigned int mx, my;
      if (cost < 0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE ||
          !map->worldToMap(x, y, mx, my) || map->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE)
        return false;
      if (k == 12) break;
      // Include braking and yaw motion rather than assuming an instantaneous stop.
      vx += std::clamp(requested.linear.x - vx, -0.15 * dt, 0.15 * dt);
      vy += std::clamp(requested.linear.y - vy, -0.15 * dt, 0.15 * dt);
      rate += std::clamp(requested.angular.z - rate, -0.35 * dt, 0.35 * dt);
      yaw += 0.5 * rate * dt;
      x += (std::cos(yaw) * vx - std::sin(yaw) * vy) * dt;
      y += (std::sin(yaw) * vx + std::cos(yaw) * vy) * dt;
      yaw += 0.5 * rate * dt;
    }
    return true;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  nav_msgs::msg::Path path_;
  smc::ReachingLaw law_, yaw_law_;
  std::array<pid::PidController, 2> path_pid_;
  std::array<pid::PidController, 1> yaw_pid_;
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
  double max_speed_ = 0.10, nominal_speed_ = 0.10, lookahead_ = 0.22;
  double max_yaw_rate_ = 0.18, rotate_threshold_ = 0.55, align_distance_ = 0.10;
  bool aligning_goal_ = false, turning_path_ = false, use_pid_ = false;
};
}
PLUGINLIB_EXPORT_CLASS(go2_smc_ros::SmcPathController, nav2_core::Controller)
