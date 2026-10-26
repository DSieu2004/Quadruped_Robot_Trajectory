#include "navigation/waypoint_navigator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace navigation {
WaypointNavigator::WaypointNavigator(std::vector<Waypoint> waypoints, NavigatorConfig config)
    : waypoints_(std::move(waypoints)), config_(config) {
  for (double value : {config.max_speed, config.max_acceleration, config.position_gain, config.tolerance})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Invalid navigator config");
  for (const auto& point : waypoints_)
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) throw std::invalid_argument("Invalid waypoint");
}
void WaypointNavigator::reset() { index_ = 0; command_ = {}; }
double WaypointNavigator::distance(double x, double y) const {
  if (complete()) return 0;
  return std::hypot(waypoints_[index_].x - x, waypoints_[index_].y - y);
}
Command WaypointNavigator::update(double x, double y, double speed, double dt) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(speed) ||
      !std::isfinite(dt) || dt <= 0 || speed < 0) throw std::invalid_argument("Invalid navigation state");
  if (!complete() && distance(x, y) <= config_.tolerance && speed < 0.06) ++index_;
  Command desired;
  if (!complete()) {
    const double dx = waypoints_[index_].x - x;
    const double dy = waypoints_[index_].y - y;
    const double dist = std::hypot(dx, dy);
    const double magnitude = std::min(config_.max_speed, config_.position_gain * dist);
    if (dist > 1e-9) desired = {magnitude * dx / dist, magnitude * dy / dist};
  }
  const double dx = desired.vx - command_.vx;
  const double dy = desired.vy - command_.vy;
  const double delta = std::hypot(dx, dy);
  const double scale = delta > 0 ? std::min(1.0, config_.max_acceleration * dt / delta) : 0;
  command_.vx += scale * dx;
  command_.vy += scale * dy;
  return command_;
}
}  // namespace navigation
