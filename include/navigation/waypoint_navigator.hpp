#pragma once

#include <cstddef>
#include <vector>

namespace navigation {
struct Waypoint { double x = 0; double y = 0; };
struct Command { double vx = 0; double vy = 0; };
struct NavigatorConfig {
  double max_speed = 0.12;
  double max_acceleration = 0.15;
  double position_gain = 0.8;
  double tolerance = 0.06;
};

// World-frame, holonomic waypoint tracking using a supplied position estimate.
// No mapping, localization, obstacle avoidance or global path planning.
class WaypointNavigator {
 public:
  explicit WaypointNavigator(std::vector<Waypoint> waypoints,
                              NavigatorConfig config = {});
  void reset();
  Command update(double x, double y, double speed, double dt);
  [[nodiscard]] bool complete() const { return index_ == waypoints_.size(); }
  [[nodiscard]] std::size_t index() const { return index_; }
  [[nodiscard]] const std::vector<Waypoint>& waypoints() const { return waypoints_; }
  [[nodiscard]] double distance(double x, double y) const;
 private:
  std::vector<Waypoint> waypoints_;
  NavigatorConfig config_;
  std::size_t index_ = 0;
  Command command_;
};
}  // namespace navigation
