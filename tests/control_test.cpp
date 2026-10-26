#include "locomotion/force_allocator.hpp"
#include "navigation/waypoint_navigator.hpp"
#include "smc/reaching_law.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
}
int main() {
  const smc::ReachingLaw law({5, 4, 2, 0.1, 1000});
  for (double e : {-0.4, -0.01, 0.01, 0.4}) {
    for (double rate : {-0.3, 0.2}) {
      const double s = rate + 5 * e;
      const double s_dot = law.acceleration(e, rate) + 5 * rate;
      check(s * s_dot <= -4 * s * s + 1e-10, "SMC reaching condition");
    }
  }
  double e = 0.4, v = -0.1;
  constexpr double dt = 0.0005;
  for (int i = 0; i < 16000; ++i) {
    v += dt * law.acceleration(e, v);
    e += dt * v;
  }
  check(std::abs(e) < 1e-5 && std::abs(v) < 1e-5, "Second-order SMC convergence");
  const smc::ReachingLaw limited({5, 4, 2, 0.1, 3});
  check(limited.acceleration(10, 10) == -3, "SMC acceleration limit");
  bool rejected = false;
  try { (void)law.acceleration(std::numeric_limits<double>::quiet_NaN(), 0); }
  catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "Reject non-finite SMC state");

  navigation::WaypointNavigator navigator({{-0.2, 0.3}, {0.4, 0.5}});
  double x = 0, y = 0, speed = 0;
  navigation::Command previous;
  for (int i = 0; i < 20000; ++i) {
    const auto command = navigator.update(x, y, speed, 0.002);
    check(std::hypot(command.vx, command.vy) <= 0.120001, "Navigation speed bound");
    check(std::hypot(command.vx - previous.vx, command.vy - previous.vy) <= 0.00030001,
          "Navigation acceleration bound");
    x += 0.002 * command.vx; y += 0.002 * command.vy;
    speed = std::hypot(command.vx, command.vy);
    previous = command;
  }
  check(navigator.complete() && std::hypot(x - 0.4, y - 0.5) < 0.07 && speed < 1e-8, "Waypoint sequencing and stop");
  navigator.reset();
  check(navigator.index() == 0 && !navigator.complete(), "Navigator reset");
  navigation::WaypointNavigator empty({});
  check(empty.complete() && empty.update(0, 0, 0, 0.002).vx == 0, "Empty route holds still");

  locomotion::ForceAllocator allocator;
  const std::array<locomotion::Vec3, 4> arms{{{0.2, 0.14, -0.3}, {0.2, -0.14, -0.3},
                                             {-0.2, 0.14, -0.3}, {-0.2, -0.14, -0.3}}};
  locomotion::Wrench desired; desired << 0, 0, 150, 0, 0, 0;
  const auto balanced = allocator.solve(arms, {{true, true, true, true}}, desired);
  check(balanced.residual < 2, "Static force balance residual");
  for (const auto& force : balanced.forces)
    check(std::abs(force.z() - 37.5) < 0.5 && force.head<2>().norm() < 0.01, "Symmetric load distribution");
  for (int k = 0; k < 60; ++k) {
    desired << 180 * std::sin(k), 200 * std::cos(k), 40 + 300 * std::cos(k * 0.4),
               30 * std::sin(k * 0.7), 20, -30;
    const std::array<bool, 4> stance{{k % 3 != 0, true, k % 2 == 0, true}};
    const auto result = allocator.solve(arms, stance, desired);
    for (int i = 0; i < 4; ++i) {
      const auto& f = result.forces[i];
      check(f.allFinite() && f.z() >= 0 && f.z() <= 160, "Unilateral normal-force limits");
      check(std::abs(f.x()) <= 0.5 * f.z() + 1e-9 && std::abs(f.y()) <= 0.5 * f.z() + 1e-9,
            "Friction pyramid limits");
      if (!stance[i]) check(f.norm() == 0, "Swing leg has no allocated support force");
    }
  }
  const auto airborne = allocator.solve(arms, {{false, false, false, false}}, desired);
  for (const auto& f : airborne.forces) check(f.norm() == 0, "No force with no support");
  std::cout << "PASS: SMC convergence, bounded navigation, constrained force allocation.\n";
}
