#include "smc/joint_smc.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
  const smc::JointSmc controller({10, 5, 0.1, 30});
  auto check = [](bool condition) {
    if (!condition) throw std::runtime_error("SMC control law check failed");
  };
  check(controller.torque(0, 0, 3) == 3);
  check(controller.torque(0.1, 0, 0) == -5);
  check(controller.torque(-0.1, 0, 0) == 5);
  check(std::abs(controller.torque(0, 0.05, 0) + 2.5) < 1e-12);
  check(controller.torque(0, 0, 100) == 30);
  check(controller.torque(0, 0, -100) == -30);
  bool rejected = false;
  try { const smc::JointSmc invalid({10, 5, 0, 30}); }
  catch (const std::invalid_argument&) { rejected = true; }
  check(rejected);
  rejected = false;
  try { (void)controller.torque(std::numeric_limits<double>::quiet_NaN(), 0, 0); }
  catch (const std::invalid_argument&) { rejected = true; }
  check(rejected);
  std::cout << "SMC sign, boundary layer, torque limits and input validation passed.\n";
}
