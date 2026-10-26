#include "smc/reaching_law.hpp"

#include <cmath>
#include <iostream>

// A unit-mass double integrator, NOT a Go2 model. Demonstrates the library API.
int main() {
  smc::ReachingLaw controller({5.0, 4.0, 2.0, 0.1, 20.0});
  constexpr double dt = 0.001;
  double position = 0.4, velocity = 0;
  for (int i = 0; i < 6000; ++i) {
    const double acceleration = controller.acceleration(position, velocity, 0);
    velocity += dt * acceleration;
    position += dt * velocity;
    if (i % 1000 == 0) std::cout << "t=" << i * dt << " e=" << position << " de=" << velocity << '\n';
  }
  return std::abs(position) < 1e-5 && std::abs(velocity) < 1e-5 ? 0 : 1;
}
