#include "pid/pid_controller.hpp"

#include <cmath>
#include <iostream>

// Demonstrates PID independently of ROS/MuJoCo on a unit-mass double integrator.
int main() {
  pid::PidGains gains;
  gains.kp = 8.0;
  gains.ki = 1.5;
  gains.kd = 4.0;
  gains.derivative_filter_time_constant = 0.02;
  gains.anti_windup_gain = 8.0;
  gains.output_min = -20.0;
  gains.output_max = 20.0;
  gains.integral_min = -8.0;
  gains.integral_max = 8.0;
  gains.max_dt = 0.01;
  pid::PidController controller(gains);

  constexpr double dt = 0.001;
  constexpr double target = 1.0;
  double position = 0.0;
  double velocity = 0.0;
  for (int step = 0; step < 5000; ++step) {
    const double acceleration = controller.update(target, position, dt);
    velocity += acceleration * dt;
    position += velocity * dt;
    if (step % 500 == 0) {
      const auto& state = controller.state();
      std::cout << "t=" << step * dt << " position=" << position
                << " error=" << state.error << " output=" << state.output
                << " saturated=" << state.saturated << '\n';
    }
  }
  return std::abs(position - target) < 0.03 && std::abs(velocity) < 0.05 ? 0 : 1;
}
