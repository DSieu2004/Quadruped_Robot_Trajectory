#pragma once

#include <cstddef>
#include <vector>

namespace mpc {

// One reference sample for a scalar double-integrator channel.
struct Reference {
  double position = 0.0;
  double velocity = 0.0;
};

struct MpcConfig {
  std::size_t horizon_steps = 15;
  double position_weight = 20.0;
  double velocity_weight = 3.0;
  double acceleration_weight = 0.5;
  double acceleration_change_weight = 0.2;
  // Maximum absolute change in acceleration between adjacent MPC samples.
  double acceleration_change_limit = 0.04;
  double acceleration_min = -0.15;
  double acceleration_max = 0.15;
  std::size_t max_iterations = 100;
  double convergence_tolerance = 1e-6;
};

// Receding-horizon MPC for a bounded discrete double integrator:
//   p[k+1] = p[k] + dt*v[k] + 0.5*dt^2*a[k]
//   v[k+1] = v[k] + dt*a[k]
// The controller minimizes a quadratic tracking/input/slew cost over the
// supplied future references, then returns only the first acceleration.
class MpcController {
 public:
  explicit MpcController(MpcConfig config = {});

  // refs must contain exactly config().horizon_steps future samples, beginning
  // one dt after the current state. Returns a bounded acceleration command.
  [[nodiscard]] double update(double position, double velocity,
                              const std::vector<Reference>& refs, double dt);
  void reset();

  [[nodiscard]] const MpcConfig& config() const { return config_; }
  [[nodiscard]] double last_acceleration() const { return previous_acceleration_; }
  [[nodiscard]] std::size_t last_iterations() const { return last_iterations_; }

 private:
  MpcConfig config_;
  std::vector<double> warm_start_;
  double previous_acceleration_ = 0.0;
  std::size_t last_iterations_ = 0;
};

}  // namespace mpc
