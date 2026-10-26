#pragma once

namespace smc {

struct ReachingGains {
  double lambda = 5.0;
  double linear_gain = 5.0;
  double switching_gain = 1.0;
  double boundary_layer = 0.1;
  double acceleration_limit = 10.0;
};

// Second-order channel: e = position - reference, s = e_dot + lambda*e.
// Returns a_ref - lambda*e_dot - linear_gain*s - switching_gain*sat(s/phi).
// The caller supplies the dynamics/contact mapping from acceleration to torque.
class ReachingLaw {
 public:
  explicit ReachingLaw(ReachingGains gains = {});
  [[nodiscard]] double acceleration(double error, double error_rate,
                                     double reference_acceleration = 0.0) const;
 private:
  ReachingGains gains_;
};

}  // namespace smc
