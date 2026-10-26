#pragma once

namespace smc {

struct JointGains {
  double lambda = 10.0;
  double switching_gain = 5.0;
  double boundary_layer = 0.1;
  double torque_limit = 30.0;
};

// Joint-space SMC building block. All angles are radians; torques are N.m.
// The caller supplies equivalent_torque from the robot's dynamic model.
// This class alone does not implement balance or locomotion.
class JointSmc {
 public:
  explicit JointSmc(JointGains gains = {});
  [[nodiscard]] double torque(double position_error, double velocity_error,
                              double equivalent_torque) const;

 private:
  JointGains gains_;
};

}  // namespace smc
