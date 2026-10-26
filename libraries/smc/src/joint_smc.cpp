#include "smc/joint_smc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace smc {

JointSmc::JointSmc(JointGains gains) : gains_(gains) {
  if (!std::isfinite(gains.lambda) || gains.lambda <= 0 ||
      !std::isfinite(gains.switching_gain) || gains.switching_gain < 0 ||
      !std::isfinite(gains.boundary_layer) || gains.boundary_layer <= 0 ||
      !std::isfinite(gains.torque_limit) || gains.torque_limit <= 0) {
    throw std::invalid_argument("SMC gains must be finite and positive (switching_gain may be zero)");
  }
}

double JointSmc::torque(double position_error, double velocity_error,
                       double equivalent_torque) const {
  if (!std::isfinite(position_error) || !std::isfinite(velocity_error) ||
      !std::isfinite(equivalent_torque)) {
    throw std::invalid_argument("SMC input must be finite");
  }
  const double surface = velocity_error + gains_.lambda * position_error;
  const double switching = std::clamp(surface / gains_.boundary_layer, -1.0, 1.0);
  return std::clamp(equivalent_torque - gains_.switching_gain * switching,
                    -gains_.torque_limit, gains_.torque_limit);
}

}  // namespace smc
