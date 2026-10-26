#include "smc/reaching_law.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace smc {
ReachingLaw::ReachingLaw(ReachingGains gains) : gains_(gains) {
  if (!std::isfinite(gains.lambda) || gains.lambda <= 0 ||
      !std::isfinite(gains.linear_gain) || gains.linear_gain < 0 ||
      !std::isfinite(gains.switching_gain) || gains.switching_gain < 0 ||
      !std::isfinite(gains.boundary_layer) || gains.boundary_layer <= 0 ||
      !std::isfinite(gains.acceleration_limit) || gains.acceleration_limit <= 0)
    throw std::invalid_argument("Invalid SMC reaching-law gains");
}

double ReachingLaw::acceleration(double error, double rate, double feedforward) const {
  if (!std::isfinite(error) || !std::isfinite(rate) || !std::isfinite(feedforward))
    throw std::invalid_argument("Non-finite SMC state");
  const double surface = rate + gains_.lambda * error;
  const double value = feedforward - gains_.lambda * rate - gains_.linear_gain * surface
      - gains_.switching_gain * std::clamp(surface / gains_.boundary_layer, -1.0, 1.0);
  if (!std::isfinite(value)) throw std::overflow_error("SMC arithmetic overflow");
  return std::clamp(value, -gains_.acceleration_limit, gains_.acceleration_limit);
}
}  // namespace smc
