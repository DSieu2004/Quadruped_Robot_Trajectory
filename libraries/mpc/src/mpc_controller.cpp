#include "mpc/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mpc {
namespace {
void require_finite(double value, const char* name) {
  if (!std::isfinite(value)) throw std::invalid_argument(name);
}

double clamp(double value, double lower, double upper) {
  return std::clamp(value, lower, upper);
}
}  // namespace

MpcController::MpcController(MpcConfig config) : config_(config) {
  if (config_.horizon_steps < 2 || config_.horizon_steps > 100 ||
      config_.max_iterations == 0 || config_.max_iterations > 10000) {
    throw std::invalid_argument("MPC horizon or iteration count is outside supported limits");
  }
  require_finite(config_.position_weight, "MPC position weight must be finite and nonnegative");
  require_finite(config_.velocity_weight, "MPC velocity weight must be finite and nonnegative");
  require_finite(config_.acceleration_weight, "MPC acceleration weight must be finite and positive");
  require_finite(config_.acceleration_change_weight, "MPC acceleration-change weight must be finite and nonnegative");
  require_finite(config_.acceleration_change_limit, "MPC acceleration-change limit must be finite and positive");
  require_finite(config_.acceleration_min, "MPC acceleration minimum must be finite");
  require_finite(config_.acceleration_max, "MPC acceleration maximum must be finite");
  require_finite(config_.convergence_tolerance, "MPC convergence tolerance must be finite and positive");
  if (config_.position_weight < 0 || config_.velocity_weight < 0 ||
      config_.acceleration_weight <= 0 || config_.acceleration_change_weight < 0 ||
      config_.acceleration_change_limit <= 0 ||
      config_.acceleration_min >= config_.acceleration_max ||
      config_.convergence_tolerance <= 0) {
    throw std::invalid_argument("Invalid MPC weights or acceleration bounds");
  }
  warm_start_.assign(config_.horizon_steps, 0.0);
}

double MpcController::update(double position, double velocity,
                             const std::vector<Reference>& refs, double dt) {
  require_finite(position, "MPC position must be finite");
  require_finite(velocity, "MPC velocity must be finite");
  require_finite(dt, "MPC dt must be finite");
  if (dt <= 0.0 || dt > 0.5) throw std::invalid_argument("MPC dt must be in (0, 0.5]");
  if (refs.size() != config_.horizon_steps)
    throw std::invalid_argument("MPC reference horizon length does not match configuration");
  for (const auto& ref : refs) {
    require_finite(ref.position, "MPC reference position must be finite");
    require_finite(ref.velocity, "MPC reference velocity must be finite");
  }

  const std::size_t count = config_.horizon_steps;
  std::vector<double> hessian(count * count, 0.0);
  std::vector<double> gradient_offset(count, 0.0);
  auto h = [&](std::size_t row, std::size_t col) -> double& {
    return hessian[row * count + col];
  };

  // Condense the linear dynamics into a quadratic cost in the acceleration
  // sequence U. For each predicted state, p_i = p_free + Gp*U and
  // v_i = v_free + Gv*U.
  for (std::size_t step = 1; step <= count; ++step) {
    const double time = static_cast<double>(step) * dt;
    const double position_error_free = position + time * velocity - refs[step - 1].position;
    const double velocity_error_free = velocity - refs[step - 1].velocity;
    for (std::size_t row = 0; row < step; ++row) {
      const double gp_row = dt * dt * (static_cast<double>(step - row) - 0.5);
      gradient_offset[row] += 2.0 * config_.position_weight * gp_row * position_error_free;
      gradient_offset[row] += 2.0 * config_.velocity_weight * dt * velocity_error_free;
      for (std::size_t col = 0; col < step; ++col) {
        const double gp_col = dt * dt * (static_cast<double>(step - col) - 0.5);
        h(row, col) += 2.0 * config_.position_weight * gp_row * gp_col;
        h(row, col) += 2.0 * config_.velocity_weight * dt * dt;
      }
    }
  }

  for (std::size_t index = 0; index < count; ++index)
    h(index, index) += 2.0 * config_.acceleration_weight;

  // Penalize acceleration changes to discourage command chatter. The first
  // difference is measured from the previous applied command.
  const double slew_weight = config_.acceleration_change_weight;
  if (slew_weight > 0.0) {
    h(0, 0) += 2.0 * slew_weight;
    gradient_offset[0] -= 2.0 * slew_weight * previous_acceleration_;
    for (std::size_t index = 1; index < count; ++index) {
      h(index, index) += 2.0 * slew_weight;
      h(index - 1, index - 1) += 2.0 * slew_weight;
      h(index, index - 1) -= 2.0 * slew_weight;
      h(index - 1, index) -= 2.0 * slew_weight;
    }
  }

  // A Gershgorin row-sum bound gives a safe Lipschitz step for projected
  // gradient descent on this positive-definite quadratic objective.
  double lipschitz = 0.0;
  for (std::size_t row = 0; row < count; ++row) {
    double row_sum = 0.0;
    for (std::size_t col = 0; col < count; ++col) row_sum += std::abs(h(row, col));
    lipschitz = std::max(lipschitz, row_sum);
  }
  if (!std::isfinite(lipschitz) || lipschitz <= 0.0)
    throw std::runtime_error("MPC Hessian is not positive definite");

  std::vector<double> solution(count);
  for (std::size_t index = 0; index < count; ++index)
    solution[index] = clamp(warm_start_[index], config_.acceleration_min, config_.acceleration_max);
  std::vector<double> extrapolated = solution;
  double momentum = 1.0;
  last_iterations_ = 0;
  for (std::size_t iteration = 0; iteration < config_.max_iterations; ++iteration) {
    std::vector<double> next(count);
    double max_change = 0.0;
    for (std::size_t row = 0; row < count; ++row) {
      double value = gradient_offset[row];
      for (std::size_t col = 0; col < count; ++col)
        value += h(row, col) * extrapolated[col];
      double lower = config_.acceleration_min;
      double upper = config_.acceleration_max;
      const double preceding_acceleration = row == 0 ? previous_acceleration_ : next[row - 1];
      lower = std::max(lower, preceding_acceleration - config_.acceleration_change_limit);
      upper = std::min(upper, preceding_acceleration + config_.acceleration_change_limit);
      next[row] = clamp(extrapolated[row] - value / lipschitz, lower, upper);
      max_change = std::max(max_change, std::abs(next[row] - solution[row]));
    }
    const double next_momentum = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * momentum * momentum));
    const double extrapolation = (momentum - 1.0) / next_momentum;
    for (std::size_t index = 0; index < count; ++index)
      extrapolated[index] = next[index] + extrapolation * (next[index] - solution[index]);
    solution.swap(next);
    momentum = next_momentum;
    last_iterations_ = iteration + 1;
    if (max_change <= config_.convergence_tolerance) break;
  }

  previous_acceleration_ = solution.front();
  for (std::size_t index = 0; index + 1 < count; ++index)
    warm_start_[index] = solution[index + 1];
  warm_start_.back() = solution.back();
  return previous_acceleration_;
}

void MpcController::reset() {
  std::fill(warm_start_.begin(), warm_start_.end(), 0.0);
  previous_acceleration_ = 0.0;
  last_iterations_ = 0;
}

}  // namespace mpc
