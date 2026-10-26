#include "pid/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pid {
namespace {
void require_finite(double value, const char* name) {
  if (!std::isfinite(value)) throw std::invalid_argument(name);
}

double clamp(double value, double lower, double upper) {
  return std::clamp(value, lower, upper);
}
}  // namespace

PidController::PidController(PidGains gains) : gains_(gains) {
  require_finite(gains.kp, "PID kp must be finite");
  require_finite(gains.ki, "PID ki must be finite");
  require_finite(gains.kd, "PID kd must be finite");
  require_finite(gains.derivative_filter_time_constant, "PID filter time constant must be finite");
  require_finite(gains.anti_windup_gain, "PID anti-windup gain must be finite");
  require_finite(gains.output_min, "PID output_min must be finite");
  require_finite(gains.output_max, "PID output_max must be finite");
  require_finite(gains.integral_min, "PID integral_min must be finite");
  require_finite(gains.integral_max, "PID integral_max must be finite");
  require_finite(gains.max_dt, "PID max_dt must be finite");
  if (gains.kp < 0 || gains.ki < 0 || gains.kd < 0 ||
      gains.derivative_filter_time_constant < 0 || gains.anti_windup_gain < 0 ||
      gains.output_min >= gains.output_max || gains.integral_min > gains.integral_max ||
      gains.max_dt <= 0) {
    throw std::invalid_argument("Invalid PID gains or limits");
  }
}

double PidController::update(double setpoint, double measurement, double dt,
                             double feedforward) {
  require_finite(setpoint, "PID setpoint must be finite");
  require_finite(measurement, "PID measurement must be finite");
  require_finite(dt, "PID dt must be finite");
  require_finite(feedforward, "PID feedforward must be finite");
  if (dt <= 0 || dt > gains_.max_dt) {
    throw std::invalid_argument("PID dt must be in (0, max_dt]");
  }

  const double error = setpoint - measurement;
  require_finite(error, "PID error overflow");
  const double raw_measurement_rate = state_.initialized
      ? (measurement - previous_measurement_) / dt : 0.0;
  require_finite(raw_measurement_rate, "PID measurement derivative overflow");
  const double tau = gains_.derivative_filter_time_constant;
  const double alpha = tau > 0 ? std::exp(-dt / tau) : 0.0;
  const double derivative = alpha * state_.derivative + (1.0 - alpha) * raw_measurement_rate;
  require_finite(derivative, "PID filtered derivative overflow");

  const double old_error = state_.initialized ? previous_error_ : error;
  double integral = state_.integral + gains_.ki * 0.5 * (old_error + error) * dt;
  require_finite(integral, "PID integral overflow");
  integral = clamp(integral, gains_.integral_min, gains_.integral_max);
  const double proportional = gains_.kp * error;
  require_finite(proportional, "PID proportional term overflow");
  double unsaturated = feedforward + proportional + integral - gains_.kd * derivative;
  require_finite(unsaturated, "PID output overflow");
  double output = clamp(unsaturated, gains_.output_min, gains_.output_max);

  // Exact discrete correction for a constant saturation mismatch. It remains
  // bounded for a variable dt unlike a forward-Euler back-calculation step.
  if (gains_.anti_windup_gain > 0) {
    const double fraction = -std::expm1(-gains_.anti_windup_gain * dt);
    integral = clamp(integral + (output - unsaturated) * fraction,
                     gains_.integral_min, gains_.integral_max);
    require_finite(integral, "PID anti-windup state overflow");
    unsaturated = feedforward + proportional + integral - gains_.kd * derivative;
    require_finite(unsaturated, "PID output overflow");
    output = clamp(unsaturated, gains_.output_min, gains_.output_max);
  }

  state_.error = error;
  state_.proportional = proportional;
  state_.integral = integral;
  state_.derivative = derivative;
  state_.feedforward = feedforward;
  state_.unsaturated_output = unsaturated;
  state_.output = output;
  state_.saturated = output != unsaturated;
  state_.initialized = true;
  previous_error_ = error;
  previous_measurement_ = measurement;
  return output;
}

void PidController::reset(double integral_output) {
  require_finite(integral_output, "PID reset output must be finite");
  state_ = {};
  state_.integral = clamp(integral_output, gains_.integral_min, gains_.integral_max);
  state_.output = clamp(state_.integral, gains_.output_min, gains_.output_max);
  state_.unsaturated_output = state_.output;
  state_.saturated = state_.output != state_.integral;
  previous_error_ = 0.0;
  previous_measurement_ = 0.0;
}

void PidController::reset(double setpoint, double measurement, double desired_output,
                          double feedforward) {
  require_finite(setpoint, "PID reset setpoint must be finite");
  require_finite(measurement, "PID reset measurement must be finite");
  require_finite(desired_output, "PID reset output must be finite");
  require_finite(feedforward, "PID reset feedforward must be finite");

  state_ = {};
  state_.error = setpoint - measurement;
  require_finite(state_.error, "PID reset error overflow");
  state_.proportional = gains_.kp * state_.error;
  require_finite(state_.proportional, "PID reset proportional term overflow");
  state_.integral = clamp(desired_output - feedforward - state_.proportional,
                          gains_.integral_min, gains_.integral_max);
  state_.feedforward = feedforward;
  state_.unsaturated_output = feedforward + state_.proportional + state_.integral;
  state_.output = clamp(state_.unsaturated_output, gains_.output_min, gains_.output_max);
  state_.saturated = state_.output != state_.unsaturated_output;
  state_.initialized = true;
  previous_error_ = state_.error;
  previous_measurement_ = measurement;
}

}  // namespace pid
