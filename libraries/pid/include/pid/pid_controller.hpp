#pragma once

namespace pid {

// Units follow the controlled signal. For joint control: position in rad,
// time in seconds, output in N.m. Integral limits apply to the I contribution.
struct PidGains {
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.0;
  double derivative_filter_time_constant = 0.02;
  double anti_windup_gain = 5.0;
  double output_min = -30.0;
  double output_max = 30.0;
  double integral_min = -30.0;
  double integral_max = 30.0;
  double max_dt = 0.25;
};

struct PidState {
  double error = 0.0;
  double proportional = 0.0;
  double integral = 0.0;
  double derivative = 0.0;  // Filtered measurement rate; D output uses its negative.
  double feedforward = 0.0;
  double unsaturated_output = 0.0;
  double output = 0.0;
  bool saturated = false;
  bool initialized = false;
};

// Discrete positional PID. Derivative is taken on measurement to avoid
// setpoint kick. Includes derivative filtering, saturation and back-calculation
// anti-windup. Use one instance per independently controlled axis.
class PidController {
 public:
  explicit PidController(PidGains gains = {});

  [[nodiscard]] double update(double setpoint, double measurement, double dt,
                              double feedforward = 0.0);
  void reset(double integral_output = 0.0);

  // Initializes history and chooses the integral term to match a desired output
  // at this operating point, when the configured limits permit it.
  void reset(double setpoint, double measurement, double desired_output,
             double feedforward = 0.0);

  [[nodiscard]] const PidState& state() const { return state_; }
  [[nodiscard]] const PidGains& gains() const { return gains_; }

 private:
  PidGains gains_;
  PidState state_;
  double previous_error_ = 0.0;
  double previous_measurement_ = 0.0;
};

}  // namespace pid
