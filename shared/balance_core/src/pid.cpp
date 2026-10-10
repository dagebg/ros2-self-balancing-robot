#include "balance_core/pid.hpp"

#include <algorithm>
#include <cmath>

namespace robot::balance {

float Pid::update(float setpoint, float measurement, float dt,
                  std::optional<float> measured_rate) noexcept {
  const bool rate_ok = !measured_rate || std::isfinite(*measured_rate);
  if (!std::isfinite(setpoint) || !std::isfinite(measurement) || !std::isfinite(dt) || dt <= 0.0F ||
      !rate_ok) {
    return output_;
  }

  const float error = setpoint - measurement;

  float rate = 0.0F;
  if (measured_rate) {
    rate = *measured_rate;
  } else if (has_previous_) {
    rate = derivative_filter_.update((measurement - previous_measurement_) / dt, dt);
  }
  previous_measurement_ = measurement;
  has_previous_ = true;

  const float proportional = gains_.kp * error;
  const float derivative = -gains_.kd * rate;

  // Conditional integration: skip the step if the output is already saturated and integrating
  // would push it further into saturation.
  const float candidate_integral =
      std::clamp(integral_ + gains_.ki * error * dt, limits_.integral_min, limits_.integral_max);
  const float unclamped = proportional + candidate_integral + derivative;
  const bool saturating_high = unclamped > limits_.output_max && error > 0.0F;
  const bool saturating_low = unclamped < limits_.output_min && error < 0.0F;
  if (!saturating_high && !saturating_low) {
    integral_ = candidate_integral;
  }

  output_ =
      std::clamp(proportional + integral_ + derivative, limits_.output_min, limits_.output_max);
  return output_;
}

void Pid::reset() noexcept {
  integral_ = 0.0F;
  has_previous_ = false;
  derivative_filter_.reset();
  output_ = 0.0F;
}

}  // namespace robot::balance
