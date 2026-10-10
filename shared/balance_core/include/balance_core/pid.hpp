#pragma once

/// @file
/// PID controller for the cascaded balance loops.

#include <optional>

#include "balance_core/filters.hpp"

namespace robot::balance {

struct PidGains {
  float kp = 0.0F;
  float ki = 0.0F;
  float kd = 0.0F;
};

struct PidLimits {
  float output_min = -1.0F;
  float output_max = 1.0F;
  float integral_min = -1.0F;  ///< Limits the integral term's contribution (output units).
  float integral_max = 1.0F;
};

/// PID with the safeguards a balancing robot needs:
///
/// - **Derivative on measurement:** the D term acts on the measured value, not the error, so a
///   setpoint step does not kick the output. When the caller has a better rate signal (the
///   gyroscope for pitch), it can pass it instead of numeric differentiation.
/// - **Anti-windup:** the integral term is clamped, and it stops integrating while the output is
///   saturated in the direction the error pushes.
/// - **Robust inputs:** non-finite inputs or dt leave the state untouched and return the last
///   output.
class Pid {
 public:
  Pid(PidGains gains, PidLimits limits, float derivative_filter_time = 0.0F) noexcept
      : gains_(gains), limits_(limits), derivative_filter_(derivative_filter_time) {}

  /// @param measured_rate  d(measurement)/dt if known (e.g. from a gyro); otherwise the rate is
  ///                       differentiated numerically and low-pass filtered.
  float update(float setpoint, float measurement, float dt,
               std::optional<float> measured_rate = std::nullopt) noexcept;

  /// Clears the integral and derivative state, e.g. when balancing starts.
  void reset() noexcept;

  void setGains(const PidGains& gains) noexcept { gains_ = gains; }
  [[nodiscard]] const PidGains& gains() const noexcept { return gains_; }

  [[nodiscard]] float integral() const noexcept { return integral_; }
  [[nodiscard]] float output() const noexcept { return output_; }

 private:
  PidGains gains_;
  PidLimits limits_;
  LowPassFilter derivative_filter_;
  float integral_ = 0.0F;  ///< Already multiplied by ki (output units).
  float previous_measurement_ = 0.0F;
  bool has_previous_ = false;
  float output_ = 0.0F;
};

}  // namespace robot::balance
