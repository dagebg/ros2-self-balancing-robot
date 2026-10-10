#pragma once

/// @file
/// Cascaded balance and motion controller.

#include <cstdint>

#include "balance_core/filters.hpp"
#include "balance_core/math.hpp"
#include "balance_core/pid.hpp"

namespace robot::balance {

/// Controller loops with tunable gains.
enum class Loop : std::uint8_t { kPitch = 0, kVelocity = 1, kYawRate = 2 };

struct BalanceConfig {
  // Defaults come from a gain search on the WheeledPendulum model: they keep it balanced across
  // body mass 0.65-1.2 kg, centre-of-mass height 5-11 cm and +-33 % motor torque, so they are a
  // safe starting point while the real parameters are unknown. Final tuning happens on the robot
  // (see the docs' tuning guide).
  PidGains pitch_gains{28.0F, 5.0F, 0.1F};     ///< pitch error [rad] -> motor command
  PidGains velocity_gains{0.3F, 0.05F, 0.0F};  ///< velocity error [m/s] -> lean angle [rad]
  PidGains yaw_rate_gains{0.12F, 0.4F, 0.0F};  ///< yaw-rate error [rad/s] -> steering command

  float max_lean = degToRad(8.0F);     ///< [rad] largest lean the velocity loop may request
  float max_velocity = 0.2F;           ///< [m/s] commands are clamped to this
  float max_yaw_rate = 2.0F;           ///< [rad/s]
  float max_acceleration = 0.4F;       ///< [m/s^2] velocity command ramp
  float max_yaw_acceleration = 4.0F;   ///< [rad/s^2] yaw-rate command ramp
  float max_steering = 0.3F;           ///< largest share of the motor command used for turning
  float velocity_filter_time = 0.05F;  ///< [s] low-pass on the measured forward velocity
};

struct BalanceInput {
  float pitch = 0.0F;             ///< [rad], positive = nose down
  float pitch_rate = 0.0F;        ///< [rad/s]
  float yaw_rate = 0.0F;          ///< [rad/s], counter-clockwise positive
  float forward_velocity = 0.0F;  ///< [m/s] over ground (see Odometry)
  float target_velocity = 0.0F;   ///< [m/s] commanded
  float target_yaw_rate = 0.0F;   ///< [rad/s] commanded
  float dt = 0.0F;                ///< [s] since the previous update
};

struct BalanceOutput {
  float left = 0.0F;             ///< Motor command in [-1, 1], positive drives forwards.
  float right = 0.0F;            ///< Motor command in [-1, 1], positive drives forwards.
  float target_pitch = 0.0F;     ///< [rad] lean requested by the velocity loop
  float ramped_velocity = 0.0F;  ///< [m/s] velocity setpoint after ramping
  float ramped_yaw_rate = 0.0F;  ///< [rad/s] yaw-rate setpoint after ramping
};

/// Three PID loops in a cascade:
///
/// ```text
/// v_cmd -> ramp -> [velocity PID] -> target pitch -> [pitch PID] -> u ----+--> left  = u - s
/// w_cmd -> ramp -> [yaw-rate PID] -----------------------------> s ------+--> right = u + s
/// ```
///
/// To move forwards the robot must first lean forwards; the velocity loop therefore commands a
/// lean angle and the inner pitch loop, running on the gyroscope rate for its D term, produces the
/// common motor command. Steering is added differentially and is reduced first when the motors
/// saturate, because balance has priority.
class BalanceController {
 public:
  BalanceController() noexcept : BalanceController(BalanceConfig{}) {}
  explicit BalanceController(const BalanceConfig& config) noexcept;

  BalanceOutput update(const BalanceInput& input) noexcept;

  /// Clears integrators, filters and ramps. Call when balancing (re)starts.
  void reset() noexcept;

  void setGains(Loop loop, const PidGains& gains) noexcept;
  [[nodiscard]] PidGains gains(Loop loop) const noexcept;
  [[nodiscard]] const BalanceConfig& config() const noexcept { return config_; }

 private:
  BalanceConfig config_;
  Pid pitch_pid_;
  Pid velocity_pid_;
  Pid yaw_rate_pid_;
  SlewRateLimiter velocity_ramp_;
  SlewRateLimiter yaw_rate_ramp_;
  LowPassFilter velocity_filter_;
};

}  // namespace robot::balance
