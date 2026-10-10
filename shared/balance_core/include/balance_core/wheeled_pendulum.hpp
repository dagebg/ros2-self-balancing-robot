#pragma once

/// @file
/// Physics model of a two-wheeled inverted pendulum, for closed-loop tests and tuning.

#include "balance_core/math.hpp"

namespace robot::balance::sim {

/// Physical parameters. Defaults mirror the placeholder values in config/robot_params.yaml plus
/// estimates for an N20 100 RPM gear motor driven at its rated voltage.
struct PendulumParams {
  float body_mass = 0.9F;         ///< [kg] without wheels
  float com_height = 0.08F;       ///< [m] centre of mass above the axle
  float body_inertia = 0.0025F;   ///< [kg m^2] about the centre of mass, pitch axis
  float wheel_radius = 0.0325F;   ///< [m]
  float wheel_mass = 0.03F;       ///< [kg] per wheel
  float track_width = 0.15F;      ///< [m]
  float stall_torque = 0.15F;     ///< [N m] per motor at full command
  float no_load_speed = 10.5F;    ///< [rad/s] wheel speed at full command (100 RPM)
  float rolling_damping = 0.05F;  ///< [N s/m] lumped rolling and bearing losses
  float yaw_damping = 0.002F;     ///< [N m s] lumped scrubbing losses when turning
};

/// State in the world frame. Pitch follows REP-103 (positive = nose down).
struct PendulumState {
  float x = 0.0F;                  ///< [m] axle position along the heading
  float velocity = 0.0F;           ///< [m/s]
  float pitch = 0.0F;              ///< [rad]
  float pitch_rate = 0.0F;         ///< [rad/s]
  float yaw = 0.0F;                ///< [rad]
  float yaw_rate = 0.0F;           ///< [rad/s]
  float left_wheel_angle = 0.0F;   ///< [rad] relative to the chassis, as an encoder sees it
  float right_wheel_angle = 0.0F;  ///< [rad] relative to the chassis
};

/// Lagrangian model of the sagittal dynamics with x (axle position) and pitch as coordinates:
///
///     m_eff x''          + M l cos(p) p'' = tau / r + M l sin(p) p'^2 - c x'
///     M l cos(p) x''     + (J + M l^2) p'' = M g l sin(p) - tau
///
/// where tau is the total motor torque (it drives the wheels and reacts on the body) and
/// m_eff = M + 2 m_w + 2 J_w / r^2. Each motor follows the linear DC-motor curve
/// tau = tau_stall (u - w_rel / w_noload), with w_rel the wheel speed relative to the chassis.
/// Yaw is a decoupled rotation driven by the torque difference.
class WheeledPendulum {
 public:
  explicit WheeledPendulum(PendulumParams params = {}, PendulumState initial = {}) noexcept
      : params_(params), state_(initial) {}

  /// Advances the model by dt with motor commands in [-1, 1] (internally in 1 ms substeps).
  void step(float left_command, float right_command, float dt) noexcept;

  /// Instant change of pitch rate, modelling a push against the body.
  void push(float pitch_rate_change) noexcept { state_.pitch_rate += pitch_rate_change; }

  [[nodiscard]] const PendulumState& state() const noexcept { return state_; }
  [[nodiscard]] const PendulumParams& params() const noexcept { return params_; }

  /// Ideal accelerometer at the axle in body axes: specific force [m/s^2].
  [[nodiscard]] Vec3 accelerometer() const noexcept;
  /// Ideal gyroscope in body axes [rad/s].
  [[nodiscard]] Vec3 gyroscope() const noexcept;

 private:
  void substep(float left_command, float right_command, float dt) noexcept;

  PendulumParams params_;
  PendulumState state_;
  float acceleration_ = 0.0F;  ///< last x'' for the accelerometer model
};

}  // namespace robot::balance::sim
