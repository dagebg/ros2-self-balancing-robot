#include "balance_core/balance_controller.hpp"

#include <algorithm>
#include <cmath>

namespace robot::balance {

namespace {
PidLimits symmetric(float output_limit) noexcept {
  return {-output_limit, output_limit, -output_limit, output_limit};
}
}  // namespace

BalanceController::BalanceController(const BalanceConfig& config) noexcept
    : config_(config),
      pitch_pid_(config.pitch_gains, symmetric(1.0F)),
      velocity_pid_(config.velocity_gains, symmetric(config.max_lean)),
      yaw_rate_pid_(config.yaw_rate_gains, symmetric(config.max_steering)),
      velocity_ramp_(config.max_acceleration),
      yaw_rate_ramp_(config.max_yaw_acceleration),
      velocity_filter_(config.velocity_filter_time) {}

BalanceOutput BalanceController::update(const BalanceInput& input) noexcept {
  BalanceOutput output;
  const float dt = input.dt;

  const float velocity_cmd =
      clampFinite(input.target_velocity, -config_.max_velocity, config_.max_velocity);
  const float yaw_rate_cmd =
      clampFinite(input.target_yaw_rate, -config_.max_yaw_rate, config_.max_yaw_rate);
  output.ramped_velocity = velocity_ramp_.update(velocity_cmd, dt);
  output.ramped_yaw_rate = yaw_rate_ramp_.update(yaw_rate_cmd, dt);

  // Outer loop: velocity error -> lean angle. Leaning forwards makes the robot accelerate.
  const float velocity = velocity_filter_.update(input.forward_velocity, dt);
  output.target_pitch = velocity_pid_.update(output.ramped_velocity, velocity, dt);

  // Inner loop: drive the wheels under the centre of mass. The PID's output has the sign of
  // (target - pitch); tilting forwards beyond the target needs a forward (positive) command.
  const float common = -pitch_pid_.update(output.target_pitch, input.pitch, dt, input.pitch_rate);

  // Steering: counter-clockwise yaw needs the right wheel faster than the left.
  float steering = yaw_rate_pid_.update(output.ramped_yaw_rate, input.yaw_rate, dt);
  const float headroom = std::max(0.0F, 1.0F - std::fabs(common));
  steering = std::clamp(steering, -headroom, headroom);

  output.left = std::clamp(common - steering, -1.0F, 1.0F);
  output.right = std::clamp(common + steering, -1.0F, 1.0F);
  return output;
}

void BalanceController::reset() noexcept {
  pitch_pid_.reset();
  velocity_pid_.reset();
  yaw_rate_pid_.reset();
  velocity_ramp_.reset();
  yaw_rate_ramp_.reset();
  velocity_filter_.reset();
}

void BalanceController::setGains(Loop loop, const PidGains& gains) noexcept {
  switch (loop) {
    case Loop::kPitch:
      pitch_pid_.setGains(gains);
      break;
    case Loop::kVelocity:
      velocity_pid_.setGains(gains);
      break;
    case Loop::kYawRate:
      yaw_rate_pid_.setGains(gains);
      break;
  }
}

PidGains BalanceController::gains(Loop loop) const noexcept {
  switch (loop) {
    case Loop::kPitch:
      return pitch_pid_.gains();
    case Loop::kVelocity:
      return velocity_pid_.gains();
    case Loop::kYawRate:
      return yaw_rate_pid_.gains();
  }
  return {};
}

}  // namespace robot::balance
