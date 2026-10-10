#include "balance_core/safety.hpp"

#include <cmath>

namespace robot::balance {

bool SafetyStateMachine::requestArm() noexcept {
  switch (state_) {
    case SafetyState::kDisarmed:
      if (!imu_ok_) {
        return false;
      }
      state_ = SafetyState::kArmed;
      upright_time_ = 0.0F;
      return true;
    case SafetyState::kArmed:
    case SafetyState::kBalancing:
      return true;
    case SafetyState::kFault:
      return false;
  }
  return false;
}

void SafetyStateMachine::requestDisarm() noexcept {
  state_ = SafetyState::kDisarmed;
  faults_ = 0;
  upright_time_ = 0.0F;
}

void SafetyStateMachine::enterFault(std::uint16_t fault) noexcept {
  faults_ |= fault;
  state_ = SafetyState::kFault;
}

SafetyUpdate SafetyStateMachine::update(float pitch, bool imu_ok, float dt) noexcept {
  imu_ok_ = imu_ok && std::isfinite(pitch);
  SafetyUpdate result;

  switch (state_) {
    case SafetyState::kDisarmed:
    case SafetyState::kFault:
      break;

    case SafetyState::kArmed:
      if (!imu_ok_) {
        enterFault(fault::kImuFailure);
        break;
      }
      upright_time_ = std::fabs(pitch) < config_.start_tilt ? upright_time_ + dt : 0.0F;
      if (upright_time_ >= config_.start_hold_time) {
        state_ = SafetyState::kBalancing;
        result.started_balancing = true;
      }
      break;

    case SafetyState::kBalancing:
      if (!imu_ok_) {
        enterFault(fault::kImuFailure);
      } else if (std::fabs(pitch) > config_.fall_tilt) {
        enterFault(fault::kTiltLimit);
      }
      break;
  }

  result.state = state_;
  return result;
}

}  // namespace robot::balance
