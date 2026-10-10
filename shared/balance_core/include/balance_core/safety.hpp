#pragma once

/// @file
/// Arming and fault handling.

#include <cstdint>

#include "balance_core/math.hpp"

namespace robot::balance {

enum class SafetyState : std::uint8_t {
  kDisarmed = 0,   ///< Motors off.
  kArmed = 1,      ///< Waiting for the robot to be held upright.
  kBalancing = 2,  ///< Controller drives the motors.
  kFault = 3,      ///< Motors off until disarmed.
};

/// Fault bits. The values match robot::protocol::fault so they can be sent unchanged.
namespace fault {
inline constexpr std::uint16_t kTiltLimit = 1U << 0U;
inline constexpr std::uint16_t kImuFailure = 1U << 1U;
}  // namespace fault

struct SafetyConfig {
  float start_tilt = degToRad(3.0F);  ///< |pitch| below which balancing may start [rad]
  float start_hold_time = 0.5F;       ///< how long it must stay there [s]
  float fall_tilt = degToRad(45.0F);  ///< |pitch| that counts as fallen [rad]
};

struct SafetyUpdate {
  SafetyState state = SafetyState::kDisarmed;
  bool started_balancing = false;  ///< True on the tick balancing begins: reset the controller.
};

/// Decides when the motors may run.
///
/// ```text
/// Disarmed --arm--> Armed --upright for 0.5 s--> Balancing
///    ^                 |                             |
///    |               fault                tilt > 45 deg or IMU failure
///    |                 v                             v
///    +-----disarm---- Fault <------------------------+
/// ```
///
/// Disarming is always possible and clears faults. Arming is rejected while a fault is latched or
/// the IMU is not delivering data.
class SafetyStateMachine {
 public:
  SafetyStateMachine() noexcept : SafetyStateMachine(SafetyConfig{}) {}
  explicit SafetyStateMachine(SafetyConfig config) noexcept : config_(config) {}

  /// @return true if the request was accepted (also when already armed or balancing).
  bool requestArm() noexcept;
  void requestDisarm() noexcept;

  /// Advances the state machine by one control tick.
  /// @param pitch   Estimated pitch [rad]; non-finite counts as an IMU failure.
  /// @param imu_ok  False if this tick's IMU data is missing.
  SafetyUpdate update(float pitch, bool imu_ok, float dt) noexcept;

  [[nodiscard]] SafetyState state() const noexcept { return state_; }
  [[nodiscard]] bool motorsEnabled() const noexcept { return state_ == SafetyState::kBalancing; }
  [[nodiscard]] std::uint16_t faults() const noexcept { return faults_; }

 private:
  void enterFault(std::uint16_t fault) noexcept;

  SafetyConfig config_;
  SafetyState state_ = SafetyState::kDisarmed;
  std::uint16_t faults_ = 0;
  float upright_time_ = 0.0F;
  bool imu_ok_ = false;
};

}  // namespace robot::balance
