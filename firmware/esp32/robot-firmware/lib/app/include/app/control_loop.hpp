#pragma once

/// @file
/// The firmware's control tick: sensors in, motor commands and telemetry out.

#include <cstdint>
#include <optional>

#include "balance_core/attitude.hpp"
#include "balance_core/balance_controller.hpp"
#include "balance_core/imu_fusion.hpp"
#include "balance_core/odometry.hpp"
#include "balance_core/safety.hpp"
#include "drivers/axis_map.hpp"
#include "drivers/mpu6xxx.hpp"
#include "robot_protocol/messages.hpp"

namespace robot::app {

/// Everything the control loop needs to know about the robot's build.
struct ControlConfig {
  drivers::AxisMap primary_mounting{1, 2, 3};    ///< MPU-9250
  drivers::AxisMap secondary_mounting{1, 2, 3};  ///< MPU-6050
  balance::WheelGeometry geometry{};
  float counts_per_wheel_rev = 1.0F;  ///< encoder counts, x4 decoding
  bool left_encoder_inverted = false;
  bool right_encoder_inverted = false;
  float nominal_period = 0.005F;  ///< [s] expected time between ticks
  balance::BalanceConfig balance{};
  balance::SafetyConfig safety{};
};

/// One snapshot of the sensors, taken at the start of a tick.
struct SensorFrame {
  std::uint32_t time_us = 0;
  std::optional<drivers::ImuReading> primary;    ///< sensor axes
  std::optional<drivers::ImuReading> secondary;  ///< sensor axes
  std::int32_t left_ticks = 0;                   ///< raw cumulative encoder counts
  std::int32_t right_ticks = 0;                  ///< raw cumulative encoder counts
};

/// Commands from the Pi as seen by the control task.
struct MotionCommand {
  float linear = 0.0F;   ///< [m/s]
  float angular = 0.0F;  ///< [rad/s]
  bool link_alive = false;
};

/// Result of one tick.
struct ControlOutput {
  float left = 0.0F;   ///< motor command in [-1, 1]
  float right = 0.0F;  ///< motor command in [-1, 1]
  bool motors_enabled = false;
  protocol::Telemetry telemetry{};
  std::optional<protocol::ImuSample> primary_sample;    ///< calibrated, body axes
  std::optional<protocol::ImuSample> secondary_sample;  ///< calibrated, body axes
  bool state_changed = false;  ///< telemetry.state differs from the previous tick
};

/// Owns the estimation, control and safety pipeline. Pure logic, no hardware access: the
/// firmware feeds it sensor frames, and unit tests feed it frames from a plant model.
///
/// Per tick: rotate both IMUs into the body frame, remove each gyro's bias (calibrated while
/// disarmed and still), fuse, estimate attitude, update safety, compute wheel rates and ground
/// velocity, and run the controller while balancing.
class ControlLoop {
 public:
  explicit ControlLoop(const ControlConfig& config) noexcept;

  ControlOutput step(const SensorFrame& frame, const MotionCommand& command) noexcept;

  /// Handles SetArmed and SetGains; other messages are answered with kInvalid.
  protocol::Ack handle(const protocol::Message& message, std::uint8_t sequence) noexcept;

  /// Reports the duration of the previous tick (for telemetry and overrun detection).
  void reportTickDuration(std::uint32_t duration_us) noexcept;

  [[nodiscard]] balance::SafetyState state() const noexcept { return safety_.state(); }
  [[nodiscard]] bool calibrated() const noexcept;
  [[nodiscard]] const balance::Attitude& attitude() const noexcept { return filter_.attitude(); }

 private:
  [[nodiscard]] float wheelAngle(std::int32_t ticks, bool inverted) const noexcept;

  ControlConfig config_;
  balance::GyroBiasEstimator primary_bias_;
  balance::GyroBiasEstimator secondary_bias_;
  balance::ImuFusion fusion_;
  balance::ComplementaryFilter filter_;
  balance::SafetyStateMachine safety_;
  balance::BalanceController controller_;

  std::uint32_t last_time_us_ = 0;
  bool has_last_time_ = false;
  float last_left_angle_ = 0.0F;
  float last_right_angle_ = 0.0F;
  std::uint16_t max_tick_us_ = 0;
  bool overrun_ = false;
  balance::SafetyState last_state_ = balance::SafetyState::kDisarmed;
};

}  // namespace robot::app
