#pragma once

/// @file
/// Driver for the Toshiba TB6612FNG dual H-bridge.

#include "hal/hal.hpp"

namespace robot::drivers {

/// One H-bridge channel (IN1, IN2, PWM) driving one motor.
///
/// Truth table (datasheet): IN1=H/IN2=L forward, IN1=L/IN2=H reverse, both high = short brake,
/// both low = coast. The PWM input modulates the drive phase.
class HBridgeChannel {
 public:
  /// @param inverted  Swap directions, for a motor mounted mirrored or wired reversed.
  /// @param max_duty  Largest PWM duty ever applied, in (0, 1]. Limits the effective motor
  ///                  voltage, e.g. rated voltage / supply voltage.
  HBridgeChannel(hal::PwmOutput& pwm, hal::DigitalOutput& in1, hal::DigitalOutput& in2,
                 bool inverted, float max_duty) noexcept;

  /// Drives the motor. `command` in [-1, 1] is scaled by max_duty; values outside are clamped and
  /// NaN coasts. Zero coasts.
  void set(float command) noexcept;

  /// Short brake: both motor terminals tied together.
  void brake() noexcept;

  /// Releases the motor (high impedance).
  void coast() noexcept;

  /// The last command after clamping, in [-1, 1] (before max_duty scaling and inversion).
  [[nodiscard]] float command() const noexcept { return command_; }

  /// The duty cycle actually applied to the PWM pin, in [0, max_duty].
  [[nodiscard]] float appliedDuty() const noexcept { return applied_duty_; }

 private:
  hal::PwmOutput& pwm_;
  hal::DigitalOutput& in1_;
  hal::DigitalOutput& in2_;
  bool inverted_;
  float max_duty_;
  float command_ = 0.0F;
  float applied_duty_ = 0.0F;
};

/// The chip's shared standby input. While disabled, both channels are off regardless of their
/// inputs; this is the motors' master switch.
class Tb6612 {
 public:
  explicit Tb6612(hal::DigitalOutput& standby) noexcept : standby_(standby) {}

  void enable() noexcept;
  void disable() noexcept;
  [[nodiscard]] bool enabled() const noexcept { return enabled_; }

 private:
  hal::DigitalOutput& standby_;
  bool enabled_ = false;
};

}  // namespace robot::drivers
