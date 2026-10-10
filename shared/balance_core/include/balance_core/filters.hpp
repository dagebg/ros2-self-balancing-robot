#pragma once

/// @file
/// First-order low-pass filter and slew-rate limiter.

namespace robot::balance {

/// First-order IIR low-pass filter, y += alpha * (x - y), with alpha derived from a time constant
/// and the actual sample period, so it behaves the same at any loop rate.
class LowPassFilter {
 public:
  /// @param time_constant  [s]; zero disables filtering.
  explicit LowPassFilter(float time_constant = 0.0F) noexcept : time_constant_(time_constant) {}

  float update(float input, float dt) noexcept;
  void reset(float value = 0.0F) noexcept {
    value_ = value;
    initialized_ = false;
  }
  [[nodiscard]] float value() const noexcept { return value_; }

 private:
  float time_constant_;
  float value_ = 0.0F;
  bool initialized_ = false;
};

/// Limits how fast a value may change, e.g. to turn a velocity step into an acceleration ramp.
class SlewRateLimiter {
 public:
  /// @param max_rate  Largest change per second (units/s); must be positive.
  explicit SlewRateLimiter(float max_rate) noexcept : max_rate_(max_rate) {}

  float update(float target, float dt) noexcept;
  void reset(float value = 0.0F) noexcept { value_ = value; }
  [[nodiscard]] float value() const noexcept { return value_; }

 private:
  float max_rate_;
  float value_ = 0.0F;
};

}  // namespace robot::balance
