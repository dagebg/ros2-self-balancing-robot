#include "balance_core/filters.hpp"

#include <algorithm>
#include <cmath>

namespace robot::balance {

float LowPassFilter::update(float input, float dt) noexcept {
  if (!std::isfinite(input)) {
    return value_;  // ignore bad samples instead of poisoning the state
  }
  if (!initialized_ || time_constant_ <= 0.0F) {
    value_ = input;
    initialized_ = true;
    return value_;
  }
  const float alpha = dt / (time_constant_ + dt);
  value_ += alpha * (input - value_);
  return value_;
}

float SlewRateLimiter::update(float target, float dt) noexcept {
  if (!std::isfinite(target)) {
    return value_;
  }
  const float max_step = max_rate_ * dt;
  value_ += std::clamp(target - value_, -max_step, max_step);
  return value_;
}

}  // namespace robot::balance
