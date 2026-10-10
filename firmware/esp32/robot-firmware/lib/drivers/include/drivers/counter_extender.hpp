#pragma once

/// @file
/// Extends a short hardware pulse counter to a 32-bit running total.

#include <cstdint>

namespace robot::drivers {

/// Turns the ESP32 PCNT's 16-bit count into a cumulative 32-bit count.
///
/// The PCNT does not wrap like an integer: when the count reaches its limit (+limit or -limit)
/// it resets to zero. Seen from outside, the raw value is the true count modulo `limit`.
/// As long as update() is called before the count changes by limit/2, the true change since the
/// previous call is the raw difference folded into (-limit/2, limit/2].
class CounterExtender {
 public:
  /// @param limit  The PCNT's symmetric limit, e.g. 30000 for h_lim = 30000, l_lim = -30000.
  explicit constexpr CounterExtender(std::int32_t limit) noexcept : limit_(limit) {}

  /// Feeds the latest raw hardware count and returns the cumulative total.
  std::int32_t update(std::int16_t raw) noexcept {
    std::int32_t delta = static_cast<std::int32_t>(raw) - last_raw_;
    if (delta > limit_ / 2) {
      delta -= limit_;
    } else if (delta <= -limit_ / 2) {
      delta += limit_;
    }
    last_raw_ = raw;
    total_ += delta;
    return total_;
  }

  [[nodiscard]] std::int32_t total() const noexcept { return total_; }

  /// Restarts from zero, treating `raw` as the current hardware count.
  void reset(std::int16_t raw) noexcept {
    last_raw_ = raw;
    total_ = 0;
  }

 private:
  std::int32_t limit_;
  std::int32_t last_raw_ = 0;
  std::int32_t total_ = 0;
};

}  // namespace robot::drivers
