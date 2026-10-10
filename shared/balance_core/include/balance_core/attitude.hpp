#pragma once

/// @file
/// Roll and pitch estimation from accelerometer and gyroscope.

#include <cstdint>

#include "balance_core/math.hpp"

namespace robot::balance {

/// Roll and pitch in the REP-103 body frame: positive roll lifts the left side, positive pitch
/// tilts the nose down (forwards).
struct Attitude {
  float roll = 0.0F;        ///< [rad]
  float pitch = 0.0F;       ///< [rad]
  float roll_rate = 0.0F;   ///< [rad/s]
  float pitch_rate = 0.0F;  ///< [rad/s]
};

/// Angles implied by the accelerometer alone, valid when the robot is not accelerating.
[[nodiscard]] Attitude attitudeFromAccel(const Vec3& accel) noexcept;

/// Complementary filter: integrates the gyroscope for fast response and corrects its drift with
/// the accelerometer over a time constant (default 0.5 s).
///
/// The accelerometer also sees the robot's own acceleration, so it is trusted only slowly. When
/// the measured specific force deviates from 1 g by more than a threshold, the accelerometer
/// correction is skipped for that sample.
class ComplementaryFilter {
 public:
  struct Config {
    float time_constant = 0.5F;     ///< [s] accelerometer correction time constant
    float accel_rejection = 0.25F;  ///< reject accel if | |a| / g - 1 | exceeds this
  };

  ComplementaryFilter() noexcept : ComplementaryFilter(Config{}) {}
  explicit ComplementaryFilter(Config config) noexcept : config_(config) {}

  /// @param accel  Body-frame specific force [m/s^2].
  /// @param gyro   Body-frame angular rate, bias removed [rad/s].
  /// @param dt     Time since the previous update [s].
  Attitude update(const Vec3& accel, const Vec3& gyro, float dt) noexcept;

  /// Forgets the estimate; the next update initializes from the accelerometer.
  void reset() noexcept { initialized_ = false; }

  [[nodiscard]] const Attitude& attitude() const noexcept { return attitude_; }

 private:
  Config config_;
  Attitude attitude_{};
  bool initialized_ = false;
};

/// Estimates the gyroscope's zero-rate offset while the robot is standing still.
///
/// Averages a window of samples; if the robot moves during the window (any axis outside the
/// tolerance band around the running mean), the window restarts.
class GyroBiasEstimator {
 public:
  struct Config {
    std::uint32_t samples = 400;     ///< 2 s at 200 Hz
    float motion_threshold = 0.05F;  ///< [rad/s] max deviation from the mean while "still"
  };

  GyroBiasEstimator() noexcept : GyroBiasEstimator(Config{}) {}
  explicit GyroBiasEstimator(Config config) noexcept : config_(config) {}

  /// Feeds one raw gyro sample. Returns true once the bias is known.
  bool update(const Vec3& gyro) noexcept;

  [[nodiscard]] bool ready() const noexcept { return ready_; }
  [[nodiscard]] const Vec3& bias() const noexcept { return bias_; }

  /// Removes the estimated bias (no-op until ready()).
  [[nodiscard]] Vec3 correct(const Vec3& gyro) const noexcept {
    return {gyro.x - bias_.x, gyro.y - bias_.y, gyro.z - bias_.z};
  }

  /// Starts a new calibration, keeping the previous bias until the new one is ready.
  void restart() noexcept {
    count_ = 0;
    sum_ = {};
    ready_ = false;
  }

 private:
  Config config_;
  Vec3 sum_{};
  Vec3 bias_{};
  std::uint32_t count_ = 0;
  bool ready_ = false;
};

}  // namespace robot::balance
