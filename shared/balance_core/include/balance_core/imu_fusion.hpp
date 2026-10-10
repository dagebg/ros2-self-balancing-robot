#pragma once

/// @file
/// Combines the two IMUs into one body-frame measurement.

#include <cstdint>
#include <optional>

#include "balance_core/math.hpp"

namespace robot::balance {

/// One IMU sample, already rotated into the body frame and bias-corrected.
struct ImuSampleBody {
  Vec3 accel;  ///< [m/s^2]
  Vec3 gyro;   ///< [rad/s]
};

/// Fuses a primary and a secondary IMU.
///
/// Both sensors sit on the same rigid body, so their angular rates must agree; their
/// accelerations may differ slightly because they are mounted at different heights. The fused
/// sample averages both while they agree. If the gyroscopes disagree for longer than a debounce
/// time, the secondary is distrusted, the primary is used alone and disagreement() is reported.
/// If one sensor is missing, the other is used; if both are missing, there is no sample.
class ImuFusion {
 public:
  struct Config {
    float gyro_tolerance = 0.35F;     ///< [rad/s] max difference per axis
    float disagreement_time = 0.05F;  ///< [s] debounce before flagging
  };

  ImuFusion() noexcept : ImuFusion(Config{}) {}
  explicit ImuFusion(Config config) noexcept : config_(config) {}

  [[nodiscard]] std::optional<ImuSampleBody> update(const std::optional<ImuSampleBody>& primary,
                                                    const std::optional<ImuSampleBody>& secondary,
                                                    float dt) noexcept;

  /// True while the gyroscopes disagree beyond the debounce time.
  [[nodiscard]] bool disagreement() const noexcept { return disagreement_; }

 private:
  Config config_;
  float disagreement_duration_ = 0.0F;
  bool disagreement_ = false;
};

}  // namespace robot::balance
