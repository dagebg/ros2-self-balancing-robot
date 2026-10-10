#include "balance_core/imu_fusion.hpp"

#include <cmath>

namespace robot::balance {

namespace {

Vec3 average(const Vec3& a, const Vec3& b) noexcept {
  return {0.5F * (a.x + b.x), 0.5F * (a.y + b.y), 0.5F * (a.z + b.z)};
}

bool agree(const Vec3& a, const Vec3& b, float tolerance) noexcept {
  return std::fabs(a.x - b.x) <= tolerance && std::fabs(a.y - b.y) <= tolerance &&
         std::fabs(a.z - b.z) <= tolerance;
}

}  // namespace

std::optional<ImuSampleBody> ImuFusion::update(const std::optional<ImuSampleBody>& primary,
                                               const std::optional<ImuSampleBody>& secondary,
                                               float dt) noexcept {
  if (!primary || !secondary) {
    disagreement_duration_ = 0.0F;
    disagreement_ = false;
    return primary ? primary : secondary;
  }

  if (agree(primary->gyro, secondary->gyro, config_.gyro_tolerance)) {
    disagreement_duration_ = 0.0F;
    disagreement_ = false;
  } else {
    disagreement_duration_ += dt;
    disagreement_ = disagreement_duration_ >= config_.disagreement_time;
  }

  if (disagreement_) {
    return primary;
  }
  return ImuSampleBody{average(primary->accel, secondary->accel),
                       average(primary->gyro, secondary->gyro)};
}

}  // namespace robot::balance
