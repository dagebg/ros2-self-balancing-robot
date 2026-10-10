#include "balance_core/attitude.hpp"

#include <cmath>

namespace robot::balance {

Attitude attitudeFromAccel(const Vec3& accel) noexcept {
  // Specific force at rest points "up". With the body pitched nose-down by theta, it reads
  // (-g sin theta, ., g cos theta); rolled left-side-up by phi, it reads (., g sin phi, g cos phi).
  Attitude attitude;
  attitude.pitch = std::atan2(-accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z));
  attitude.roll = std::atan2(accel.y, accel.z);
  return attitude;
}

Attitude ComplementaryFilter::update(const Vec3& accel, const Vec3& gyro, float dt) noexcept {
  const bool finite = std::isfinite(accel.x) && std::isfinite(accel.y) && std::isfinite(accel.z) &&
                      std::isfinite(gyro.x) && std::isfinite(gyro.y) && std::isfinite(dt) &&
                      dt > 0.0F;
  if (!finite) {
    return attitude_;
  }

  const Attitude measured = attitudeFromAccel(accel);
  attitude_.roll_rate = gyro.x;
  attitude_.pitch_rate = gyro.y;
  if (!initialized_) {
    attitude_.roll = measured.roll;
    attitude_.pitch = measured.pitch;
    initialized_ = true;
    return attitude_;
  }

  // Predict with the gyroscope.
  attitude_.roll += gyro.x * dt;
  attitude_.pitch += gyro.y * dt;

  // Correct towards the accelerometer when it plausibly measures gravity only.
  const float magnitude = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
  if (std::fabs(magnitude / kGravity - 1.0F) <= config_.accel_rejection) {
    const float alpha = dt / (config_.time_constant + dt);
    attitude_.roll += alpha * (measured.roll - attitude_.roll);
    attitude_.pitch += alpha * (measured.pitch - attitude_.pitch);
  }
  return attitude_;
}

bool GyroBiasEstimator::update(const Vec3& gyro) noexcept {
  if (!std::isfinite(gyro.x) || !std::isfinite(gyro.y) || !std::isfinite(gyro.z)) {
    restart();
    return ready_;
  }
  if (count_ > 0) {
    const auto n = static_cast<float>(count_);
    const Vec3 mean{sum_.x / n, sum_.y / n, sum_.z / n};
    const float threshold = config_.motion_threshold;
    if (std::fabs(gyro.x - mean.x) > threshold || std::fabs(gyro.y - mean.y) > threshold ||
        std::fabs(gyro.z - mean.z) > threshold) {
      count_ = 0;  // moved: start the window again
      sum_ = {};
    }
  }
  sum_.x += gyro.x;
  sum_.y += gyro.y;
  sum_.z += gyro.z;
  ++count_;
  if (count_ >= config_.samples) {
    const auto n = static_cast<float>(count_);
    bias_ = {sum_.x / n, sum_.y / n, sum_.z / n};
    ready_ = true;
    count_ = 0;
    sum_ = {};
  }
  return ready_;
}

}  // namespace robot::balance
