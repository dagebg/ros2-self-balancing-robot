#pragma once

/// @file
/// Small math helpers shared by the balance core.

#include <algorithm>
#include <cmath>

namespace robot::balance {

inline constexpr float kPi = 3.14159265358979F;
inline constexpr float kGravity = 9.80665F;  ///< [m/s^2]

[[nodiscard]] constexpr float degToRad(float degrees) noexcept { return degrees * kPi / 180.0F; }
[[nodiscard]] constexpr float radToDeg(float radians) noexcept { return radians * 180.0F / kPi; }

/// Body-frame vector (x forward, y left, z up; REP-103).
struct Vec3 {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

/// Clamps to [low, high]; NaN maps to 0 so a bad sensor value can never saturate an actuator.
[[nodiscard]] inline float clampFinite(float value, float low, float high) noexcept {
  return std::isfinite(value) ? std::clamp(value, low, high) : 0.0F;
}

/// Wraps an angle to (-pi, pi].
[[nodiscard]] inline float wrapAngle(float angle) noexcept {
  const float wrapped = std::remainder(angle, 2.0F * kPi);
  return wrapped <= -kPi ? wrapped + 2.0F * kPi : wrapped;
}

}  // namespace robot::balance
