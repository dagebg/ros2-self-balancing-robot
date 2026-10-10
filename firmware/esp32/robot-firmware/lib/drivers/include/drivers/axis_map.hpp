#pragma once

/// @file
/// Maps vectors from an IMU's mounting orientation into the robot body frame.

#include <array>
#include <cstddef>

#include "drivers/vector3.hpp"

namespace robot::drivers {

/// Axis permutation with signs, describing how a sensor is mounted.
///
/// Each body axis (x forward, y left, z up; REP-103) is given as a code: 1, 2, 3 select the sensor
/// x, y, z axis, and a negative code reverses it. Example: `{-1, -2, 3}` describes a sensor
/// rotated 180 degrees about z (facing backwards).
class AxisMap {
 public:
  /// Builds a map from three codes. Invalid maps (see isValid()) map every vector to zero.
  constexpr AxisMap(int body_x, int body_y, int body_z) noexcept
      : codes_{body_x, body_y, body_z}, valid_(validate(body_x, body_y, body_z)) {}

  /// True if the codes select each sensor axis exactly once and form a proper rotation
  /// (determinant +1). A mirror image cannot be produced by mounting a sensor.
  [[nodiscard]] constexpr bool isValid() const noexcept { return valid_; }

  /// Transforms a vector from body axes back to sensor axes (the inverse of apply()).
  [[nodiscard]] Vector3 toSensor(const Vector3& body) const noexcept {
    if (!valid_) {
      return {};
    }
    std::array<float, 3> s{};
    const std::array<float, 3> b{body.x, body.y, body.z};
    for (std::size_t i = 0; i < 3; ++i) {
      const int code = codes_[i];
      s[static_cast<std::size_t>(magnitude(code) - 1)] = code < 0 ? -b[i] : b[i];
    }
    return {s[0], s[1], s[2]};
  }

  /// Transforms a vector from sensor axes to body axes.
  [[nodiscard]] Vector3 apply(const Vector3& sensor) const noexcept {
    if (!valid_) {
      return {};
    }
    const std::array<float, 3> s{sensor.x, sensor.y, sensor.z};
    const auto pick = [&s](int code) {
      const float value = s[static_cast<std::size_t>(magnitude(code) - 1)];  // validated: 1..3
      return code < 0 ? -value : value;
    };
    return {pick(codes_[0]), pick(codes_[1]), pick(codes_[2])};
  }

 private:
  static constexpr int magnitude(int value) noexcept { return value < 0 ? -value : value; }

  static constexpr bool validate(int x, int y, int z) noexcept {
    const int ax = magnitude(x);
    const int ay = magnitude(y);
    const int az = magnitude(z);
    const bool in_range = ax >= 1 && ax <= 3 && ay >= 1 && ay <= 3 && az >= 1 && az <= 3;
    if (!in_range || ax == ay || ay == az || ax == az) {
      return false;
    }
    // Determinant of a signed permutation matrix: permutation parity times the product of signs.
    const int inversions = (ax > ay ? 1 : 0) + (ax > az ? 1 : 0) + (ay > az ? 1 : 0);
    const int parity = inversions % 2 == 0 ? 1 : -1;
    const int signs = (x < 0 ? -1 : 1) * (y < 0 ? -1 : 1) * (z < 0 ? -1 : 1);
    return parity * signs == 1;
  }

  std::array<int, 3> codes_;
  bool valid_;
};

}  // namespace robot::drivers
