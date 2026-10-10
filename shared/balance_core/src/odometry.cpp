#include "balance_core/odometry.hpp"

#include <cmath>

#include "balance_core/math.hpp"

namespace robot::balance {

BodyTwist bodyTwist(const WheelGeometry& geometry, float left_rate, float right_rate,
                    float pitch_rate) noexcept {
  const float left_ground = geometry.wheel_radius * (left_rate + pitch_rate);
  const float right_ground = geometry.wheel_radius * (right_rate + pitch_rate);
  return {0.5F * (left_ground + right_ground), (right_ground - left_ground) / geometry.track_width};
}

WheelRates wheelRates(const WheelGeometry& geometry, const BodyTwist& twist) noexcept {
  const float half_track = 0.5F * geometry.track_width;
  return {(twist.linear - twist.angular * half_track) / geometry.wheel_radius,
          (twist.linear + twist.angular * half_track) / geometry.wheel_radius};
}

void Odometry::update(float left_angle, float right_angle, float pitch) noexcept {
  if (!std::isfinite(left_angle) || !std::isfinite(right_angle) || !std::isfinite(pitch)) {
    return;
  }
  if (!initialized_) {
    last_left_ = left_angle;
    last_right_ = right_angle;
    last_pitch_ = pitch;
    initialized_ = true;
    return;
  }

  const float pitch_delta = pitch - last_pitch_;
  const float left = geometry_.wheel_radius * (left_angle - last_left_ + pitch_delta);
  const float right = geometry_.wheel_radius * (right_angle - last_right_ + pitch_delta);
  last_left_ = left_angle;
  last_right_ = right_angle;
  last_pitch_ = pitch;

  const float distance = 0.5F * (left + right);
  const float heading_delta = (right - left) / geometry_.track_width;
  const float mid_heading = pose_.theta + 0.5F * heading_delta;  // midpoint integration
  pose_.x += distance * std::cos(mid_heading);
  pose_.y += distance * std::sin(mid_heading);
  pose_.theta = wrapAngle(pose_.theta + heading_delta);
}

}  // namespace robot::balance
