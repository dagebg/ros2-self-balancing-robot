#include "balance_core/wheeled_pendulum.hpp"

#include <algorithm>
#include <cmath>

namespace robot::balance::sim {

namespace {
constexpr float kMaxSubstep = 0.001F;  // [s]
}  // namespace

void WheeledPendulum::step(float left_command, float right_command, float dt) noexcept {
  if (!(dt > 0.0F)) {
    return;
  }
  const int substeps = std::max(1, static_cast<int>(std::ceil(dt / kMaxSubstep)));
  const float h = dt / static_cast<float>(substeps);
  for (int i = 0; i < substeps; ++i) {
    substep(left_command, right_command, h);
  }
}

void WheeledPendulum::substep(float left_command, float right_command, float dt) noexcept {
  const PendulumParams& p = params_;
  PendulumState& s = state_;
  const float half_track = 0.5F * p.track_width;

  // Motor torques from the DC-motor curve, using wheel speed relative to the chassis.
  const float left_ground_rate = (s.velocity - s.yaw_rate * half_track) / p.wheel_radius;
  const float right_ground_rate = (s.velocity + s.yaw_rate * half_track) / p.wheel_radius;
  const float left_relative = left_ground_rate - s.pitch_rate;
  const float right_relative = right_ground_rate - s.pitch_rate;
  const auto torque = [&p](float command, float relative_rate) {
    const float u = clampFinite(command, -1.0F, 1.0F);
    return p.stall_torque * (u - relative_rate / p.no_load_speed);
  };
  const float left_torque = torque(left_command, left_relative);
  const float right_torque = torque(right_command, right_relative);
  const float total_torque = left_torque + right_torque;

  // Sagittal dynamics: solve the 2x2 mass matrix system for x'' and pitch''.
  const float m = p.body_mass;
  const float l = p.com_height;
  const float r = p.wheel_radius;
  const float wheel_inertia = 0.5F * p.wheel_mass * r * r;
  const float m_eff = m + 2.0F * p.wheel_mass + 2.0F * wheel_inertia / (r * r);
  const float sin_p = std::sin(s.pitch);
  const float cos_p = std::cos(s.pitch);

  const float a11 = m_eff;
  const float a12 = m * l * cos_p;
  const float a22 = p.body_inertia + m * l * l;
  const float b1 = total_torque / r + m * l * sin_p * s.pitch_rate * s.pitch_rate -
                   p.rolling_damping * s.velocity;
  const float b2 = m * kGravity * l * sin_p - total_torque;
  const float det = a11 * a22 - a12 * a12;
  const float x_acc = (b1 * a22 - a12 * b2) / det;
  const float pitch_acc = (a11 * b2 - a12 * b1) / det;

  // Yaw: torque difference over the half track acts as a force couple at the contact points.
  const float yaw_inertia =
      m * p.track_width * p.track_width / 12.0F + 2.0F * p.wheel_mass * half_track * half_track;
  const float yaw_torque =
      (right_torque - left_torque) * half_track / r - p.yaw_damping * s.yaw_rate;
  const float yaw_acc = yaw_torque / yaw_inertia;

  // Semi-implicit Euler: velocities first, then positions with the new velocities.
  s.velocity += x_acc * dt;
  s.pitch_rate += pitch_acc * dt;
  s.yaw_rate += yaw_acc * dt;
  s.x += s.velocity * dt;
  s.pitch += s.pitch_rate * dt;
  s.yaw = wrapAngle(s.yaw + s.yaw_rate * dt);

  const float left_rel_new = (s.velocity - s.yaw_rate * half_track) / r - s.pitch_rate;
  const float right_rel_new = (s.velocity + s.yaw_rate * half_track) / r - s.pitch_rate;
  s.left_wheel_angle += left_rel_new * dt;
  s.right_wheel_angle += right_rel_new * dt;
  acceleration_ = x_acc;
}

Vec3 WheeledPendulum::accelerometer() const noexcept {
  // Specific force (acceleration minus gravity) rotated into body axes; pitch rotates about y.
  const float sin_p = std::sin(state_.pitch);
  const float cos_p = std::cos(state_.pitch);
  return {acceleration_ * cos_p - kGravity * sin_p, 0.0F, acceleration_ * sin_p + kGravity * cos_p};
}

Vec3 WheeledPendulum::gyroscope() const noexcept {
  return {0.0F, state_.pitch_rate, state_.yaw_rate};
}

}  // namespace robot::balance::sim
