#pragma once

/// @file
/// Differential-drive kinematics and odometry for a balancing robot.

namespace robot::balance {

struct WheelGeometry {
  float wheel_radius = 0.0F;  ///< [m]
  float track_width = 0.0F;   ///< [m]
};

/// Planar pose in the odometry frame.
struct Pose2D {
  float x = 0.0F;      ///< [m]
  float y = 0.0F;      ///< [m]
  float theta = 0.0F;  ///< [rad], wrapped to (-pi, pi]
};

/// Body velocity over ground.
struct BodyTwist {
  float linear = 0.0F;   ///< [m/s], forwards positive
  float angular = 0.0F;  ///< [rad/s], counter-clockwise positive
};

/// Body velocity from wheel rates.
///
/// The encoders sit on the motor shafts, which are fixed to the chassis, so they measure the
/// wheel's rotation *relative to the chassis*. When the chassis pitches, the wheels turn relative
/// to it even if the robot does not move. The rotation over ground is the relative rotation plus
/// the pitch rate (both are rotations about the body y axis).
///
/// @param left_rate, right_rate  Wheel rates relative to the chassis [rad/s], forwards positive.
/// @param pitch_rate             Body pitch rate [rad/s].
[[nodiscard]] BodyTwist bodyTwist(const WheelGeometry& geometry, float left_rate, float right_rate,
                                  float pitch_rate) noexcept;

/// Wheel rates relative to the chassis [rad/s] that produce a body twist at zero pitch rate.
struct WheelRates {
  float left = 0.0F;
  float right = 0.0F;
};
[[nodiscard]] WheelRates wheelRates(const WheelGeometry& geometry, const BodyTwist& twist) noexcept;

/// Integrates wheel angles into a pose, compensating for body pitch.
class Odometry {
 public:
  explicit Odometry(WheelGeometry geometry) noexcept : geometry_(geometry) {}

  /// @param left_angle, right_angle  Cumulative wheel angles relative to the chassis [rad].
  /// @param pitch                    Body pitch at the same instant [rad].
  /// The first call only initializes the reference.
  void update(float left_angle, float right_angle, float pitch) noexcept;

  void reset(const Pose2D& pose = {}) noexcept {
    pose_ = pose;
    initialized_ = false;
  }

  [[nodiscard]] const Pose2D& pose() const noexcept { return pose_; }

 private:
  WheelGeometry geometry_;
  Pose2D pose_{};
  float last_left_ = 0.0F;
  float last_right_ = 0.0F;
  float last_pitch_ = 0.0F;
  bool initialized_ = false;
};

}  // namespace robot::balance
