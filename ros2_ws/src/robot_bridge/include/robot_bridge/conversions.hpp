#pragma once

/// @file
/// Conversions between protocol messages and ROS 2 messages.

#include <cstdint>
#include <string>

#include "balance_core/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"
#include "robot_interfaces/msg/balance_state.hpp"
#include "robot_protocol/messages.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace robot_bridge {

/// Quaternion from roll, pitch, yaw (extrinsic x-y-z, REP-103).
[[nodiscard]] geometry_msgs::msg::Quaternion quaternionFromRpy(double roll, double pitch,
                                                               double yaw);

[[nodiscard]] robot_interfaces::msg::BalanceState toBalanceState(
    const robot::protocol::Telemetry& telemetry, const rclcpp::Time& stamp,
    const std::string& frame_id);

/// IMU message from a calibrated body-frame sample. Orientation is filled from the ESP32's
/// estimate when `has_orientation` is set; otherwise orientation_covariance[0] = -1 marks it
/// unknown, as sensor_msgs/Imu specifies.
[[nodiscard]] sensor_msgs::msg::Imu toImu(const robot::protocol::ImuSample& sample,
                                          bool has_orientation, double roll, double pitch,
                                          double yaw, const rclcpp::Time& stamp,
                                          const std::string& frame_id);

/// Tracks pose and velocity from telemetry, using balance_core::Odometry (the same code that
/// runs in the firmware tests), and integrates the gyro's yaw rate for the IMU orientation.
class OdometryTracker {
 public:
  OdometryTracker(robot::balance::WheelGeometry geometry, double counts_per_wheel_rev) noexcept;

  void update(const robot::protocol::Telemetry& telemetry) noexcept;
  void reset() noexcept;

  [[nodiscard]] const robot::balance::Pose2D& pose() const noexcept { return odometry_.pose(); }
  [[nodiscard]] const robot::balance::BodyTwist& twist() const noexcept { return twist_; }
  [[nodiscard]] double leftWheelAngle() const noexcept { return left_angle_; }
  [[nodiscard]] double rightWheelAngle() const noexcept { return right_angle_; }
  [[nodiscard]] double gyroYaw() const noexcept { return gyro_yaw_; }

 private:
  robot::balance::WheelGeometry geometry_;
  double counts_per_wheel_rev_;
  robot::balance::Odometry odometry_;
  robot::balance::BodyTwist twist_{};
  double left_angle_ = 0.0;
  double right_angle_ = 0.0;
  double gyro_yaw_ = 0.0;
  std::uint32_t last_timestamp_us_ = 0;
  bool has_last_ = false;
};

[[nodiscard]] nav_msgs::msg::Odometry toOdometry(const OdometryTracker& tracker,
                                                 const rclcpp::Time& stamp,
                                                 const std::string& odom_frame,
                                                 const std::string& base_frame);

}  // namespace robot_bridge
