#include "robot_bridge/conversions.hpp"

#include <array>
#include <cmath>

namespace robot_bridge {

namespace {
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kAngleVariance = 0.0003;    // ~1 deg standard deviation, roll and pitch
constexpr double kGyroVariance = 0.0001;     // [rad^2/s^2]
constexpr double kAccelVariance = 0.01;      // [m^2/s^4]
constexpr double kUnknownYawVariance = 1e6;  // integrated gyro yaw drifts without bound

/// 6x6 covariance (x, y, z, roll, pitch, yaw) with the given diagonal.
std::array<double, 36> diagonal(const std::array<double, 6>& values) {
  std::array<double, 36> matrix{};
  for (std::size_t i = 0; i < values.size(); ++i) {
    matrix[i * 7] = values[i];
  }
  return matrix;
}
}  // namespace

geometry_msgs::msg::Quaternion quaternionFromRpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll / 2.0);
  const double sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0);
  const double sp = std::sin(pitch / 2.0);
  const double cy = std::cos(yaw / 2.0);
  const double sy = std::sin(yaw / 2.0);
  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

robot_interfaces::msg::BalanceState toBalanceState(const robot::protocol::Telemetry& telemetry,
                                                   const rclcpp::Time& stamp,
                                                   const std::string& frame_id) {
  robot_interfaces::msg::BalanceState state;
  state.header.stamp = stamp;
  state.header.frame_id = frame_id;
  state.state = static_cast<std::uint8_t>(telemetry.state);
  state.faults = telemetry.faults;
  state.roll = telemetry.roll;
  state.pitch = telemetry.pitch;
  state.pitch_rate = telemetry.pitch_rate;
  state.yaw_rate = telemetry.yaw_rate;
  state.left_command = telemetry.left_duty;
  state.right_command = telemetry.right_duty;
  state.max_loop_time_us = telemetry.max_loop_time_us;
  return state;
}

sensor_msgs::msg::Imu toImu(const robot::protocol::ImuSample& sample, bool has_orientation,
                            double roll, double pitch, double yaw, const rclcpp::Time& stamp,
                            const std::string& frame_id) {
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = frame_id;
  if (has_orientation) {
    imu.orientation = quaternionFromRpy(roll, pitch, yaw);
    imu.orientation_covariance = {kAngleVariance,     0.0, 0.0, 0.0, kAngleVariance, 0.0, 0.0, 0.0,
                                  kUnknownYawVariance};
  } else {
    imu.orientation_covariance[0] = -1.0;
  }
  imu.angular_velocity.x = sample.gyro[0];
  imu.angular_velocity.y = sample.gyro[1];
  imu.angular_velocity.z = sample.gyro[2];
  imu.linear_acceleration.x = sample.accel[0];
  imu.linear_acceleration.y = sample.accel[1];
  imu.linear_acceleration.z = sample.accel[2];
  for (std::size_t i = 0; i < 3; ++i) {
    imu.angular_velocity_covariance[i * 4] = kGyroVariance;
    imu.linear_acceleration_covariance[i * 4] = kAccelVariance;
  }
  return imu;
}

OdometryTracker::OdometryTracker(robot::balance::WheelGeometry geometry,
                                 double counts_per_wheel_rev) noexcept
    : geometry_(geometry), counts_per_wheel_rev_(counts_per_wheel_rev), odometry_(geometry) {}

void OdometryTracker::update(const robot::protocol::Telemetry& telemetry) noexcept {
  left_angle_ = static_cast<double>(telemetry.left_ticks) * kTwoPi / counts_per_wheel_rev_;
  right_angle_ = static_cast<double>(telemetry.right_ticks) * kTwoPi / counts_per_wheel_rev_;
  odometry_.update(static_cast<float>(left_angle_), static_cast<float>(right_angle_),
                   telemetry.pitch);
  twist_ = robot::balance::bodyTwist(geometry_, telemetry.left_wheel_velocity,
                                     telemetry.right_wheel_velocity, telemetry.pitch_rate);

  if (has_last_) {
    // Unsigned subtraction handles the ESP32's 32-bit microsecond counter wrapping.
    const double dt = static_cast<double>(telemetry.timestamp_us - last_timestamp_us_) * 1e-6;
    if (dt > 0.0 && dt < 1.0) {
      gyro_yaw_ = std::remainder(gyro_yaw_ + telemetry.yaw_rate * dt, kTwoPi);
    }
  }
  last_timestamp_us_ = telemetry.timestamp_us;
  has_last_ = true;
}

void OdometryTracker::reset() noexcept {
  odometry_.reset();
  twist_ = {};
  gyro_yaw_ = 0.0;
  has_last_ = false;
}

nav_msgs::msg::Odometry toOdometry(const OdometryTracker& tracker, const rclcpp::Time& stamp,
                                   const std::string& odom_frame, const std::string& base_frame) {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_frame;
  odom.pose.pose.position.x = tracker.pose().x;
  odom.pose.pose.position.y = tracker.pose().y;
  odom.pose.pose.orientation = quaternionFromRpy(0.0, 0.0, tracker.pose().theta);
  odom.twist.twist.linear.x = tracker.twist().linear;
  odom.twist.twist.angular.z = tracker.twist().angular;
  // Wheel odometry: good along x and in yaw, unobserved in the other axes.
  constexpr double kUnobserved = 1e3;
  odom.pose.covariance = diagonal({0.01, 0.01, kUnobserved, kUnobserved, kUnobserved, 0.03});
  odom.twist.covariance = diagonal({0.001, 0.001, kUnobserved, kUnobserved, kUnobserved, 0.01});
  return odom;
}

}  // namespace robot_bridge
