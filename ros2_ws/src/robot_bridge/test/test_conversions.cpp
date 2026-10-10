#include <gtest/gtest.h>

#include <cmath>

#include "robot_bridge/conversions.hpp"

namespace {

namespace rp = robot::protocol;
using robot_bridge::OdometryTracker;

constexpr double kTolerance = 1e-9;

TEST(ConversionsTest, QuaternionFromRpyMatchesKnownRotations) {
  const auto identity = robot_bridge::quaternionFromRpy(0.0, 0.0, 0.0);
  EXPECT_NEAR(identity.w, 1.0, kTolerance);

  const auto yaw90 = robot_bridge::quaternionFromRpy(0.0, 0.0, M_PI / 2.0);
  EXPECT_NEAR(yaw90.w, std::sqrt(0.5), kTolerance);
  EXPECT_NEAR(yaw90.z, std::sqrt(0.5), kTolerance);

  const auto pitch = robot_bridge::quaternionFromRpy(0.0, 0.2, 0.0);
  EXPECT_NEAR(pitch.y, std::sin(0.1), kTolerance);
  EXPECT_NEAR(pitch.x, 0.0, kTolerance);
}

TEST(ConversionsTest, QuaternionIsNormalized) {
  const auto q = robot_bridge::quaternionFromRpy(0.3, -0.7, 2.1);
  EXPECT_NEAR(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z, 1.0, kTolerance);
}

TEST(ConversionsTest, BalanceStateCopiesTelemetry) {
  rp::Telemetry telemetry;
  telemetry.state = rp::RobotState::kBalancing;
  telemetry.faults = rp::fault::kLinkTimeout;
  telemetry.pitch = 0.05F;
  telemetry.left_duty = -0.3F;
  telemetry.max_loop_time_us = 812;
  const auto state = robot_bridge::toBalanceState(telemetry, rclcpp::Time(5, 0), "base_link");
  EXPECT_EQ(state.state, robot_interfaces::msg::BalanceState::STATE_BALANCING);
  EXPECT_EQ(state.faults, robot_interfaces::msg::BalanceState::FAULT_LINK_TIMEOUT);
  EXPECT_FLOAT_EQ(state.pitch, 0.05F);
  EXPECT_FLOAT_EQ(state.left_command, -0.3F);
  EXPECT_EQ(state.max_loop_time_us, 812);
  EXPECT_EQ(state.header.frame_id, "base_link");
}

TEST(ConversionsTest, FaultFlagsMatchTheProtocol) {
  using robot_interfaces::msg::BalanceState;
  EXPECT_EQ(BalanceState::FAULT_TILT_LIMIT, rp::fault::kTiltLimit);
  EXPECT_EQ(BalanceState::FAULT_IMU_FAILURE, rp::fault::kImuFailure);
  EXPECT_EQ(BalanceState::FAULT_LINK_TIMEOUT, rp::fault::kLinkTimeout);
  EXPECT_EQ(BalanceState::FAULT_IMU_DISAGREEMENT, rp::fault::kImuDisagreement);
  EXPECT_EQ(BalanceState::FAULT_LOOP_OVERRUN, rp::fault::kLoopOverrun);
}

TEST(ConversionsTest, ImuWithoutOrientationIsMarkedUnknown) {
  rp::ImuSample sample;
  sample.gyro = {0.1F, 0.2F, 0.3F};
  sample.accel = {0.0F, 0.0F, 9.81F};
  const auto imu = robot_bridge::toImu(sample, false, 0.0, 0.0, 0.0, rclcpp::Time(0, 0), "imu");
  EXPECT_EQ(imu.orientation_covariance[0], -1.0);
  EXPECT_FLOAT_EQ(static_cast<float>(imu.angular_velocity.y), 0.2F);
  EXPECT_FLOAT_EQ(static_cast<float>(imu.linear_acceleration.z), 9.81F);
}

TEST(OdometryTrackerTest, IntegratesWheelTicksIntoPose) {
  // One wheel revolution = 4200 counts; radius 0.05 m.
  OdometryTracker tracker({0.05F, 0.2F}, 4200.0);
  rp::Telemetry telemetry;
  tracker.update(telemetry);
  telemetry.left_ticks = 4200;
  telemetry.right_ticks = 4200;
  tracker.update(telemetry);
  EXPECT_NEAR(tracker.pose().x, 2.0 * M_PI * 0.05, 1e-4);
  EXPECT_NEAR(tracker.leftWheelAngle(), 2.0 * M_PI, 1e-9);
}

TEST(OdometryTrackerTest, VelocityIncludesPitchRate) {
  OdometryTracker tracker({0.05F, 0.2F}, 4200.0);
  rp::Telemetry telemetry;
  telemetry.left_wheel_velocity = -1.0F;  // wheels turn back relative to the leaning chassis
  telemetry.right_wheel_velocity = -1.0F;
  telemetry.pitch_rate = 1.0F;
  tracker.update(telemetry);
  EXPECT_NEAR(tracker.twist().linear, 0.0, 1e-6);
}

TEST(OdometryTrackerTest, IntegratesGyroYawAcrossTimestampWrap) {
  OdometryTracker tracker({0.05F, 0.2F}, 4200.0);
  rp::Telemetry telemetry;
  telemetry.yaw_rate = 1.0F;
  telemetry.timestamp_us = 0xFFFFFFFFU - 9999U;  // 10 ms before the counter wraps
  tracker.update(telemetry);
  telemetry.timestamp_us = 10000;  // 20 ms later, after the wrap
  tracker.update(telemetry);
  EXPECT_NEAR(tracker.gyroYaw(), 0.02, 1e-6);
}

TEST(OdometryTrackerTest, OdometryMessageCarriesPoseAndTwist) {
  OdometryTracker tracker({0.05F, 0.2F}, 4200.0);
  rp::Telemetry telemetry;
  tracker.update(telemetry);
  telemetry.left_ticks = 2100;
  telemetry.right_ticks = 2100;
  telemetry.left_wheel_velocity = 2.0F;
  telemetry.right_wheel_velocity = 2.0F;
  tracker.update(telemetry);
  const auto odom = robot_bridge::toOdometry(tracker, rclcpp::Time(1, 0), "odom", "base_link");
  EXPECT_EQ(odom.header.frame_id, "odom");
  EXPECT_EQ(odom.child_frame_id, "base_link");
  EXPECT_NEAR(odom.pose.pose.position.x, M_PI * 0.05, 1e-4);
  EXPECT_NEAR(odom.twist.twist.linear.x, 0.1, 1e-6);
  EXPECT_GT(odom.pose.covariance[14], 100.0);  // z unobserved
}

}  // namespace
