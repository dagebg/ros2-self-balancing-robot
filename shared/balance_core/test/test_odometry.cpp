#include <gtest/gtest.h>

#include <limits>

#include "balance_core/math.hpp"
#include "balance_core/odometry.hpp"

namespace {

namespace rb = robot::balance;
using rb::kPi;

const rb::WheelGeometry kGeometry{0.05F, 0.2F};

TEST(OdometryTest, DrivesStraight) {
  rb::Odometry odometry(kGeometry);
  odometry.update(0.0F, 0.0F, 0.0F);
  odometry.update(2.0F * kPi, 2.0F * kPi, 0.0F);  // one revolution each
  EXPECT_NEAR(odometry.pose().x, 2.0F * kPi * 0.05F, 1e-5F);
  EXPECT_NEAR(odometry.pose().y, 0.0F, 1e-6F);
  EXPECT_NEAR(odometry.pose().theta, 0.0F, 1e-6F);
}

TEST(OdometryTest, SpinsInPlace) {
  rb::Odometry odometry(kGeometry);
  odometry.update(0.0F, 0.0F, 0.0F);
  // Quarter turn: each wheel travels (pi/2) * track/2 in opposite directions.
  const float wheel_angle = (kPi / 2.0F) * 0.1F / 0.05F;
  for (int i = 1; i <= 100; ++i) {
    const float a = wheel_angle * static_cast<float>(i) / 100.0F;
    odometry.update(-a, a, 0.0F);
  }
  EXPECT_NEAR(odometry.pose().theta, kPi / 2.0F, 1e-4F);
  EXPECT_NEAR(odometry.pose().x, 0.0F, 1e-5F);
  EXPECT_NEAR(odometry.pose().y, 0.0F, 1e-5F);
}

TEST(OdometryTest, DrivesQuarterCircle) {
  rb::Odometry odometry(kGeometry);
  odometry.update(0.0F, 0.0F, 0.0F);
  // Arc of radius 0.5 m: outer wheel 0.6 m, inner 0.4 m radius, quarter circle.
  const float outer = (kPi / 2.0F) * 0.6F / 0.05F;
  const float inner = (kPi / 2.0F) * 0.4F / 0.05F;
  for (int i = 1; i <= 200; ++i) {
    const float f = static_cast<float>(i) / 200.0F;
    odometry.update(inner * f, outer * f, 0.0F);
  }
  EXPECT_NEAR(odometry.pose().x, 0.5F, 1e-3F);
  EXPECT_NEAR(odometry.pose().y, 0.5F, 1e-3F);
  EXPECT_NEAR(odometry.pose().theta, kPi / 2.0F, 1e-4F);
}

TEST(OdometryTest, PitchingInPlaceDoesNotMoveTheRobot) {
  // Leaning forwards by 0.1 rad with the wheels on the spot turns the wheels -0.1 rad relative
  // to the chassis. Without compensation this would read as 5 mm of backwards travel.
  rb::Odometry odometry(kGeometry);
  odometry.update(0.0F, 0.0F, 0.0F);
  odometry.update(-0.1F, -0.1F, 0.1F);
  EXPECT_NEAR(odometry.pose().x, 0.0F, 1e-6F);
}

TEST(OdometryTest, IgnoresNonFiniteInput) {
  rb::Odometry odometry(kGeometry);
  odometry.update(0.0F, 0.0F, 0.0F);
  odometry.update(std::numeric_limits<float>::quiet_NaN(), 1.0F, 0.0F);
  odometry.update(1.0F, 1.0F, 0.0F);
  EXPECT_NEAR(odometry.pose().x, 0.05F, 1e-6F);
}

TEST(OdometryTest, ResetSetsPoseAndReinitializes) {
  rb::Odometry odometry(kGeometry);
  odometry.update(0.0F, 0.0F, 0.0F);
  odometry.update(5.0F, 5.0F, 0.0F);
  odometry.reset({1.0F, 2.0F, 0.5F});
  odometry.update(100.0F, 100.0F, 0.0F);  // first update after reset only re-references
  EXPECT_FLOAT_EQ(odometry.pose().x, 1.0F);
  EXPECT_FLOAT_EQ(odometry.pose().theta, 0.5F);
}

TEST(KinematicsTest, BodyTwistCompensatesPitchRate) {
  // Wheels turn backwards relative to the chassis exactly as fast as the body pitches forwards:
  // over ground, nothing moves.
  const auto still = rb::bodyTwist(kGeometry, -1.0F, -1.0F, 1.0F);
  EXPECT_NEAR(still.linear, 0.0F, 1e-6F);

  const auto forward = rb::bodyTwist(kGeometry, 10.0F, 10.0F, 0.0F);
  EXPECT_NEAR(forward.linear, 0.5F, 1e-6F);
  EXPECT_NEAR(forward.angular, 0.0F, 1e-6F);
}

TEST(KinematicsTest, WheelRatesInvertBodyTwist) {
  const rb::BodyTwist twist{0.3F, -1.2F};
  const auto rates = rb::wheelRates(kGeometry, twist);
  const auto back = rb::bodyTwist(kGeometry, rates.left, rates.right, 0.0F);
  EXPECT_NEAR(back.linear, twist.linear, 1e-5F);
  EXPECT_NEAR(back.angular, twist.angular, 1e-5F);
  EXPECT_GT(rates.left, rates.right);  // clockwise turn: left wheel faster
}

}  // namespace
