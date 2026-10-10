#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "balance_core/attitude.hpp"

namespace {

namespace rb = robot::balance;
using rb::degToRad;
using rb::kGravity;
using rb::Vec3;

/// Specific force of a robot at rest with the given pitch (nose down positive) and roll.
Vec3 restingAccel(float pitch, float roll = 0.0F) {
  return {-kGravity * std::sin(pitch), kGravity * std::cos(pitch) * std::sin(roll),
          kGravity * std::cos(pitch) * std::cos(roll)};
}

TEST(AttitudeTest, AccelerometerGivesPitchWithRep103Sign) {
  const auto forward = rb::attitudeFromAccel(restingAccel(degToRad(10.0F)));
  EXPECT_NEAR(forward.pitch, degToRad(10.0F), 1e-5F);
  const auto backward = rb::attitudeFromAccel(restingAccel(degToRad(-20.0F)));
  EXPECT_NEAR(backward.pitch, degToRad(-20.0F), 1e-5F);
}

TEST(AttitudeTest, AccelerometerGivesRoll) {
  const auto rolled = rb::attitudeFromAccel(restingAccel(0.0F, degToRad(15.0F)));
  EXPECT_NEAR(rolled.roll, degToRad(15.0F), 1e-5F);
  EXPECT_NEAR(rolled.pitch, 0.0F, 1e-5F);
}

TEST(ComplementaryFilterTest, InitializesFromAccelerometer) {
  rb::ComplementaryFilter filter;
  const auto attitude = filter.update(restingAccel(degToRad(5.0F)), {}, 0.005F);
  EXPECT_NEAR(attitude.pitch, degToRad(5.0F), 1e-5F);
}

TEST(ComplementaryFilterTest, FollowsGyroDuringFastRotation) {
  rb::ComplementaryFilter filter;
  filter.update(restingAccel(0.0F), {}, 0.005F);
  // Rotate at 1 rad/s for 0.1 s while the accelerometer (misleadingly) still reads level.
  for (int i = 0; i < 20; ++i) {
    filter.update(restingAccel(0.0F), {0.0F, 1.0F, 0.0F}, 0.005F);
  }
  // Mostly the gyro integral (0.1 rad), slightly pulled back by the accelerometer.
  EXPECT_NEAR(filter.attitude().pitch, 0.1F, 0.01F);
  EXPECT_FLOAT_EQ(filter.attitude().pitch_rate, 1.0F);
}

TEST(ComplementaryFilterTest, CorrectsGyroDriftTowardsAccelerometer) {
  rb::ComplementaryFilter filter;
  filter.update(restingAccel(0.0F), {}, 0.005F);
  for (int i = 0; i < 2000; ++i) {  // 10 s with an uncorrected 0.02 rad/s gyro offset
    filter.update(restingAccel(0.0F), {0.0F, 0.02F, 0.0F}, 0.005F);
  }
  // Steady-state error of a complementary filter = offset * time constant.
  EXPECT_NEAR(filter.attitude().pitch, 0.02F * 0.5F, 0.002F);
}

TEST(ComplementaryFilterTest, IgnoresAccelerometerDuringStrongAcceleration) {
  rb::ComplementaryFilter filter;
  filter.update(restingAccel(0.0F), {}, 0.005F);
  const Vec3 accelerating{-9.0F, 0.0F, kGravity};  // |a| = 1.36 g: clearly not gravity alone
  for (int i = 0; i < 100; ++i) {
    filter.update(accelerating, {}, 0.005F);
  }
  EXPECT_NEAR(filter.attitude().pitch, 0.0F, 1e-6F);
}

TEST(ComplementaryFilterTest, IgnoresNonFiniteSamples) {
  rb::ComplementaryFilter filter;
  filter.update(restingAccel(degToRad(3.0F)), {}, 0.005F);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  filter.update({nan, 0.0F, kGravity}, {}, 0.005F);
  filter.update(restingAccel(0.0F), {0.0F, nan, 0.0F}, 0.005F);
  filter.update(restingAccel(0.0F), {}, 0.0F);
  EXPECT_NEAR(filter.attitude().pitch, degToRad(3.0F), 1e-5F);
}

TEST(GyroBiasEstimatorTest, EstimatesBiasWhileStill) {
  rb::GyroBiasEstimator estimator;
  std::mt19937 rng(3);
  std::normal_distribution<float> noise(0.0F, 0.003F);
  bool ready = false;
  for (int i = 0; i < 400; ++i) {
    ready = estimator.update({0.01F + noise(rng), -0.02F + noise(rng), 0.005F + noise(rng)});
  }
  ASSERT_TRUE(ready);
  EXPECT_NEAR(estimator.bias().x, 0.01F, 0.001F);
  EXPECT_NEAR(estimator.bias().y, -0.02F, 0.001F);
  const Vec3 corrected = estimator.correct({0.01F, -0.02F, 0.005F});
  EXPECT_NEAR(corrected.z, 0.0F, 0.001F);
}

TEST(GyroBiasEstimatorTest, RestartsWindowWhenRobotMoves) {
  rb::GyroBiasEstimator estimator;
  for (int i = 0; i < 300; ++i) {
    estimator.update({});
  }
  estimator.update({0.0F, 0.5F, 0.0F});  // a bump
  for (int i = 0; i < 300; ++i) {
    estimator.update({});
  }
  EXPECT_FALSE(estimator.ready());  // only 300 still samples since the bump
  for (int i = 0; i < 100; ++i) {
    estimator.update({});
  }
  EXPECT_TRUE(estimator.ready());
}

TEST(GyroBiasEstimatorTest, RestartKeepsPreviousBias) {
  rb::GyroBiasEstimator estimator({10, 0.05F});
  for (int i = 0; i < 10; ++i) {
    estimator.update({0.1F, 0.0F, 0.0F});
  }
  ASSERT_TRUE(estimator.ready());
  estimator.restart();
  EXPECT_FALSE(estimator.ready());
  EXPECT_NEAR(estimator.bias().x, 0.1F, 1e-6F);
}

}  // namespace
