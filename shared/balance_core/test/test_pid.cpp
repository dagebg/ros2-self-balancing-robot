#include <gtest/gtest.h>

#include <limits>

#include "balance_core/pid.hpp"

namespace {

using robot::balance::Pid;
using robot::balance::PidGains;
using robot::balance::PidLimits;

constexpr float kDt = 0.01F;
const PidLimits kWide{-100.0F, 100.0F, -100.0F, 100.0F};

TEST(PidTest, ProportionalTermActsOnError) {
  Pid pid({2.0F, 0.0F, 0.0F}, kWide);
  EXPECT_FLOAT_EQ(pid.update(1.0F, 0.25F, kDt), 1.5F);
}

TEST(PidTest, IntegralAccumulatesError) {
  Pid pid({0.0F, 10.0F, 0.0F}, kWide);
  pid.update(1.0F, 0.0F, kDt);
  EXPECT_FLOAT_EQ(pid.update(1.0F, 0.0F, kDt), 0.2F);
}

TEST(PidTest, DerivativeActsOnMeasurementNotOnSetpoint) {
  Pid pid({0.0F, 0.0F, 1.0F}, kWide);
  pid.update(0.0F, 0.0F, kDt);
  EXPECT_FLOAT_EQ(pid.update(5.0F, 0.0F, kDt), 0.0F);    // setpoint step: no kick
  EXPECT_FLOAT_EQ(pid.update(5.0F, 0.1F, kDt), -10.0F);  // measurement rising at 10/s
}

TEST(PidTest, UsesMeasuredRateWhenGiven) {
  Pid pid({0.0F, 0.0F, 0.5F}, kWide);
  EXPECT_FLOAT_EQ(pid.update(0.0F, 0.0F, kDt, 2.0F), -1.0F);
}

TEST(PidTest, OutputIsClamped) {
  Pid pid({10.0F, 0.0F, 0.0F}, {-1.0F, 1.0F, -1.0F, 1.0F});
  EXPECT_FLOAT_EQ(pid.update(5.0F, 0.0F, kDt), 1.0F);
  EXPECT_FLOAT_EQ(pid.update(-5.0F, 0.0F, kDt), -1.0F);
}

TEST(PidTest, IntegralDoesNotWindUpWhileSaturated) {
  Pid pid({1.0F, 5.0F, 0.0F}, {-1.0F, 1.0F, -1.0F, 1.0F});
  for (int i = 0; i < 1000; ++i) {  // 10 s of large error: P alone saturates
    pid.update(10.0F, 0.0F, kDt);
  }
  EXPECT_LT(pid.integral(), 0.1F);
  // When the error reverses, the output follows immediately instead of unwinding first.
  EXPECT_LT(pid.update(-0.5F, 0.0F, kDt), 0.0F);
}

TEST(PidTest, IntegralIsClampedToItsLimits) {
  Pid pid({0.0F, 100.0F, 0.0F}, {-10.0F, 10.0F, -0.5F, 0.5F});
  for (int i = 0; i < 100; ++i) {
    pid.update(1.0F, 0.0F, kDt);
  }
  EXPECT_FLOAT_EQ(pid.integral(), 0.5F);
}

TEST(PidTest, NonFiniteInputsHoldTheLastOutput) {
  Pid pid({1.0F, 1.0F, 1.0F}, kWide);
  const float last = pid.update(1.0F, 0.0F, kDt);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FLOAT_EQ(pid.update(nan, 0.0F, kDt), last);
  EXPECT_FLOAT_EQ(pid.update(1.0F, nan, kDt), last);
  EXPECT_FLOAT_EQ(pid.update(1.0F, 0.0F, 0.0F), last);
  EXPECT_FLOAT_EQ(pid.update(1.0F, 0.0F, kDt, nan), last);
}

TEST(PidTest, ResetClearsState) {
  Pid pid({1.0F, 10.0F, 0.0F}, kWide);
  pid.update(1.0F, 0.0F, kDt);
  pid.reset();
  EXPECT_FLOAT_EQ(pid.integral(), 0.0F);
  EXPECT_FLOAT_EQ(pid.output(), 0.0F);
}

TEST(PidTest, GainsCanBeChangedAtRuntime) {
  Pid pid({1.0F, 0.0F, 0.0F}, kWide);
  pid.setGains({3.0F, 0.0F, 0.0F});
  EXPECT_FLOAT_EQ(pid.gains().kp, 3.0F);
  EXPECT_FLOAT_EQ(pid.update(1.0F, 0.0F, kDt), 3.0F);
}

}  // namespace
