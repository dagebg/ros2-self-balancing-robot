#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "balance_core/filters.hpp"
#include "balance_core/math.hpp"

namespace {

using robot::balance::LowPassFilter;
using robot::balance::SlewRateLimiter;

TEST(LowPassFilterTest, FirstSampleInitializesOutput) {
  LowPassFilter filter(0.1F);
  EXPECT_FLOAT_EQ(filter.update(5.0F, 0.01F), 5.0F);
}

TEST(LowPassFilterTest, ReachesAbout63PercentAfterOneTimeConstant) {
  LowPassFilter filter(0.1F);
  filter.update(0.0F, 0.001F);
  float value = 0.0F;
  for (int i = 0; i < 100; ++i) {  // 0.1 s in 1 ms steps
    value = filter.update(1.0F, 0.001F);
  }
  EXPECT_NEAR(value, 1.0F - std::exp(-1.0F), 0.01F);
}

TEST(LowPassFilterTest, ResponseIsIndependentOfSampleRate) {
  LowPassFilter fast(0.1F);
  LowPassFilter slow(0.1F);
  fast.update(0.0F, 0.001F);
  slow.update(0.0F, 0.005F);
  for (int i = 0; i < 200; ++i) {
    fast.update(1.0F, 0.001F);
  }
  for (int i = 0; i < 40; ++i) {
    slow.update(1.0F, 0.005F);
  }
  EXPECT_NEAR(fast.value(), slow.value(), 0.01F);
}

TEST(LowPassFilterTest, IgnoresNonFiniteSamples) {
  LowPassFilter filter(0.1F);
  filter.update(2.0F, 0.01F);
  EXPECT_FLOAT_EQ(filter.update(std::numeric_limits<float>::quiet_NaN(), 0.01F), 2.0F);
}

TEST(LowPassFilterTest, ZeroTimeConstantPassesThrough) {
  LowPassFilter filter(0.0F);
  filter.update(1.0F, 0.01F);
  EXPECT_FLOAT_EQ(filter.update(7.0F, 0.01F), 7.0F);
}

TEST(SlewRateLimiterTest, LimitsRateOfChange) {
  SlewRateLimiter limiter(2.0F);  // units per second
  EXPECT_FLOAT_EQ(limiter.update(10.0F, 0.1F), 0.2F);
  EXPECT_FLOAT_EQ(limiter.update(10.0F, 0.1F), 0.4F);
  EXPECT_FLOAT_EQ(limiter.update(-10.0F, 0.1F), 0.2F);
}

TEST(SlewRateLimiterTest, ReachesTargetAndStays) {
  SlewRateLimiter limiter(1.0F);
  for (int i = 0; i < 100; ++i) {
    limiter.update(0.5F, 0.01F);
  }
  EXPECT_FLOAT_EQ(limiter.value(), 0.5F);
}

TEST(SlewRateLimiterTest, IgnoresNonFiniteTargets) {
  SlewRateLimiter limiter(1.0F);
  limiter.update(1.0F, 0.1F);
  EXPECT_FLOAT_EQ(limiter.update(std::numeric_limits<float>::infinity(), 0.1F), 0.1F);
}

TEST(MathTest, ClampFiniteMapsNanToZero) {
  EXPECT_FLOAT_EQ(robot::balance::clampFinite(std::nanf(""), -1.0F, 1.0F), 0.0F);
  EXPECT_FLOAT_EQ(robot::balance::clampFinite(3.0F, -1.0F, 1.0F), 1.0F);
}

TEST(MathTest, WrapAngleStaysInHalfOpenInterval) {
  using robot::balance::kPi;
  using robot::balance::wrapAngle;
  EXPECT_NEAR(wrapAngle(3.0F * kPi), kPi, 1e-5F);
  EXPECT_NEAR(wrapAngle(-kPi), kPi, 1e-5F);
  EXPECT_NEAR(wrapAngle(0.5F), 0.5F, 1e-6F);
  EXPECT_NEAR(wrapAngle(-2.5F * kPi), -0.5F * kPi, 1e-5F);
}

}  // namespace
