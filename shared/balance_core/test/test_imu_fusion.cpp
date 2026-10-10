#include <gtest/gtest.h>

#include "balance_core/imu_fusion.hpp"
#include "balance_core/math.hpp"

namespace {

namespace rb = robot::balance;
using rb::kGravity;

rb::ImuSampleBody sample(float gyro_y, float accel_x = 0.0F) {
  return {{accel_x, 0.0F, kGravity}, {0.0F, gyro_y, 0.0F}};
}

TEST(ImuFusionTest, AveragesAgreeingSensors) {
  rb::ImuFusion fusion;
  const auto fused = fusion.update(sample(0.10F, 1.0F), sample(0.12F, 3.0F), 0.005F);
  ASSERT_TRUE(fused.has_value());
  EXPECT_FLOAT_EQ(fused->gyro.y, 0.11F);
  EXPECT_FLOAT_EQ(fused->accel.x, 2.0F);
  EXPECT_FALSE(fusion.disagreement());
}

TEST(ImuFusionTest, UsesTheRemainingSensorWhenOneIsMissing) {
  rb::ImuFusion fusion;
  EXPECT_FLOAT_EQ(fusion.update(sample(0.3F), std::nullopt, 0.005F)->gyro.y, 0.3F);
  EXPECT_FLOAT_EQ(fusion.update(std::nullopt, sample(0.4F), 0.005F)->gyro.y, 0.4F);
  EXPECT_FALSE(fusion.update(std::nullopt, std::nullopt, 0.005F).has_value());
}

TEST(ImuFusionTest, FlagsPersistentDisagreementAndFallsBackToPrimary) {
  rb::ImuFusion fusion;  // 50 ms debounce
  for (int i = 0; i < 9; ++i) {
    (void)fusion.update(sample(0.0F), sample(1.0F), 0.005F);
  }
  EXPECT_FALSE(fusion.disagreement());  // a 45 ms glitch is tolerated

  (void)fusion.update(sample(0.0F), sample(1.0F), 0.005F);
  const auto fused = fusion.update(sample(0.0F), sample(1.0F), 0.005F);  // 55 ms
  EXPECT_TRUE(fusion.disagreement());
  EXPECT_FLOAT_EQ(fused->gyro.y, 0.0F);

  (void)fusion.update(sample(0.0F), sample(0.0F), 0.005F);
  EXPECT_FALSE(fusion.disagreement());  // recovers as soon as they agree again
}

TEST(ImuFusionTest, ToleranceIsPerAxis) {
  rb::ImuFusion fusion({0.1F, 0.0F});  // no debounce
  const rb::ImuSampleBody a{{}, {0.0F, 0.0F, 0.0F}};
  const rb::ImuSampleBody b{{}, {0.05F, -0.05F, 0.11F}};  // only z exceeds the tolerance
  (void)fusion.update(a, b, 0.005F);
  EXPECT_TRUE(fusion.disagreement());
}

TEST(ImuFusionTest, MissingSensorClearsDisagreement) {
  rb::ImuFusion fusion({0.1F, 0.0F});
  const rb::ImuSampleBody a{{}, {0.0F, 0.0F, 0.0F}};
  const rb::ImuSampleBody b{{}, {1.0F, 0.0F, 0.0F}};
  (void)fusion.update(a, b, 0.005F);
  ASSERT_TRUE(fusion.disagreement());
  (void)fusion.update(a, std::nullopt, 0.005F);
  EXPECT_FALSE(fusion.disagreement());
}

}  // namespace
