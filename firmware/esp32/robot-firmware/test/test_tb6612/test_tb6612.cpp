#include <gtest/gtest.h>

#include <limits>

#include "drivers/tb6612.hpp"
#include "hal_fakes/fakes.hpp"

namespace {

using robot::drivers::HBridgeChannel;
using robot::drivers::Tb6612;
using robot::hal_fakes::FakeDigitalOutput;
using robot::hal_fakes::FakePwm;

class HBridgeTest : public ::testing::Test {
 protected:
  HBridgeChannel channel(bool inverted = false, float max_duty = 1.0F) {
    return HBridgeChannel(pwm, in1, in2, inverted, max_duty);
  }

  FakePwm pwm;
  FakeDigitalOutput in1;
  FakeDigitalOutput in2;
};

TEST_F(HBridgeTest, PositiveCommandDrivesForward) {
  auto motor = channel();
  motor.set(0.5F);
  EXPECT_TRUE(in1.level);
  EXPECT_FALSE(in2.level);
  EXPECT_FLOAT_EQ(pwm.duty(), 0.5F);
}

TEST_F(HBridgeTest, NegativeCommandDrivesReverse) {
  auto motor = channel();
  motor.set(-0.25F);
  EXPECT_FALSE(in1.level);
  EXPECT_TRUE(in2.level);
  EXPECT_FLOAT_EQ(pwm.duty(), 0.25F);
}

TEST_F(HBridgeTest, InversionSwapsDirection) {
  auto motor = channel(/*inverted=*/true);
  motor.set(0.5F);
  EXPECT_FALSE(in1.level);
  EXPECT_TRUE(in2.level);
  EXPECT_FLOAT_EQ(motor.command(), 0.5F);  // reported command stays logical
}

TEST_F(HBridgeTest, MaxDutyScalesAndLimitsOutput) {
  auto motor = channel(false, 0.52F);
  motor.set(1.0F);
  EXPECT_FLOAT_EQ(pwm.duty(), 0.52F);
  motor.set(0.5F);
  EXPECT_FLOAT_EQ(pwm.duty(), 0.26F);
  EXPECT_FLOAT_EQ(motor.appliedDuty(), 0.26F);
}

TEST_F(HBridgeTest, CommandsAreClampedToUnitRange) {
  auto motor = channel();
  motor.set(3.0F);
  EXPECT_FLOAT_EQ(pwm.duty(), 1.0F);
  EXPECT_FLOAT_EQ(motor.command(), 1.0F);
  motor.set(-7.0F);
  EXPECT_FLOAT_EQ(pwm.duty(), 1.0F);
  EXPECT_FLOAT_EQ(motor.command(), -1.0F);
}

TEST_F(HBridgeTest, ZeroAndNanCoast) {
  auto motor = channel();
  motor.set(0.8F);
  motor.set(std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(in1.level);
  EXPECT_FALSE(in2.level);
  EXPECT_FLOAT_EQ(pwm.duty(), 0.0F);

  motor.set(0.8F);
  motor.set(0.0F);
  EXPECT_FALSE(in1.level);
  EXPECT_FALSE(in2.level);
  EXPECT_FLOAT_EQ(motor.command(), 0.0F);
}

TEST_F(HBridgeTest, PwmIsZeroWhileDirectionChanges) {
  auto motor = channel();
  motor.set(0.6F);
  pwm.history.clear();
  motor.set(-0.6F);
  ASSERT_EQ(pwm.history.size(), 2U);
  EXPECT_FLOAT_EQ(pwm.history[0], 0.0F);
  EXPECT_FLOAT_EQ(pwm.history[1], 0.6F);
}

TEST_F(HBridgeTest, BrakeShortsBothTerminals) {
  auto motor = channel();
  motor.set(0.6F);
  motor.brake();
  EXPECT_TRUE(in1.level);
  EXPECT_TRUE(in2.level);
  EXPECT_FLOAT_EQ(pwm.duty(), 0.0F);
}

TEST_F(HBridgeTest, InvalidMaxDutyDisablesOutput) {
  auto motor = channel(false, std::numeric_limits<float>::infinity());
  motor.set(1.0F);
  EXPECT_FLOAT_EQ(pwm.duty(), 0.0F);
}

TEST(Tb6612Test, StandbyControlsTheChip) {
  FakeDigitalOutput standby;
  Tb6612 driver(standby);
  EXPECT_FALSE(driver.enabled());

  driver.enable();
  EXPECT_TRUE(standby.level);
  EXPECT_TRUE(driver.enabled());

  driver.disable();
  EXPECT_FALSE(standby.level);
  EXPECT_FALSE(driver.enabled());
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
