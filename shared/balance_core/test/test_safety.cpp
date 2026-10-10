#include <gtest/gtest.h>

#include <limits>

#include "balance_core/safety.hpp"

namespace {

namespace rb = robot::balance;
using rb::degToRad;
using rb::SafetyState;
using rb::SafetyStateMachine;

constexpr float kDt = 0.005F;

/// Runs `seconds` of updates with a constant pitch.
rb::SafetyUpdate hold(SafetyStateMachine& safety, float pitch, float seconds, bool imu_ok = true) {
  rb::SafetyUpdate last{};
  for (float t = 0.0F; t < seconds; t += kDt) {
    last = safety.update(pitch, imu_ok, kDt);
  }
  return last;
}

SafetyStateMachine balancingMachine() {
  SafetyStateMachine safety;
  safety.update(0.0F, true, kDt);
  safety.requestArm();
  hold(safety, 0.0F, 0.6F);
  return safety;
}

TEST(SafetyTest, StartsDisarmedWithMotorsOff) {
  SafetyStateMachine safety;
  EXPECT_EQ(safety.state(), SafetyState::kDisarmed);
  EXPECT_FALSE(safety.motorsEnabled());
}

TEST(SafetyTest, ArmingRequiresWorkingImu) {
  SafetyStateMachine safety;
  EXPECT_FALSE(safety.requestArm());  // no IMU data yet
  safety.update(0.0F, false, kDt);
  EXPECT_FALSE(safety.requestArm());
  safety.update(0.0F, true, kDt);
  EXPECT_TRUE(safety.requestArm());
  EXPECT_EQ(safety.state(), SafetyState::kArmed);
}

TEST(SafetyTest, StartsBalancingAfterBeingUprightLongEnough) {
  SafetyStateMachine safety;
  safety.update(degToRad(40.0F), true, kDt);  // lying on its stand
  ASSERT_TRUE(safety.requestArm());
  hold(safety, degToRad(40.0F), 2.0F);
  EXPECT_EQ(safety.state(), SafetyState::kArmed);  // waits, motors stay off
  EXPECT_FALSE(safety.motorsEnabled());

  hold(safety, degToRad(1.0F), 0.45F);
  EXPECT_EQ(safety.state(), SafetyState::kArmed);  // not yet 0.5 s
  const auto update = hold(safety, degToRad(1.0F), 0.1F);
  EXPECT_EQ(update.state, SafetyState::kBalancing);
  EXPECT_TRUE(safety.motorsEnabled());
}

TEST(SafetyTest, UprightTimerRestartsWhenTilted) {
  SafetyStateMachine safety;
  safety.update(0.0F, true, kDt);
  safety.requestArm();
  hold(safety, 0.0F, 0.4F);
  hold(safety, degToRad(10.0F), 0.05F);
  hold(safety, 0.0F, 0.4F);
  EXPECT_EQ(safety.state(), SafetyState::kArmed);
}

TEST(SafetyTest, ReportsBalancingStartExactlyOnce) {
  SafetyStateMachine safety;
  safety.update(0.0F, true, kDt);
  safety.requestArm();
  int starts = 0;
  for (int i = 0; i < 400; ++i) {
    starts += safety.update(0.0F, true, kDt).started_balancing ? 1 : 0;
  }
  EXPECT_EQ(starts, 1);
}

TEST(SafetyTest, FallingLatchesTiltFault) {
  auto safety = balancingMachine();
  ASSERT_EQ(safety.state(), SafetyState::kBalancing);

  safety.update(degToRad(46.0F), true, kDt);
  EXPECT_EQ(safety.state(), SafetyState::kFault);
  EXPECT_EQ(safety.faults(), rb::fault::kTiltLimit);
  EXPECT_FALSE(safety.motorsEnabled());

  hold(safety, 0.0F, 2.0F);  // picking it up again does not restart the motors
  EXPECT_EQ(safety.state(), SafetyState::kFault);
  EXPECT_FALSE(safety.requestArm());
}

TEST(SafetyTest, ImuLossWhileBalancingLatchesImuFault) {
  auto safety = balancingMachine();
  safety.update(0.0F, false, kDt);
  EXPECT_EQ(safety.state(), SafetyState::kFault);
  EXPECT_EQ(safety.faults(), rb::fault::kImuFailure);
}

TEST(SafetyTest, NonFinitePitchCountsAsImuFailure) {
  auto safety = balancingMachine();
  safety.update(std::numeric_limits<float>::quiet_NaN(), true, kDt);
  EXPECT_EQ(safety.faults(), rb::fault::kImuFailure);
}

TEST(SafetyTest, ImuLossWhileArmedLatchesImuFault) {
  SafetyStateMachine safety;
  safety.update(0.0F, true, kDt);
  safety.requestArm();
  safety.update(0.0F, false, kDt);
  EXPECT_EQ(safety.state(), SafetyState::kFault);
}

TEST(SafetyTest, DisarmClearsFaultsFromAnyState) {
  auto safety = balancingMachine();
  safety.update(degToRad(60.0F), true, kDt);
  safety.requestDisarm();
  EXPECT_EQ(safety.state(), SafetyState::kDisarmed);
  EXPECT_EQ(safety.faults(), 0U);

  safety.update(0.0F, true, kDt);
  EXPECT_TRUE(safety.requestArm());
}

TEST(SafetyTest, ArmingIsIdempotent) {
  auto safety = balancingMachine();
  EXPECT_TRUE(safety.requestArm());
  EXPECT_EQ(safety.state(), SafetyState::kBalancing);
}

}  // namespace
