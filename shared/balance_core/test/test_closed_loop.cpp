// Controller tests: sign conventions first, then closed-loop behaviour against the
// WheeledPendulum plant with noisy, biased, delayed sensors and quantized encoders.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "balance_core/balance_controller.hpp"
#include "closed_loop.hpp"

namespace {

namespace rb = robot::balance;
using rb::degToRad;
using rb::SafetyState;
using rb::testing::ClosedLoop;

constexpr float kDt = 0.005F;

rb::BalanceInput input(float pitch, float pitch_rate = 0.0F) {
  rb::BalanceInput in;
  in.pitch = pitch;
  in.pitch_rate = pitch_rate;
  in.dt = kDt;
  return in;
}

// --- Sign conventions and limits -----------------------------------------------------------------

TEST(BalanceControllerTest, TiltingForwardsDrivesForwards) {
  rb::BalanceController controller;
  const auto out = controller.update(input(degToRad(5.0F)));
  EXPECT_GT(out.left, 0.0F);
  EXPECT_GT(out.right, 0.0F);
  EXPECT_FLOAT_EQ(out.left, out.right);
}

TEST(BalanceControllerTest, FallingBackwardsDrivesBackwards) {
  rb::BalanceController controller;
  const auto out = controller.update(input(degToRad(-2.0F), -0.5F));
  EXPECT_LT(out.left, 0.0F);
}

TEST(BalanceControllerTest, CounterClockwiseTurnSpeedsUpRightWheel) {
  rb::BalanceController controller;
  auto in = input(0.0F);
  in.target_yaw_rate = 1.0F;
  rb::BalanceOutput out{};
  for (int i = 0; i < 50; ++i) {
    out = controller.update(in);
  }
  EXPECT_GT(out.right, out.left);
}

TEST(BalanceControllerTest, ForwardCommandLeansForwards) {
  rb::BalanceController controller;
  auto in = input(0.0F);
  in.target_velocity = 0.2F;
  rb::BalanceOutput out{};
  for (int i = 0; i < 50; ++i) {
    out = controller.update(in);
  }
  EXPECT_GT(out.target_pitch, 0.0F);
  EXPECT_LE(out.target_pitch, controller.config().max_lean);
}

TEST(BalanceControllerTest, CommandsAreRampedAndClamped) {
  rb::BalanceController controller;
  auto in = input(0.0F);
  in.target_velocity = 5.0F;  // far above max_velocity
  const auto first = controller.update(in);
  EXPECT_NEAR(first.ramped_velocity, controller.config().max_acceleration * kDt, 1e-6F);
  for (int i = 0; i < 1000; ++i) {
    controller.update(in);
  }
  EXPECT_FLOAT_EQ(controller.update(in).ramped_velocity, controller.config().max_velocity);
}

TEST(BalanceControllerTest, BalanceHasPriorityOverSteering) {
  rb::BalanceController controller;
  auto in = input(degToRad(30.0F));  // saturates the pitch loop
  in.target_yaw_rate = 2.0F;
  rb::BalanceOutput out{};
  for (int i = 0; i < 100; ++i) {
    out = controller.update(in);
  }
  EXPECT_FLOAT_EQ(out.left, 1.0F);
  EXPECT_FLOAT_EQ(out.right, 1.0F);  // no headroom left for turning
}

TEST(BalanceControllerTest, NonFiniteCommandsAreTreatedAsZero) {
  rb::BalanceController controller;
  auto in = input(0.0F);
  in.target_velocity = std::numeric_limits<float>::quiet_NaN();
  in.target_yaw_rate = std::numeric_limits<float>::infinity();
  const auto out = controller.update(in);
  EXPECT_TRUE(std::isfinite(out.left));
  EXPECT_FLOAT_EQ(out.ramped_velocity, 0.0F);
}

TEST(BalanceControllerTest, GainsCanBeChangedPerLoop) {
  rb::BalanceController controller;
  controller.setGains(rb::Loop::kVelocity, {1.0F, 2.0F, 3.0F});
  EXPECT_FLOAT_EQ(controller.gains(rb::Loop::kVelocity).ki, 2.0F);
  EXPECT_FLOAT_EQ(controller.gains(rb::Loop::kPitch).kp, controller.config().pitch_gains.kp);
}

// --- Closed loop
// -----------------------------------------------------------------------------------

class ClosedLoopTest : public ::testing::Test {
 protected:
  void SetUp() override {
    loop.calibrate();
    ASSERT_TRUE(loop.armAndStart());
  }
  ClosedLoop loop;
};

TEST_F(ClosedLoopTest, StaysUpright) {
  loop.run(10.0F);
  EXPECT_EQ(loop.safety().state(), SafetyState::kBalancing);
  EXPECT_LT(loop.maxAbsPitch(), degToRad(2.0F));
  EXPECT_LT(std::fabs(loop.plant().state().x), 0.2F);  // does not wander off
}

TEST_F(ClosedLoopTest, RecoversFromInitialTilt) {
  loop.plant().push(0.15F);  // starts falling forwards
  loop.run(3.0F);
  EXPECT_EQ(loop.safety().state(), SafetyState::kBalancing);
  EXPECT_LT(std::fabs(loop.plant().state().pitch), degToRad(0.5F));
  EXPECT_LT(std::fabs(loop.plant().state().velocity), 0.05F);
}

TEST_F(ClosedLoopTest, RejectsAPush) {
  loop.run(1.0F);
  loop.resetMaxAbsPitch();
  loop.plant().push(-0.6F);  // shove backwards
  loop.run(3.0F);
  EXPECT_EQ(loop.safety().state(), SafetyState::kBalancing);
  EXPECT_LT(loop.maxAbsPitch(), degToRad(15.0F));
  EXPECT_LT(std::fabs(loop.plant().state().pitch), degToRad(1.0F));
}

TEST_F(ClosedLoopTest, TracksForwardVelocity) {
  loop.run(5.0F, 0.15F, 0.0F);
  EXPECT_NEAR(loop.plant().state().velocity, 0.15F, 0.03F);
  EXPECT_GT(loop.plant().state().x, 0.4F);
  loop.run(4.0F, 0.0F, 0.0F);
  EXPECT_NEAR(loop.plant().state().velocity, 0.0F, 0.03F);
  EXPECT_EQ(loop.safety().state(), SafetyState::kBalancing);
}

TEST_F(ClosedLoopTest, TurnsOnTheSpot) {
  loop.run(3.0F, 0.0F, 1.0F);
  EXPECT_NEAR(loop.plant().state().yaw_rate, 1.0F, 0.2F);
  EXPECT_LT(std::fabs(loop.plant().state().velocity), 0.05F);
  EXPECT_EQ(loop.safety().state(), SafetyState::kBalancing);
}

TEST(ClosedLoopFailureTest, MotorsTooWeakEndInTiltFaultNotRunaway) {
  rb::sim::PendulumParams weak;
  // Balancing near upright needs surprisingly little torque (about 0.01 N m holds 1.5 deg), so
  // the motors must be far weaker than that, and the push strong, for the robot to fall.
  weak.stall_torque = 0.001F;
  ClosedLoop loop({}, {}, {}, weak);
  loop.calibrate();
  ASSERT_TRUE(loop.armAndStart());
  loop.plant().push(1.0F);
  loop.run(3.0F);
  EXPECT_EQ(loop.safety().state(), SafetyState::kFault);
  EXPECT_EQ(loop.safety().faults(), rb::fault::kTiltLimit);
  EXPECT_FLOAT_EQ(loop.lastCommand().left, 0.0F);  // motors off after the fault
}

struct Variant {
  float mass;
  float com_height;
  float stall_torque;
};

class RobustnessTest : public ::testing::TestWithParam<Variant> {};

// The real robot's parameters are not measured yet; the default gains must cope with a range.
TEST_P(RobustnessTest, DefaultGainsBalanceDespiteParameterError) {
  const Variant v = GetParam();
  rb::sim::PendulumParams params;
  params.body_mass = v.mass;
  params.com_height = v.com_height;
  params.stall_torque = v.stall_torque;
  params.body_inertia = v.mass * (4.0F * v.com_height * v.com_height + 0.01F) / 12.0F;
  ClosedLoop loop({}, {}, {}, params);
  loop.calibrate();
  ASSERT_TRUE(loop.armAndStart());
  loop.plant().push(0.5F);
  loop.run(4.0F, 0.1F, 0.5F);
  EXPECT_EQ(loop.safety().state(), SafetyState::kBalancing);
  EXPECT_LT(loop.maxAbsPitch(), degToRad(15.0F));
}

INSTANTIATE_TEST_SUITE_P(MassComTorque, RobustnessTest,
                         ::testing::Values(Variant{0.65F, 0.05F, 0.10F},
                                           Variant{0.65F, 0.11F, 0.20F},
                                           Variant{1.20F, 0.05F, 0.20F},
                                           Variant{1.20F, 0.11F, 0.10F},
                                           Variant{0.90F, 0.08F, 0.15F}));

}  // namespace
