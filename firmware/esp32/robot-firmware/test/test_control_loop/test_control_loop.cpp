// Firmware-level software-in-the-loop test: the real ControlLoop drives the WheeledPendulum plant
// through the same data path as on the robot (sensor-axis IMU readings, raw encoder counts).
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "app/control_loop.hpp"
#include "balance_core/wheeled_pendulum.hpp"

namespace {

namespace rb = robot::balance;
namespace rp = robot::protocol;
using robot::app::ControlConfig;
using robot::app::ControlLoop;
using robot::app::ControlOutput;
using robot::app::MotionCommand;
using robot::app::SensorFrame;
using robot::drivers::AxisMap;
using robot::drivers::ImuReading;
using robot::drivers::Vector3;

constexpr float kCountsPerRev = 4200.0F;
constexpr std::uint32_t kPeriodUs = 5000;

ControlConfig testConfig() {
  ControlConfig config;
  // Deliberately non-trivial mountings and an inverted encoder: the loop must undo all of them.
  config.primary_mounting = AxisMap(3, 1, 2);      // board standing upright, facing forwards
  config.secondary_mounting = AxisMap(-1, -2, 3);  // flat, facing backwards
  config.geometry = {0.0325F, 0.15F};
  config.counts_per_wheel_rev = kCountsPerRev;
  config.right_encoder_inverted = true;
  return config;
}

class Robot {
 public:
  Robot() : config_(testConfig()), loop_(config_), rng_(5) {}

  /// One 5 ms tick: sensors from the plant -> ControlLoop -> plant.
  ControlOutput tick(const MotionCommand& command) {
    const ControlOutput out = loop_.step(frame(), command);
    plant_.step(out.left, out.right, kPeriodUs * 1e-6F);
    time_us_ += kPeriodUs;
    return out;
  }

  ControlOutput run(float seconds, MotionCommand command) {
    ControlOutput out{};
    for (int i = 0; i < static_cast<int>(seconds / (kPeriodUs * 1e-6F)); ++i) {
      out = tick(command);
    }
    return out;
  }

  ControlLoop& loop() { return loop_; }
  rb::sim::WheeledPendulum& plant() { return plant_; }

 private:
  ImuReading reading(const AxisMap& mounting, Vector3 gyro_bias) {
    std::normal_distribution<float> noise(0.0F, 0.003F);
    const rb::Vec3 a = plant_.accelerometer();
    const rb::Vec3 g = plant_.gyroscope();
    ImuReading r;
    r.accel = mounting.toSensor({a.x + noise(rng_), a.y + noise(rng_), a.z + noise(rng_)});
    const Vector3 gyro_sensor = mounting.toSensor({g.x, g.y, g.z});
    r.gyro = {gyro_sensor.x + gyro_bias.x + noise(rng_), gyro_sensor.y + gyro_bias.y + noise(rng_),
              gyro_sensor.z + gyro_bias.z + noise(rng_)};
    r.temperature = 30.0F;
    return r;
  }

  SensorFrame frame() {
    SensorFrame f;
    f.time_us = time_us_;
    f.primary = reading(config_.primary_mounting, {0.02F, -0.01F, 0.015F});
    f.secondary = reading(config_.secondary_mounting, {-0.015F, 0.01F, 0.02F});
    const auto counts = [](float angle) {
      return static_cast<std::int32_t>(std::floor(angle * kCountsPerRev / (2.0F * rb::kPi)));
    };
    f.left_ticks = counts(plant_.state().left_wheel_angle);
    f.right_ticks = -counts(plant_.state().right_wheel_angle);  // wired reversed
    return f;
  }

  ControlConfig config_;
  ControlLoop loop_;
  rb::sim::WheeledPendulum plant_;
  std::mt19937 rng_;
  std::uint32_t time_us_ = 123456;
};

const MotionCommand kIdle{0.0F, 0.0F, true};

rp::Ack arm(ControlLoop& loop, bool armed) { return loop.handle(rp::SetArmed{armed}, 7); }

TEST(ControlLoopTest, RejectsArmingUntilGyroIsCalibrated) {
  Robot robot;
  robot.tick(kIdle);
  EXPECT_FALSE(robot.loop().calibrated());
  EXPECT_EQ(arm(robot.loop(), true).status, rp::AckStatus::kRejected);

  robot.run(2.5F, kIdle);  // standing still: 2 s window completes
  EXPECT_TRUE(robot.loop().calibrated());
  const rp::Ack ack = arm(robot.loop(), true);
  EXPECT_EQ(ack.status, rp::AckStatus::kOk);
  EXPECT_EQ(ack.acked_type, rp::MessageType::kSetArmed);
  EXPECT_EQ(ack.acked_sequence, 7);
}

TEST(ControlLoopTest, EstimatesAttitudeThroughMountingsAndBias) {
  Robot robot;
  robot.run(2.5F, kIdle);
  const auto out = robot.run(0.5F, kIdle);
  EXPECT_NEAR(out.telemetry.pitch, 0.0F, rb::degToRad(0.5F));
  EXPECT_NEAR(out.telemetry.roll, 0.0F, rb::degToRad(0.5F));
  EXPECT_NEAR(out.telemetry.pitch_rate, 0.0F, 0.02F);  // bias removed
  ASSERT_TRUE(out.primary_sample.has_value());
  EXPECT_NEAR(out.primary_sample->accel[2], rb::kGravity, 0.1F);  // body z is up
}

TEST(ControlLoopTest, BalancesAndDrivesForwardsOnTheModel) {
  Robot robot;
  robot.run(2.5F, kIdle);
  ASSERT_EQ(arm(robot.loop(), true).status, rp::AckStatus::kOk);
  auto out = robot.run(1.0F, kIdle);
  ASSERT_EQ(out.telemetry.state, rp::RobotState::kBalancing);
  EXPECT_TRUE(out.motors_enabled);

  out = robot.run(5.0F, {0.1F, 0.0F, true});
  EXPECT_EQ(out.telemetry.state, rp::RobotState::kBalancing);
  EXPECT_NEAR(robot.plant().state().velocity, 0.1F, 0.03F);
  EXPECT_GT(robot.plant().state().x, 0.2F);
  // Both encoders report forward motion although the right one is wired reversed.
  EXPECT_GT(out.telemetry.left_ticks, 0);
  EXPECT_GT(out.telemetry.right_ticks, 0);
  EXPECT_GT(out.telemetry.left_wheel_velocity, 0.0F);
  EXPECT_GT(out.telemetry.right_wheel_velocity, 0.0F);
}

TEST(ControlLoopTest, LinkLossStopsTheRobotButKeepsBalancing) {
  Robot robot;
  robot.run(2.5F, kIdle);
  arm(robot.loop(), true);
  robot.run(1.0F, kIdle);
  robot.run(3.0F, {0.1F, 0.0F, true});

  const auto out = robot.run(4.0F, {0.1F, 0.0F, /*link_alive=*/false});
  EXPECT_EQ(out.telemetry.state, rp::RobotState::kBalancing);
  EXPECT_TRUE(out.telemetry.faults & rp::fault::kLinkTimeout);
  EXPECT_NEAR(robot.plant().state().velocity, 0.0F, 0.03F);
}

TEST(ControlLoopTest, DisarmStopsTheMotorsImmediately) {
  Robot robot;
  robot.run(2.5F, kIdle);
  arm(robot.loop(), true);
  robot.run(1.0F, kIdle);

  EXPECT_EQ(arm(robot.loop(), false).status, rp::AckStatus::kOk);
  const auto out = robot.tick(kIdle);
  EXPECT_FALSE(out.motors_enabled);
  EXPECT_FLOAT_EQ(out.left, 0.0F);
  EXPECT_EQ(out.telemetry.state, rp::RobotState::kDisarmed);
  EXPECT_TRUE(out.state_changed);
}

TEST(ControlLoopTest, ValidatesGainUpdates) {
  Robot robot;
  EXPECT_EQ(robot.loop().handle(rp::SetGains{rp::GainLoop::kPitch, 20.0F, 1.0F, 0.1F}, 1).status,
            rp::AckStatus::kOk);
  EXPECT_EQ(robot.loop().handle(rp::SetGains{rp::GainLoop::kPitch, -1.0F, 0.0F, 0.0F}, 2).status,
            rp::AckStatus::kInvalid);
  EXPECT_EQ(robot.loop()
                .handle(rp::SetGains{rp::GainLoop::kYawRate, std::nanf(""), 0.0F, 0.0F}, 3)
                .status,
            rp::AckStatus::kInvalid);
  EXPECT_EQ(robot.loop().handle(rp::CmdVel{}, 4).status, rp::AckStatus::kInvalid);
}

TEST(ControlLoopTest, ImuLossWhileBalancingLatchesFaultAndCutsMotors) {
  Robot robot;
  robot.run(2.5F, kIdle);
  arm(robot.loop(), true);
  robot.run(1.0F, kIdle);

  robot::app::SensorFrame empty;
  empty.time_us = 99999999;
  const auto out = robot.loop().step(empty, kIdle);
  EXPECT_EQ(out.telemetry.state, rp::RobotState::kFault);
  EXPECT_TRUE(out.telemetry.faults & rp::fault::kImuFailure);
  EXPECT_FALSE(out.motors_enabled);
}

TEST(ControlLoopTest, ReportsLoopOverrun) {
  Robot robot;
  robot.loop().reportTickDuration(6000);  // longer than the 5 ms period
  const auto out = robot.tick(kIdle);
  EXPECT_TRUE(out.telemetry.faults & rp::fault::kLoopOverrun);
  EXPECT_EQ(out.telemetry.max_loop_time_us, 6000);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
