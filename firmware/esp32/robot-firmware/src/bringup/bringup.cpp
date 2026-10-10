// Hardware bring-up firmware (PlatformIO env "bringup").
//
// An interactive tool over telnet to determine the values marked as placeholders in
// config/robot_params.yaml: which TB6612 channel and encoder belong to which wheel, their
// directions, and how each IMU is mounted. It talks to channels and connectors directly, ignoring
// the current wiring parameters.
//
// SAFETY: put the robot on a stand with both wheels off the ground before running motor tests.

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

#include <array>
#include <cmath>
#include <cstdint>

#include "board.hpp"
#include "credentials.hpp"
#include "drivers/counter_extender.hpp"
#include "drivers/mpu6xxx.hpp"
#include "drivers/tb6612.hpp"
#include "hal_arduino/hal_arduino.hpp"
#include "logging/logger.hpp"
#include "pins.hpp"
#include "telemetry/telnet_sink.hpp"

namespace {

namespace board = robot::board;
namespace drivers = robot::drivers;
namespace hal_arduino = robot::hal_arduino;
namespace pins = robot::pins;

constexpr float kTestCommand = 0.3F;  // fraction of the maximum duty
constexpr std::uint32_t kMotorRunMs = 1000;
constexpr std::uint32_t kPollMs = 10;

robot::logging::Logger g_log;
robot::telemetry::TelnetSink g_telnet{pins::kTelnetPort};
hal_arduino::ArduinoClock g_clock;
hal_arduino::WireI2cBus g_i2c{Wire};

drivers::Mpu6xxx g_mpu9250{g_i2c, g_clock, pins::kMpu9250Address, drivers::MpuVariant::kMpu9250};
drivers::Mpu6xxx g_mpu6050{g_i2c, g_clock, pins::kMpu6050Address, drivers::MpuVariant::kMpu6050};

hal_arduino::GpioOutput g_standby{pins::kMotorStandby};
hal_arduino::LedcPwmOutput g_pwm_a{board::kChannelA.pwm, board::kChannelA.ledc_channel};
hal_arduino::GpioOutput g_in1_a{board::kChannelA.in1};
hal_arduino::GpioOutput g_in2_a{board::kChannelA.in2};
hal_arduino::LedcPwmOutput g_pwm_b{board::kChannelB.pwm, board::kChannelB.ledc_channel};
hal_arduino::GpioOutput g_in1_b{board::kChannelB.in1};
hal_arduino::GpioOutput g_in2_b{board::kChannelB.in2};
drivers::Tb6612 g_driver{g_standby};
drivers::HBridgeChannel g_channel_a{g_pwm_a, g_in1_a, g_in2_a, false, board::kMaxMotorDuty};
drivers::HBridgeChannel g_channel_b{g_pwm_b, g_in1_b, g_in2_b, false, board::kMaxMotorDuty};

hal_arduino::PcntQuadratureCounter g_pcnt_j5{PCNT_UNIT_0, board::kEncoderJ5.a, board::kEncoderJ5.b};
hal_arduino::PcntQuadratureCounter g_pcnt_j6{PCNT_UNIT_1, board::kEncoderJ6.a, board::kEncoderJ6.b};
drivers::CounterExtender g_count_j5{hal_arduino::PcntQuadratureCounter::kLimit};
drivers::CounterExtender g_count_j6{hal_arduino::PcntQuadratureCounter::kLimit};

/// Waits while keeping telnet and the encoder counts up to date.
void waitMs(std::uint32_t duration_ms) {
  const std::uint32_t start = millis();
  while (millis() - start < duration_ms) {
    g_telnet.poll();
    g_count_j5.update(g_pcnt_j5.rawCount());
    g_count_j6.update(g_pcnt_j6.rawCount());
    delay(kPollMs);
  }
}

void stopMotors() {
  g_channel_a.coast();
  g_channel_b.coast();
  g_driver.disable();
}

void printHelp() {
  g_log.info(
      "commands: i = IMU report, a / b = test motor channel A / B, "
      "e = watch encoders 20 s, s = stop motors, h = help");
  g_log.info("motor tests spin the wheels: put the robot on a stand first");
}

const char* largestAxis(const drivers::Vector3& v) {
  const float ax = std::fabs(v.x);
  const float ay = std::fabs(v.y);
  const float az = std::fabs(v.z);
  if (ax >= ay && ax >= az) {
    return v.x > 0.0F ? "+x" : "-x";
  }
  if (ay >= az) {
    return v.y > 0.0F ? "+y" : "-y";
  }
  return v.z > 0.0F ? "+z" : "-z";
}

void reportImu(drivers::Mpu6xxx& imu, const drivers::AxisMap& mounting, const char* name) {
  const drivers::MpuStatus status = imu.begin();
  if (status != drivers::MpuStatus::kOk) {
    g_log.error("%s at 0x%02X: %s (WHO_AM_I 0x%02X)", name, imu.address(),
                status == drivers::MpuStatus::kBusError ? "no answer" : "unexpected device",
                imu.whoAmI());
    return;
  }
  g_log.info("%s at 0x%02X ok (WHO_AM_I 0x%02X)", name, imu.address(), imu.whoAmI());
  for (int i = 0; i < 5; ++i) {
    const auto reading = imu.read();
    if (!reading) {
      g_log.error("%s read failed", name);
      return;
    }
    const drivers::Vector3 body = mounting.apply(reading->accel);
    g_log.info(
        "%s accel sensor [%6.2f %6.2f %6.2f] body [%6.2f %6.2f %6.2f] m/s2, gyro [%5.2f %5.2f "
        "%5.2f] rad/s, %.1f C",
        name, reading->accel.x, reading->accel.y, reading->accel.z, body.x, body.y, body.z,
        reading->gyro.x, reading->gyro.y, reading->gyro.z, reading->temperature);
    if (i == 4) {
      g_log.info(
          "%s: 'up' is sensor %s; with the robot upright the body accel must read about "
          "[0 0 +9.8]",
          name, largestAxis(reading->accel));
    }
    waitMs(200);
  }
}

void testChannel(drivers::HBridgeChannel& channel, const char* name) {
  g_driver.enable();
  for (const float command : {kTestCommand, -kTestCommand}) {
    const std::int32_t j5_before = g_count_j5.total();
    const std::int32_t j6_before = g_count_j6.total();
    channel.set(command);
    waitMs(kMotorRunMs);
    channel.coast();
    waitMs(500);  // let the wheel spin down so the counts settle
    g_log.info("channel %s %s: J5 %+ld counts, J6 %+ld counts", name,
               command > 0.0F ? "positive" : "negative",
               static_cast<long>(g_count_j5.total() - j5_before),
               static_cast<long>(g_count_j6.total() - j6_before));
  }
  stopMotors();
  g_log.info("note which wheel turned and whether 'positive' moved it forwards");
}

void watchEncoders() {
  g_log.info("turn a wheel by hand; counting for 20 s");
  const std::int32_t j5_start = g_count_j5.total();
  const std::int32_t j6_start = g_count_j6.total();
  for (int i = 0; i < 40; ++i) {
    waitMs(500);
    g_log.info("J5 %+ld  J6 %+ld", static_cast<long>(g_count_j5.total() - j5_start),
               static_cast<long>(g_count_j6.total() - j6_start));
  }
  g_log.info("counts per wheel revolution = counts / turns (x4 decoding: 4 * PPR * gear ratio)");
}

void handleCommand(int command) {
  switch (command) {
    case 'i':
      reportImu(g_mpu9250, board::kMpu9250Mounting, "MPU-9250");
      reportImu(g_mpu6050, board::kMpu6050Mounting, "MPU-6050");
      break;
    case 'a':
      testChannel(g_channel_a, "A");
      break;
    case 'b':
      testChannel(g_channel_b, "B");
      break;
    case 'e':
      watchEncoders();
      break;
    case 's':
      stopMotors();
      g_log.info("motors stopped");
      break;
    case 'h':
      printHelp();
      break;
    default:
      break;  // ignore newlines and telnet negotiation bytes
  }
}

}  // namespace

void setup() {
  g_log.addSink(g_telnet);
  robot::telemetry::startDebugAccessPoint(robot::secrets::kDebugApSsid,
                                          robot::secrets::kDebugApPassword);
  g_telnet.begin();
  Wire.begin(pins::kI2cSda, pins::kI2cScl, pins::kI2cClockHz);

  g_standby.begin();
  for (auto* output : {&g_in1_a, &g_in2_a, &g_in1_b, &g_in2_b}) {
    output->begin();
  }
  g_pwm_a.begin();
  g_pwm_b.begin();
  stopMotors();

  if (!g_pcnt_j5.begin() || !g_pcnt_j6.begin()) {
    g_log.error("PCNT configuration failed");
  }
  g_count_j5.reset(g_pcnt_j5.rawCount());
  g_count_j6.reset(g_pcnt_j6.rawCount());
}

void loop() {
  static bool greeted = false;
  waitMs(kPollMs);
  const int command = g_telnet.read();
  if (command < 0) {
    return;
  }
  if (!greeted) {
    g_log.info("robot bring-up firmware, max motor duty %.2f", board::kMaxMotorDuty);
    printHelp();
    greeted = true;
  }
  handleCommand(command);
}
