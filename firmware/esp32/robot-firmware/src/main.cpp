// Composition root of the robot firmware: creates the long-lived objects, wires them together and
// starts the tasks. All decisions live in the portable libraries; this file only connects them to
// the hardware and to FreeRTOS.
//
//   control task (core 1, high priority, 200 Hz): sensors -> ControlLoop -> motors
//   comms task   (core 0, medium priority)       : UART <-> protocol, telemetry, acks, logs
//   Arduino loop (core 1, lowest priority)       : telnet, state-change logging, statistics

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#include <array>
#include <cstdio>
#include <optional>

#include "app/control_loop.hpp"
#include "app/link.hpp"
#include "app/mailbox.hpp"
#include "board.hpp"
#include "credentials.hpp"
#include "diagnostics/i2c_scan.hpp"
#include "drivers/counter_extender.hpp"
#include "drivers/mpu6xxx.hpp"
#include "drivers/tb6612.hpp"
#include "hal_arduino/hal_arduino.hpp"
#include "logging/logger.hpp"
#include "pins.hpp"
#include "robot_config.hpp"
#include "robot_protocol/messages.hpp"
#include "telemetry/telnet_sink.hpp"

namespace {

namespace app = robot::app;
namespace board = robot::board;
namespace config = robot::config;
namespace drivers = robot::drivers;
namespace hal_arduino = robot::hal_arduino;
namespace pins = robot::pins;
namespace protocol = robot::protocol;

constexpr std::uint32_t kControlPeriodMs = 1000 / config::kLoopRate;
constexpr std::uint32_t kTelemetryPeriodUs = 1'000'000 / config::kTelemetryRate;
constexpr std::uint32_t kLinkTimeoutUs = 500'000;
constexpr std::uint32_t kCommsPeriodMs = 2;
constexpr std::uint32_t kWatchdogTimeoutS = 1;
constexpr std::uint32_t kStatsPeriodMs = 10'000;
constexpr std::uint32_t kControlStackBytes = 6144;
constexpr std::uint32_t kCommsStackBytes = 4096;
constexpr UBaseType_t kControlPriority = configMAX_PRIORITIES - 2;
constexpr UBaseType_t kCommsPriority = 5;
constexpr BaseType_t kControlCore = 1;
constexpr BaseType_t kCommsCore = 0;  // shares core 0 with WiFi; the control loop has core 1

// --- Hardware
// -------------------------------------------------------------------------------------

hal_arduino::ArduinoClock g_clock;
hal_arduino::WireI2cBus g_i2c{Wire};
hal_arduino::SerialByteStream g_uart{Serial};

drivers::Mpu6xxx g_mpu9250{g_i2c, g_clock, pins::kMpu9250Address, drivers::MpuVariant::kMpu9250};
drivers::Mpu6xxx g_mpu6050{g_i2c, g_clock, pins::kMpu6050Address, drivers::MpuVariant::kMpu6050};
bool g_mpu9250_ok = false;
bool g_mpu6050_ok = false;

hal_arduino::GpioOutput g_standby{pins::kMotorStandby};
hal_arduino::LedcPwmOutput g_left_pwm{board::kLeftMotor.pwm, board::kLeftMotor.ledc_channel};
hal_arduino::GpioOutput g_left_in1{board::kLeftMotor.in1};
hal_arduino::GpioOutput g_left_in2{board::kLeftMotor.in2};
hal_arduino::LedcPwmOutput g_right_pwm{board::kRightMotor.pwm, board::kRightMotor.ledc_channel};
hal_arduino::GpioOutput g_right_in1{board::kRightMotor.in1};
hal_arduino::GpioOutput g_right_in2{board::kRightMotor.in2};
drivers::Tb6612 g_driver{g_standby};
drivers::HBridgeChannel g_left_motor{g_left_pwm, g_left_in1, g_left_in2, board::kLeftMotorInverted,
                                     board::kMaxMotorDuty};
drivers::HBridgeChannel g_right_motor{g_right_pwm, g_right_in1, g_right_in2,
                                      board::kRightMotorInverted, board::kMaxMotorDuty};

hal_arduino::PcntQuadratureCounter g_left_pcnt{PCNT_UNIT_0, board::kLeftEncoder.a,
                                               board::kLeftEncoder.b};
hal_arduino::PcntQuadratureCounter g_right_pcnt{PCNT_UNIT_1, board::kRightEncoder.a,
                                                board::kRightEncoder.b};
drivers::CounterExtender g_left_count{hal_arduino::PcntQuadratureCounter::kLimit};
drivers::CounterExtender g_right_count{hal_arduino::PcntQuadratureCounter::kLimit};

// --- Software
// -------------------------------------------------------------------------------------

app::ControlConfig makeControlConfig() {
  app::ControlConfig control;
  control.primary_mounting = board::kMpu9250Mounting;
  control.secondary_mounting = board::kMpu6050Mounting;
  control.geometry = {config::kWheelRadius, config::kTrackWidth};
  control.counts_per_wheel_rev = board::kCountsPerWheelRev;
  // The encoder direction is handled in ControlLoop so that raw counts stay visible in tests.
  control.left_encoder_inverted = board::kLeftEncoderInverted;
  control.right_encoder_inverted = board::kRightEncoderInverted;
  control.nominal_period = static_cast<float>(kControlPeriodMs) * 1e-3F;
  return control;
}

robot::logging::Logger g_log;
robot::telemetry::TelnetSink g_telnet{pins::kTelnetPort};
app::LogQueue g_log_queue;
app::ProtocolLogSink g_protocol_log{g_log_queue};

app::ControlLoop g_control{makeControlConfig()};
app::Mailbox<app::MotionTarget> g_motion;
app::RequestQueue g_requests;
app::AckQueue g_acks;
app::Mailbox<app::TelemetrySnapshot> g_telemetry;
app::Link g_link{g_uart};
app::CommsService g_comms{
    g_link, {g_motion, g_requests, g_acks, g_telemetry, g_log_queue}, kTelemetryPeriodUs};

// --- Tasks
// ----------------------------------------------------------------------------------------

void applyMotors(const app::ControlOutput& out) {
  if (out.motors_enabled) {
    g_driver.enable();
    g_left_motor.set(out.left);
    g_right_motor.set(out.right);
  } else {
    g_left_motor.coast();
    g_right_motor.coast();
    g_driver.disable();
  }
}

std::optional<drivers::ImuReading> readImu(drivers::Mpu6xxx& imu, bool ok) {
  return ok ? imu.read() : std::nullopt;
}

/// Hard real-time path. Never logs or touches WiFi: anything that could block stays elsewhere.
[[noreturn]] void controlTask(void* /*unused*/) {
  esp_task_wdt_add(nullptr);
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kControlPeriodMs));
    const std::uint32_t start = micros();

    app::SensorFrame frame;
    frame.time_us = start;
    frame.primary = readImu(g_mpu9250, g_mpu9250_ok);
    frame.secondary = readImu(g_mpu6050, g_mpu6050_ok);
    frame.left_ticks = g_left_count.update(g_left_pcnt.rawCount());
    frame.right_ticks = g_right_count.update(g_right_pcnt.rawCount());

    while (const auto request = g_requests.pop()) {
      g_acks.push(g_control.handle(request->message, request->sequence));
    }

    const app::MotionTarget target = g_motion.read();
    const bool link_alive = target.received_any && start - target.last_rx_us < kLinkTimeoutUs;
    const app::ControlOutput out =
        g_control.step(frame, {target.linear, target.angular, link_alive});
    applyMotors(out);
    g_telemetry.publish({out.telemetry, out.primary_sample, out.secondary_sample});

    g_control.reportTickDuration(micros() - start);
    esp_task_wdt_reset();
  }
}

[[noreturn]] void commsTask(void* /*unused*/) {
  for (;;) {
    g_comms.poll(micros());
    vTaskDelay(pdMS_TO_TICKS(kCommsPeriodMs));
  }
}

// --- Start-up helpers
// -----------------------------------------------------------------------------

const char* stateName(protocol::RobotState state) {
  switch (state) {
    case protocol::RobotState::kDisarmed:
      return "disarmed";
    case protocol::RobotState::kArmed:
      return "armed, waiting to be upright";
    case protocol::RobotState::kBalancing:
      return "balancing";
    case protocol::RobotState::kFault:
      return "FAULT";
  }
  return "?";
}

bool beginImu(drivers::Mpu6xxx& imu, const char* name) {
  const drivers::MpuStatus status = imu.begin();
  if (status == drivers::MpuStatus::kOk) {
    g_log.info("%s ready (WHO_AM_I 0x%02X)", name, imu.whoAmI());
    return true;
  }
  g_log.error("%s at 0x%02X: %s (WHO_AM_I 0x%02X)", name, imu.address(),
              status == drivers::MpuStatus::kBusError ? "no answer" : "unexpected device",
              imu.whoAmI());
  return false;
}

void logI2cScan() {
  const auto result = robot::diagnostics::scanI2c([](std::uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
  });
  std::array<char, robot::logging::Logger::kMaxMessageLength> line{};
  int used = std::snprintf(line.data(), line.size(), "I2C scan: %u device(s)",
                           static_cast<unsigned>(result.count()));
  for (std::size_t i = 0; i < result.storedCount() && used > 0; ++i) {
    const auto offset = static_cast<std::size_t>(used);
    if (offset >= line.size()) {
      break;
    }
    used += std::snprintf(line.data() + offset, line.size() - offset, " 0x%02X", result.at(i));
  }
  g_log.info("%s", line.data());
}

void beginMotors() {
  g_standby.begin();
  for (auto* output : {&g_left_in1, &g_left_in2, &g_right_in1, &g_right_in2}) {
    output->begin();
  }
  g_left_pwm.begin();
  g_right_pwm.begin();
  g_left_motor.coast();
  g_right_motor.coast();
  g_driver.disable();
}

void beginEncoders() {
  if (!g_left_pcnt.begin() || !g_right_pcnt.begin()) {
    g_log.error("PCNT configuration failed");
  }
  g_left_count.reset(g_left_pcnt.rawCount());
  g_right_count.reset(g_right_pcnt.rawCount());
}

}  // namespace

void setup() {
  beginMotors();  // first: guarantee the motors are off

  g_log.addSink(g_telnet);
  g_log.addSink(g_protocol_log);
  const bool ap_up = robot::telemetry::startDebugAccessPoint(robot::secrets::kDebugApSsid,
                                                             robot::secrets::kDebugApPassword);
  g_telnet.begin();

  // UART0 belongs to the Pi link from here on; the delimiter discards boot-ROM output.
  g_uart.begin(static_cast<std::uint32_t>(config::kUartBaud));
  g_link.sendResync();

  Wire.begin(pins::kI2cSda, pins::kI2cScl, pins::kI2cClockHz);
  logI2cScan();
  g_mpu9250_ok = beginImu(g_mpu9250, "MPU-9250");
  g_mpu6050_ok = beginImu(g_mpu6050, "MPU-6050");
  beginEncoders();

  const IPAddress ip = WiFi.softAPIP();
  g_log.info("robot firmware ready (protocol v%u), debug AP %s, telnet %u.%u.%u.%u:%u",
             protocol::kProtocolVersion, ap_up ? "up" : "FAILED", ip[0], ip[1], ip[2], ip[3],
             pins::kTelnetPort);
  if (!config::kAllParamsMeasured) {
    g_log.warn("config/robot_params.yaml still contains placeholder values");
  }

  esp_task_wdt_init(kWatchdogTimeoutS, true);
  xTaskCreatePinnedToCore(controlTask, "control", kControlStackBytes, nullptr, kControlPriority,
                          nullptr, kControlCore);
  xTaskCreatePinnedToCore(commsTask, "comms", kCommsStackBytes, nullptr, kCommsPriority, nullptr,
                          kCommsCore);
}

void loop() {
  static protocol::RobotState last_state = protocol::RobotState::kDisarmed;
  static std::uint16_t last_faults = 0;
  static std::uint32_t last_stats_ms = 0;

  g_telnet.poll();

  const protocol::Telemetry telemetry = g_telemetry.read().telemetry;
  if (telemetry.state != last_state) {
    g_log.info("state: %s", stateName(telemetry.state));
    last_state = telemetry.state;
  }
  if (telemetry.faults != last_faults) {
    g_log.warn("faults: 0x%04X", telemetry.faults);
    last_faults = telemetry.faults;
  }
  if (millis() - last_stats_ms >= kStatsPeriodMs) {
    last_stats_ms = millis();
    const auto& rx = g_link.rxStats();
    g_log.info("link: %lu ok, %lu crc, %lu cobs, %lu other, %lu lost; max loop %u us",
               static_cast<unsigned long>(rx.frames_ok), static_cast<unsigned long>(rx.crc_errors),
               static_cast<unsigned long>(rx.cobs_errors),
               static_cast<unsigned long>(rx.other_errors),
               static_cast<unsigned long>(g_link.rxLost()), telemetry.max_loop_time_us);
  }
  delay(20);
}
