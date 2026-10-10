// Composition root of the robot firmware: creates the long-lived objects and wires them up.

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

#include <array>
#include <cstdio>

#include "credentials.hpp"
#include "diagnostics/i2c_scan.hpp"
#include "logging/logger.hpp"
#include "pins.hpp"
#include "robot_config.hpp"
#include "telemetry/telnet_sink.hpp"

namespace {

namespace pins = robot::pins;

constexpr std::uint32_t kI2cScanIntervalMs = 5000;
constexpr std::uint32_t kLoopIdleMs = 10;

robot::logging::Logger g_log;
robot::telemetry::TelnetSink g_telnet{pins::kTelnetPort};

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

  if (!result.contains(pins::kMpu9250Address)) {
    g_log.warn("MPU-9250 missing at 0x%02X", pins::kMpu9250Address);
  }
  if (!result.contains(pins::kMpu6050Address)) {
    g_log.warn("MPU-6050 missing at 0x%02X", pins::kMpu6050Address);
  }
}

}  // namespace

void setup() {
  g_log.addSink(g_telnet);

  const bool ap_up = robot::telemetry::startDebugAccessPoint(robot::secrets::kDebugApSsid,
                                                             robot::secrets::kDebugApPassword);
  g_telnet.begin();
  Wire.begin(pins::kI2cSda, pins::kI2cScl, pins::kI2cClockHz);

  const IPAddress ip = WiFi.softAPIP();
  g_log.info("robot firmware ready, debug AP %s, telnet %u.%u.%u.%u:%u", ap_up ? "up" : "FAILED",
             ip[0], ip[1], ip[2], ip[3], pins::kTelnetPort);
  if (!robot::config::kAllParamsMeasured) {
    g_log.warn("config/robot_params.yaml still contains placeholder values");
  }
  logI2cScan();
}

void loop() {
  static std::uint32_t last_scan_ms = 0;

  g_telnet.poll();
  if (millis() - last_scan_ms >= kI2cScanIntervalMs) {
    last_scan_ms = millis();
    logI2cScan();
  }
  delay(kLoopIdleMs);
}
