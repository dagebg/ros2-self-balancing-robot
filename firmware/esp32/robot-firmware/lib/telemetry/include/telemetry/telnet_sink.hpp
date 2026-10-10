#pragma once

/// @file
/// Debug access over WiFi: a SoftAP and a telnet log sink.
///
/// UART0 is reserved for the Raspberry Pi link, so this is the firmware's human-readable
/// log channel. Connect to the access point, then `telnet 192.168.4.1 23`.

#include <WiFi.h>

#include <cstdint>
#include <mutex>

#include "logging/logger.hpp"

namespace robot::telemetry {

/// Opens the ESP32's WiFi access point used for debug access.
/// @return false if the access point could not be started.
bool startDebugAccessPoint(const char* ssid, const char* password);

/// Log sink that streams messages to a single telnet client.
///
/// A new connection replaces a client that has gone away; while a client is connected,
/// further connection attempts are rejected. Thread-safe.
class TelnetSink final : public logging::Sink {
 public:
  explicit TelnetSink(std::uint16_t port) : server_(port) {}

  /// Starts listening. Call after the access point is up.
  void begin();

  /// Accepts pending connections. Call periodically from one task.
  void poll();

  void write(logging::Level level, std::string_view message) override;

  /// Next byte typed by the telnet client, or -1 if none is pending.
  [[nodiscard]] int read();

 private:
  [[nodiscard]] bool clientConnected();  // requires mutex_ held

  WiFiServer server_;
  WiFiClient client_;
  std::mutex mutex_;
};

}  // namespace robot::telemetry
