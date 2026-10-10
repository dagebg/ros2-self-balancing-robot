#include "telemetry/telnet_sink.hpp"

#include <string_view>

namespace robot::telemetry {

namespace {
constexpr std::string_view kGreeting = "robot firmware log stream\r\n";
constexpr std::string_view kLineEnd = "\r\n";
}  // namespace

bool startDebugAccessPoint(const char* ssid, const char* password) {
  return WiFi.softAP(ssid, password);
}

void TelnetSink::begin() {
  server_.begin();
  server_.setNoDelay(true);
}

bool TelnetSink::clientConnected() { return client_.connected() != 0; }

void TelnetSink::poll() {
  if (!server_.hasClient()) {
    return;
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  if (clientConnected()) {
    server_.available().stop();  // one client at a time
    return;
  }
  client_.stop();
  client_ = server_.available();
  client_.write(kGreeting.data(), kGreeting.size());
}

void TelnetSink::write(logging::Level level, std::string_view message) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!clientConnected()) {
    return;
  }
  client_.write(logging::toString(level).data(), logging::toString(level).size());
  client_.write(' ');
  client_.write(message.data(), message.size());
  client_.write(kLineEnd.data(), kLineEnd.size());
}

}  // namespace robot::telemetry
