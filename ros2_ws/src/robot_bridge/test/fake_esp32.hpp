#pragma once

// A pseudo-terminal pair: the bridge opens the slave side like a real serial device, the test
// plays the ESP32 on the master side with the same protocol library.

#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_protocol/codec.hpp"
#include "robot_protocol/frame_parser.hpp"

namespace robot_bridge::testing {

class FakeEsp32 {
 public:
  FakeEsp32() {
    std::array<char, 256> name{};
    if (openpty(&master_, &slave_, name.data(), nullptr, nullptr) != 0) {
      throw std::runtime_error("openpty failed");
    }
    termios raw{};
    tcgetattr(slave_, &raw);
    cfmakeraw(&raw);  // no echo or line editing before the bridge configures the port
    tcsetattr(slave_, TCSANOW, &raw);
    fcntl(master_, F_SETFL, fcntl(master_, F_GETFL) | O_NONBLOCK);
    slave_path_ = name.data();
  }

  ~FakeEsp32() {
    close(master_);
    close(slave_);  // kept open until now so the master never sees a hang-up
  }

  FakeEsp32(const FakeEsp32&) = delete;
  FakeEsp32& operator=(const FakeEsp32&) = delete;
  FakeEsp32(FakeEsp32&&) = delete;
  FakeEsp32& operator=(FakeEsp32&&) = delete;

  [[nodiscard]] const std::string& path() const { return slave_path_; }
  [[nodiscard]] int masterFd() const { return master_; }

  void send(const robot::protocol::Message& message, std::uint8_t sequence = 0) {
    std::array<std::uint8_t, robot::protocol::kMaxFrameSize> frame{};
    const auto size = robot::protocol::encodeFrame(message, sequence, frame.data(), frame.size());
    if (!size || write(master_, frame.data(), *size) != static_cast<ssize_t>(*size)) {
      throw std::runtime_error("fake ESP32 could not write");
    }
  }

  /// Everything the bridge has sent since the last call.
  std::vector<robot::protocol::DecodeResult> receive() {
    std::vector<robot::protocol::DecodeResult> frames;
    std::array<std::uint8_t, 256> buffer{};
    ssize_t count = 0;
    while ((count = read(master_, buffer.data(), buffer.size())) > 0) {
      for (ssize_t i = 0; i < count; ++i) {
        if (auto result = parser_.push(buffer[static_cast<std::size_t>(i)]);
            result && result->ok()) {
          frames.push_back(*result);
        }
      }
    }
    return frames;
  }

 private:
  int master_ = -1;
  int slave_ = -1;
  std::string slave_path_;
  robot::protocol::FrameParser parser_;
};

}  // namespace robot_bridge::testing
