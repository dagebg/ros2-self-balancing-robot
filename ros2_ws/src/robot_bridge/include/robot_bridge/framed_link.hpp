#pragma once

/// @file
/// Protocol frames over the serial port.

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "robot_bridge/serial_port.hpp"
#include "robot_protocol/codec.hpp"
#include "robot_protocol/frame_parser.hpp"

namespace robot_bridge {

/// Encodes outgoing messages with increasing sequence numbers and decodes incoming frames.
class FramedLink {
 public:
  explicit FramedLink(SerialPort& port) noexcept : port_(port) {}

  /// Sends a message; returns the sequence number used, or nothing if the frame did not fit
  /// into the port. Throws std::system_error if the port fails.
  std::optional<std::uint8_t> send(const robot::protocol::Message& message);

  /// Reads and decodes everything pending. Throws std::system_error if the port fails.
  std::vector<robot::protocol::DecodeResult> receive();

  /// Forgets any partial frame, e.g. after reconnecting.
  void reset() noexcept { parser_.reset(); }

  [[nodiscard]] const robot::protocol::FrameParser::Stats& stats() const noexcept {
    return parser_.stats();
  }
  [[nodiscard]] std::uint32_t lost() const noexcept { return sequence_tracker_.lost(); }

 private:
  SerialPort& port_;
  robot::protocol::FrameParser parser_;
  robot::protocol::SequenceTracker sequence_tracker_;
  std::uint8_t next_sequence_ = 0;
};

}  // namespace robot_bridge
