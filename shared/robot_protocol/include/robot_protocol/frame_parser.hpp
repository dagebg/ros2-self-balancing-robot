#pragma once

/// @file
/// Incremental frame extraction from a byte stream (UART, serial port).

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "robot_protocol/codec.hpp"

namespace robot::protocol {

/// Splits a byte stream into frames at 0x00 delimiters and decodes them.
///
/// Garbage (boot-ROM text, line noise, partial frames) produces at most one error result and
/// the parser resynchronizes at the next delimiter. Empty frames (consecutive delimiters) are
/// ignored, so a sender may emit a lone 0x00 to force resynchronization.
class FrameParser {
 public:
  /// Counters for link diagnostics.
  struct Stats {
    std::uint32_t frames_ok = 0;
    std::uint32_t cobs_errors = 0;
    std::uint32_t crc_errors = 0;
    std::uint32_t overflows = 0;
    std::uint32_t other_errors = 0;  ///< Version, type, length and value errors.
  };

  /// Consumes one byte.
  /// @return A decode result when a delimiter completes a non-empty frame, otherwise nullopt.
  [[nodiscard]] std::optional<DecodeResult> push(std::uint8_t byte) noexcept;

  [[nodiscard]] const Stats& stats() const noexcept { return stats_; }

  /// Drops any partial frame. Statistics are kept.
  void reset() noexcept;

 private:
  void count(DecodeError error) noexcept;

  std::array<std::uint8_t, kMaxFrameSize - 1> buffer_{};
  std::size_t length_ = 0;
  bool overflow_ = false;
  Stats stats_{};
};

/// Counts messages lost on the link from gaps in the 8-bit sequence numbers.
class SequenceTracker {
 public:
  /// Records a received sequence number. Returns the number of messages missed before it.
  std::uint8_t observe(std::uint8_t sequence) noexcept;

  [[nodiscard]] std::uint32_t lost() const noexcept { return lost_; }
  [[nodiscard]] std::uint32_t received() const noexcept { return received_; }

 private:
  std::uint8_t last_ = 0;
  bool has_last_ = false;
  std::uint32_t lost_ = 0;
  std::uint32_t received_ = 0;
};

}  // namespace robot::protocol
