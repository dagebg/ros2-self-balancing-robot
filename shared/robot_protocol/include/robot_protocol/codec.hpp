#pragma once

/// @file
/// Encoding and decoding of complete frames.
///
/// Wire format of one frame:
///
///     COBS( version:u8 | type:u8 | sequence:u8 | body | crc16:u16le ) | 0x00
///
/// The CRC covers version, type, sequence and body. All multi-byte values are little-endian.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "robot_protocol/cobs.hpp"
#include "robot_protocol/messages.hpp"

namespace robot::protocol {

inline constexpr std::size_t kHeaderSize = 3;
inline constexpr std::size_t kCrcSize = 2;
/// Largest message body (LogMessage: level + text).
inline constexpr std::size_t kMaxBodySize = 1 + kMaxLogTextLength;
inline constexpr std::size_t kMaxPacketSize = kHeaderSize + kMaxBodySize + kCrcSize;
/// Largest frame on the wire, including the trailing 0x00 delimiter.
inline constexpr std::size_t kMaxFrameSize = cobsMaxEncodedSize(kMaxPacketSize) + 1;
inline constexpr std::uint8_t kFrameDelimiter = 0x00;

enum class DecodeError : std::uint8_t {
  kNone = 0,
  kCobs,                ///< Invalid COBS encoding.
  kTooShort,            ///< Shorter than header + CRC.
  kCrcMismatch,         ///< Corrupted in transit.
  kUnsupportedVersion,  ///< Sent by an incompatible protocol version.
  kUnknownType,         ///< Unknown message type identifier.
  kBadLength,           ///< Body length does not match the message type.
  kBadValue,            ///< A field is outside its valid range (e.g. unknown enum value).
  kOverflow,            ///< No delimiter within kMaxFrameSize bytes (stream parser only).
};

[[nodiscard]] std::string_view toString(DecodeError error) noexcept;

/// Outcome of decoding one frame. `message` is only meaningful when ok().
struct DecodeResult {
  DecodeError error = DecodeError::kNone;
  std::uint8_t sequence = 0;
  Message message;

  [[nodiscard]] bool ok() const noexcept { return error == DecodeError::kNone; }
};

/// Encodes a message into a complete frame, including the trailing delimiter.
/// @return Frame length, or `std::nullopt` if `capacity` is too small (kMaxFrameSize always fits).
[[nodiscard]] std::optional<std::size_t> encodeFrame(const Message& message, std::uint8_t sequence,
                                                     std::uint8_t* out,
                                                     std::size_t capacity) noexcept;

/// Decodes one frame given *without* its trailing delimiter.
[[nodiscard]] DecodeResult decodeFrame(const std::uint8_t* frame, std::size_t size) noexcept;

}  // namespace robot::protocol
