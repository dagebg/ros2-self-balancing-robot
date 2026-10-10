#pragma once

/// @file
/// Consistent Overhead Byte Stuffing (COBS).
///
/// COBS removes every `0x00` from a packet so that `0x00` can delimit frames on a byte stream.
/// After line noise or a partial frame, the receiver resynchronizes at the next `0x00`.

#include <cstddef>
#include <cstdint>
#include <optional>

namespace robot::protocol {

/// Upper bound of the encoded size of `size` input bytes (excluding the frame delimiter).
[[nodiscard]] constexpr std::size_t cobsMaxEncodedSize(std::size_t size) noexcept {
  return size + (size / 254) + 1;
}

/// Encodes `size` bytes from `input` into `output`. The result contains no `0x00` bytes and no
/// delimiter.
/// @return Encoded length, or `std::nullopt` if `capacity` is too small.
[[nodiscard]] std::optional<std::size_t> cobsEncode(const std::uint8_t* input, std::size_t size,
                                                    std::uint8_t* output,
                                                    std::size_t capacity) noexcept;

/// Decodes one COBS-encoded block (without its delimiter).
/// @return Decoded length, or `std::nullopt` if the input is malformed (contains `0x00`, ends
///         inside a block) or `capacity` is too small.
[[nodiscard]] std::optional<std::size_t> cobsDecode(const std::uint8_t* input, std::size_t size,
                                                    std::uint8_t* output,
                                                    std::size_t capacity) noexcept;

}  // namespace robot::protocol
