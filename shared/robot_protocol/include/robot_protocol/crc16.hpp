#pragma once

/// @file
/// CRC-16/CCITT-FALSE checksum used to protect every protocol packet.

#include <cstddef>
#include <cstdint>

namespace robot::protocol {

/// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection, no final XOR.
/// Check value: `crc16("123456789") == 0x29B1`.
[[nodiscard]] std::uint16_t crc16(const std::uint8_t* data, std::size_t size) noexcept;

}  // namespace robot::protocol
