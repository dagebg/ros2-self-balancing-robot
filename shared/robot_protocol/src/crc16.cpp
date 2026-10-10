#include "robot_protocol/crc16.hpp"

#include <array>

namespace robot::protocol {

namespace {

constexpr std::uint16_t kPolynomial = 0x1021;
constexpr std::uint16_t kInitialValue = 0xFFFF;

constexpr std::array<std::uint16_t, 256> makeTable() {
  std::array<std::uint16_t, 256> table{};
  for (std::size_t i = 0; i < table.size(); ++i) {
    auto crc = static_cast<std::uint16_t>(i << 8U);
    for (int bit = 0; bit < 8; ++bit) {
      const bool msb_set = (crc & 0x8000U) != 0;
      crc = static_cast<std::uint16_t>(crc << 1U);
      if (msb_set) {
        crc ^= kPolynomial;
      }
    }
    table[i] = crc;
  }
  return table;
}

constexpr std::array<std::uint16_t, 256> kTable = makeTable();

}  // namespace

std::uint16_t crc16(const std::uint8_t* data, std::size_t size) noexcept {
  std::uint16_t crc = kInitialValue;
  for (std::size_t i = 0; i < size; ++i) {
    const auto index = static_cast<std::uint8_t>((crc >> 8U) ^ data[i]);
    crc = static_cast<std::uint16_t>((crc << 8U) ^ kTable[index]);
  }
  return crc;
}

}  // namespace robot::protocol
