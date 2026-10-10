#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string_view>

#include "robot_protocol/crc16.hpp"

namespace {

using robot::protocol::crc16;

std::uint16_t crcOf(std::string_view text) {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  return crc16(reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
}

TEST(Crc16Test, MatchesCcittFalseCheckValue) { EXPECT_EQ(crcOf("123456789"), 0x29B1); }

TEST(Crc16Test, EmptyInputReturnsInitialValue) { EXPECT_EQ(crc16(nullptr, 0), 0xFFFF); }

TEST(Crc16Test, DetectsEverySingleBitFlip) {
  std::array<std::uint8_t, 16> data{};
  for (std::size_t i = 0; i < data.size(); ++i) {
    data[i] = static_cast<std::uint8_t>(i * 37U + 11U);
  }
  const std::uint16_t reference = crc16(data.data(), data.size());

  for (std::size_t byte = 0; byte < data.size(); ++byte) {
    for (unsigned bit = 0; bit < 8; ++bit) {
      auto corrupted = data;
      corrupted[byte] = static_cast<std::uint8_t>(corrupted[byte] ^ (1U << bit));
      EXPECT_NE(crc16(corrupted.data(), corrupted.size()), reference)
          << "byte " << byte << " bit " << bit;
    }
  }
}

}  // namespace
