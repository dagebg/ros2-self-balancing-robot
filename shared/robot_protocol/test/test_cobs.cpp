#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <vector>

#include "robot_protocol/cobs.hpp"

namespace {

using Bytes = std::vector<std::uint8_t>;
using robot::protocol::cobsDecode;
using robot::protocol::cobsEncode;
using robot::protocol::cobsMaxEncodedSize;

Bytes encode(const Bytes& input) {
  Bytes out(cobsMaxEncodedSize(input.size()));
  const auto size = cobsEncode(input.data(), input.size(), out.data(), out.size());
  EXPECT_TRUE(size.has_value());
  out.resize(size.value_or(0));
  return out;
}

Bytes sequence(std::size_t count, std::uint8_t first) {
  Bytes bytes(count);
  for (std::size_t i = 0; i < count; ++i) {
    bytes[i] = static_cast<std::uint8_t>(first + i);
  }
  return bytes;
}

// Reference vectors from the COBS paper / Wikipedia (without the frame delimiter).
TEST(CobsTest, EncodesReferenceVectors) {
  EXPECT_EQ(encode({}), (Bytes{0x01}));
  EXPECT_EQ(encode({0x00}), (Bytes{0x01, 0x01}));
  EXPECT_EQ(encode({0x00, 0x00}), (Bytes{0x01, 0x01, 0x01}));
  EXPECT_EQ(encode({0x00, 0x11, 0x00}), (Bytes{0x01, 0x02, 0x11, 0x01}));
  EXPECT_EQ(encode({0x11, 0x22, 0x00, 0x33}), (Bytes{0x03, 0x11, 0x22, 0x02, 0x33}));
  EXPECT_EQ(encode({0x11, 0x22, 0x33, 0x44}), (Bytes{0x05, 0x11, 0x22, 0x33, 0x44}));
  EXPECT_EQ(encode({0x11, 0x00, 0x00, 0x00}), (Bytes{0x02, 0x11, 0x01, 0x01, 0x01}));
}

TEST(CobsTest, Encodes254NonZeroBytesAsOneFullBlock) {
  const Bytes input = sequence(254, 0x01);  // 01 .. FE
  Bytes expected{0xFF};
  expected.insert(expected.end(), input.begin(), input.end());
  EXPECT_EQ(encode(input), expected);
}

TEST(CobsTest, Encodes255NonZeroBytesAsTwoBlocks) {
  const Bytes input = sequence(255, 0x01);  // 01 .. FF
  Bytes expected{0xFF};
  expected.insert(expected.end(), input.begin(), input.begin() + 254);
  expected.push_back(0x02);
  expected.push_back(0xFF);
  EXPECT_EQ(encode(input), expected);
}

TEST(CobsTest, EncodedOutputNeverContainsZero) {
  std::mt19937 rng(1234);
  std::uniform_int_distribution<int> byte(0, 3);  // many zeros
  for (std::size_t size = 0; size < 600; size += 7) {
    Bytes input(size);
    for (auto& b : input) {
      b = static_cast<std::uint8_t>(byte(rng));
    }
    for (const std::uint8_t b : encode(input)) {
      ASSERT_NE(b, 0);
    }
  }
}

TEST(CobsTest, RoundTripsRandomBuffersAcrossBlockBoundaries) {
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> byte(0, 255);
  for (std::size_t size = 0; size <= 1024; ++size) {
    Bytes input(size);
    for (auto& b : input) {
      b = static_cast<std::uint8_t>(byte(rng));
    }
    const Bytes encoded = encode(input);
    ASSERT_LE(encoded.size(), cobsMaxEncodedSize(size));

    Bytes decoded(size + 1);
    const auto length = cobsDecode(encoded.data(), encoded.size(), decoded.data(), decoded.size());
    ASSERT_TRUE(length.has_value()) << "size " << size;
    decoded.resize(*length);
    ASSERT_EQ(decoded, input) << "size " << size;
  }
}

TEST(CobsTest, EncodeFailsWhenOutputTooSmall) {
  const Bytes input{0x11, 0x22, 0x33};
  Bytes out(3);  // needs 4
  EXPECT_FALSE(cobsEncode(input.data(), input.size(), out.data(), out.size()).has_value());
  EXPECT_FALSE(cobsEncode(input.data(), input.size(), out.data(), 0).has_value());
}

TEST(CobsTest, DecodeRejectsZeroCode) {
  const Bytes input{0x00, 0x11};
  Bytes out(8);
  EXPECT_FALSE(cobsDecode(input.data(), input.size(), out.data(), out.size()).has_value());
}

TEST(CobsTest, DecodeRejectsZeroInsideBlock) {
  const Bytes input{0x03, 0x11, 0x00};
  Bytes out(8);
  EXPECT_FALSE(cobsDecode(input.data(), input.size(), out.data(), out.size()).has_value());
}

TEST(CobsTest, DecodeRejectsTruncatedBlock) {
  const Bytes input{0x05, 0x11, 0x22};
  Bytes out(8);
  EXPECT_FALSE(cobsDecode(input.data(), input.size(), out.data(), out.size()).has_value());
}

TEST(CobsTest, DecodeFailsWhenOutputTooSmall) {
  const Bytes encoded = encode({0x11, 0x00, 0x22});
  Bytes out(2);
  EXPECT_FALSE(cobsDecode(encoded.data(), encoded.size(), out.data(), out.size()).has_value());
}

}  // namespace
