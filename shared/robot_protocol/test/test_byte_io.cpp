#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>

#include "robot_protocol/byte_io.hpp"

namespace {

using robot::protocol::ByteReader;
using robot::protocol::ByteWriter;

TEST(ByteIoTest, WritesLittleEndian) {
  std::array<std::uint8_t, 10> buffer{};
  ByteWriter writer(buffer.data(), buffer.size());
  writer.writeU16(0x1234);
  writer.writeU32(0xA1B2C3D4);
  writer.writeF32(1.0F);  // 0x3F800000

  ASSERT_TRUE(writer.ok());
  EXPECT_EQ(writer.size(), 10U);
  const std::array<std::uint8_t, 10> expected{0x34, 0x12, 0xD4, 0xC3, 0xB2,
                                              0xA1, 0x00, 0x00, 0x80, 0x3F};
  EXPECT_EQ(buffer, expected);
}

TEST(ByteIoTest, RoundTripsAllTypes) {
  std::array<std::uint8_t, 32> buffer{};
  ByteWriter writer(buffer.data(), buffer.size());
  writer.writeU8(0xAB);
  writer.writeU16(0xBEEF);
  writer.writeU32(0xDEADBEEF);
  writer.writeI32(-123456789);
  writer.writeF32(-3.25F);
  writer.writeF32(std::numeric_limits<float>::infinity());
  ASSERT_TRUE(writer.ok());

  ByteReader reader(buffer.data(), writer.size());
  EXPECT_EQ(reader.readU8(), 0xAB);
  EXPECT_EQ(reader.readU16(), 0xBEEF);
  EXPECT_EQ(reader.readU32(), 0xDEADBEEFU);
  EXPECT_EQ(reader.readI32(), -123456789);
  EXPECT_EQ(reader.readF32(), -3.25F);
  EXPECT_EQ(reader.readF32(), std::numeric_limits<float>::infinity());
  EXPECT_TRUE(reader.ok());
  EXPECT_EQ(reader.remaining(), 0U);
}

TEST(ByteIoTest, WriterOverflowIsStickyAndWritesNothing) {
  std::array<std::uint8_t, 3> buffer{};
  ByteWriter writer(buffer.data(), buffer.size());
  writer.writeU16(0xFFFF);
  writer.writeU16(0xFFFF);  // does not fit
  writer.writeU8(0x01);     // would fit, but the writer already failed

  EXPECT_FALSE(writer.ok());
  EXPECT_EQ(writer.size(), 2U);
  EXPECT_EQ(buffer[2], 0x00);
}

TEST(ByteIoTest, ReaderUnderflowIsStickyAndReturnsZero) {
  const std::array<std::uint8_t, 3> buffer{0x01, 0x02, 0x03};
  ByteReader reader(buffer.data(), buffer.size());

  EXPECT_EQ(reader.readU32(), 0U);
  EXPECT_FALSE(reader.ok());
  EXPECT_EQ(reader.readU8(), 0U);  // still failed
}

TEST(ByteIoTest, SkipAdvancesWithinBounds) {
  const std::array<std::uint8_t, 4> buffer{0x01, 0x02, 0x03, 0x04};
  ByteReader reader(buffer.data(), buffer.size());

  reader.skip(3);
  EXPECT_EQ(reader.readU8(), 0x04);
  reader.skip(1);
  EXPECT_FALSE(reader.ok());
}

}  // namespace
