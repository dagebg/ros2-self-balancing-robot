#include <gtest/gtest.h>

#include <set>
#include <vector>

#include "diagnostics/i2c_scan.hpp"

namespace {

using robot::diagnostics::I2cScanResult;
using robot::diagnostics::kFirstI2cAddress;
using robot::diagnostics::kLastI2cAddress;
using robot::diagnostics::scanI2c;

TEST(I2cScanTest, FindsDevicesThatAcknowledge) {
  const std::set<std::uint8_t> present{0x68, 0x69};

  const auto result = scanI2c([&](std::uint8_t address) { return present.count(address) > 0; });

  EXPECT_EQ(result.count(), 2U);
  EXPECT_TRUE(result.contains(0x68));
  EXPECT_TRUE(result.contains(0x69));
  EXPECT_FALSE(result.contains(0x0C));
}

TEST(I2cScanTest, ResultIsInAscendingOrder) {
  const auto result =
      scanI2c([](std::uint8_t address) { return address == 0x69 || address == 0x10; });

  ASSERT_EQ(result.storedCount(), 2U);
  EXPECT_EQ(result.at(0), 0x10);
  EXPECT_EQ(result.at(1), 0x69);
}

TEST(I2cScanTest, ProbesExactlyTheNonReservedRange) {
  std::vector<std::uint8_t> probed;

  (void)scanI2c([&](std::uint8_t address) {
    probed.push_back(address);
    return false;
  });

  ASSERT_FALSE(probed.empty());
  EXPECT_EQ(probed.front(), kFirstI2cAddress);
  EXPECT_EQ(probed.back(), kLastI2cAddress);
  EXPECT_EQ(probed.size(), static_cast<std::size_t>(kLastI2cAddress - kFirstI2cAddress + 1));
}

TEST(I2cScanTest, CountsDevicesBeyondCapacity) {
  const auto result = scanI2c([](std::uint8_t) { return true; });

  EXPECT_EQ(result.count(), static_cast<std::size_t>(kLastI2cAddress - kFirstI2cAddress + 1));
  EXPECT_EQ(result.storedCount(), I2cScanResult::kCapacity);
}

TEST(I2cScanTest, EmptyBusYieldsEmptyResult) {
  const auto result = scanI2c([](std::uint8_t) { return false; });

  EXPECT_EQ(result.count(), 0U);
  EXPECT_FALSE(result.contains(0x68));
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
