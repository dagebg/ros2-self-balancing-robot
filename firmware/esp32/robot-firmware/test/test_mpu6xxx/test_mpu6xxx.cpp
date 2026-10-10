#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "drivers/mpu6xxx.hpp"
#include "hal_fakes/fakes.hpp"

namespace {

using robot::drivers::Mpu6xxx;
using robot::drivers::MpuStatus;
using robot::drivers::MpuVariant;
using robot::hal_fakes::FakeClock;
using robot::hal_fakes::FakeI2cBus;
using robot::hal_fakes::I2cWrite;
namespace reg = robot::drivers::mpu_register;

constexpr std::uint8_t kAddress = 0x68;
constexpr float kPi = 3.14159265358979F;

class Mpu6xxxTest : public ::testing::Test {
 protected:
  void SetUp() override { bus.addDevice(kAddress); }

  void setWhoAmI(std::uint8_t value) { bus.setRegister(kAddress, reg::kWhoAmI, value); }

  void setRaw(std::uint8_t first_register, std::int16_t value) {
    const auto bits = static_cast<std::uint16_t>(value);
    bus.setRegister(kAddress, first_register, static_cast<std::uint8_t>(bits >> 8U));
    bus.setRegister(kAddress, static_cast<std::uint8_t>(first_register + 1),
                    static_cast<std::uint8_t>(bits & 0xFFU));
  }

  FakeI2cBus bus;
  FakeClock clock;
};

TEST_F(Mpu6xxxTest, Mpu6050InitializationWritesExpectedSequence) {
  setWhoAmI(0x68);
  Mpu6xxx imu(bus, clock, kAddress, MpuVariant::kMpu6050);

  ASSERT_EQ(imu.begin(), MpuStatus::kOk);

  const std::vector<I2cWrite> expected{
      {kAddress, reg::kPowerManagement1, 0x80},  // reset
      {kAddress, reg::kPowerManagement1, 0x01},  // wake, PLL clock
      {kAddress, reg::kSampleRateDivider, 4},    // 200 Hz
      {kAddress, reg::kConfig, 0x03},            // DLPF ~42 Hz
      {kAddress, reg::kGyroConfig, 0x08},        // +-500 dps
      {kAddress, reg::kAccelConfig, 0x08},       // +-4 g
  };
  EXPECT_EQ(bus.writes(), expected);
  EXPECT_GE(clock.total_delay_ms, 100U);  // waited for the reset to complete
}

TEST_F(Mpu6xxxTest, Mpu9250AlsoConfiguresAccelerometerFilter) {
  setWhoAmI(0x71);
  Mpu6xxx imu(bus, clock, kAddress, MpuVariant::kMpu9250);

  ASSERT_EQ(imu.begin(), MpuStatus::kOk);

  ASSERT_FALSE(bus.writes().empty());
  EXPECT_EQ(bus.writes().back(), (I2cWrite{kAddress, reg::kAccelConfig2, 0x03}));
}

TEST_F(Mpu6xxxTest, AcceptsMpu9250FamilyIdentifiers) {
  for (const std::uint8_t id : {0x70, 0x71, 0x73}) {
    setWhoAmI(id);
    Mpu6xxx imu(bus, clock, kAddress, MpuVariant::kMpu9250);
    EXPECT_EQ(imu.begin(), MpuStatus::kOk) << "WHO_AM_I " << int{id};
  }
}

TEST_F(Mpu6xxxTest, RejectsWrongDeviceWithoutWritingAnything) {
  setWhoAmI(0x71);  // an MPU-9250 where an MPU-6050 is expected
  Mpu6xxx imu(bus, clock, kAddress, MpuVariant::kMpu6050);

  EXPECT_EQ(imu.begin(), MpuStatus::kWrongDevice);
  EXPECT_EQ(imu.whoAmI(), 0x71);
  EXPECT_TRUE(bus.writes().empty());
}

TEST_F(Mpu6xxxTest, ReportsBusErrorWhenDeviceIsMissing) {
  Mpu6xxx imu(bus, clock, 0x69, MpuVariant::kMpu9250);  // nothing at 0x69
  EXPECT_EQ(imu.begin(), MpuStatus::kBusError);
}

TEST_F(Mpu6xxxTest, ConvertsRawSamplesToSiUnits) {
  setWhoAmI(0x68);
  Mpu6xxx imu(bus, clock, kAddress, MpuVariant::kMpu6050);
  ASSERT_EQ(imu.begin(), MpuStatus::kOk);

  setRaw(reg::kAccelXoutH, 0);
  setRaw(reg::kAccelXoutH + 2, -4096);  // -0.5 g
  setRaw(reg::kAccelXoutH + 4, 8192);   // +1 g
  setRaw(reg::kAccelXoutH + 6, 0);      // temperature
  setRaw(reg::kAccelXoutH + 8, 655);    // ~10 deg/s
  setRaw(reg::kAccelXoutH + 10, -655);
  setRaw(reg::kAccelXoutH + 12, 0);

  const auto reading = imu.read();

  ASSERT_TRUE(reading.has_value());
  EXPECT_FLOAT_EQ(reading->accel.x, 0.0F);
  EXPECT_FLOAT_EQ(reading->accel.y, -0.5F * Mpu6xxx::kStandardGravity);
  EXPECT_FLOAT_EQ(reading->accel.z, Mpu6xxx::kStandardGravity);
  EXPECT_NEAR(reading->gyro.x, 10.0F * kPi / 180.0F, 1e-4F);
  EXPECT_NEAR(reading->gyro.y, -10.0F * kPi / 180.0F, 1e-4F);
  EXPECT_FLOAT_EQ(reading->gyro.z, 0.0F);
  EXPECT_FLOAT_EQ(reading->temperature, 36.53F);
}

TEST_F(Mpu6xxxTest, UsesVariantSpecificTemperatureFormula) {
  setWhoAmI(0x71);
  Mpu6xxx imu(bus, clock, kAddress, MpuVariant::kMpu9250);
  ASSERT_EQ(imu.begin(), MpuStatus::kOk);
  setRaw(reg::kAccelXoutH + 6, 3339);  // ~ +10 degC above 21 degC

  const auto reading = imu.read();

  ASSERT_TRUE(reading.has_value());
  EXPECT_NEAR(reading->temperature, 31.0F, 0.01F);
}

TEST_F(Mpu6xxxTest, ReadFailsOnBusError) {
  setWhoAmI(0x68);
  Mpu6xxx imu(bus, clock, kAddress, MpuVariant::kMpu6050);
  ASSERT_EQ(imu.begin(), MpuStatus::kOk);
  bus.setFailing(true);

  EXPECT_FALSE(imu.read().has_value());
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
