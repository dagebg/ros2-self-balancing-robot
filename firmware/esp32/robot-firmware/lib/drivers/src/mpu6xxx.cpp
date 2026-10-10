#include "drivers/mpu6xxx.hpp"

#include <array>

namespace robot::drivers {

namespace {

constexpr float kPi = 3.14159265358979F;
constexpr float kAccelScale = Mpu6xxx::kStandardGravity / Mpu6xxx::kAccelLsbPerG;  // m/s^2 per LSB
constexpr float kGyroScale = kPi / 180.0F / Mpu6xxx::kGyroLsbPerDps;               // rad/s per LSB

constexpr std::uint8_t kDeviceReset = 0x80;
constexpr std::uint8_t kClockPllGyroX = 0x01;
constexpr std::uint8_t kSampleRateDivider200Hz = 4;  // 1 kHz / (1 + 4)
constexpr std::uint8_t kDlpf42Hz = 0x03;
constexpr std::uint8_t kGyroRange500Dps = 0x08;
constexpr std::uint8_t kAccelRange4G = 0x08;
constexpr std::uint8_t kAccelDlpf41Hz = 0x03;
constexpr std::uint32_t kResetDelayMs = 100;
constexpr std::uint32_t kWakeDelayMs = 10;

constexpr std::size_t kDataLength = 14;

float bigEndianToFloat(const std::array<std::uint8_t, kDataLength>& data, std::size_t offset) {
  const auto high = static_cast<std::uint16_t>(data[offset] << 8U);
  return static_cast<float>(static_cast<std::int16_t>(high | data[offset + 1]));
}

}  // namespace

bool Mpu6xxx::expectedWhoAmI(std::uint8_t value) const noexcept {
  if (variant_ == MpuVariant::kMpu6050) {
    return value == 0x68;
  }
  return value == 0x71 || value == 0x73 || value == 0x70;
}

MpuStatus Mpu6xxx::begin() noexcept {
  who_am_i_ = 0;
  if (!bus_.readRegisters(address_, mpu_register::kWhoAmI, &who_am_i_, 1)) {
    return MpuStatus::kBusError;
  }
  if (!expectedWhoAmI(who_am_i_)) {
    return MpuStatus::kWrongDevice;
  }

  if (!bus_.writeRegister(address_, mpu_register::kPowerManagement1, kDeviceReset)) {
    return MpuStatus::kBusError;
  }
  clock_.delayMs(kResetDelayMs);

  bool ok = bus_.writeRegister(address_, mpu_register::kPowerManagement1, kClockPllGyroX);
  clock_.delayMs(kWakeDelayMs);
  ok =
      ok && bus_.writeRegister(address_, mpu_register::kSampleRateDivider, kSampleRateDivider200Hz);
  ok = ok && bus_.writeRegister(address_, mpu_register::kConfig, kDlpf42Hz);
  ok = ok && bus_.writeRegister(address_, mpu_register::kGyroConfig, kGyroRange500Dps);
  ok = ok && bus_.writeRegister(address_, mpu_register::kAccelConfig, kAccelRange4G);
  if (variant_ == MpuVariant::kMpu9250) {
    ok = ok && bus_.writeRegister(address_, mpu_register::kAccelConfig2, kAccelDlpf41Hz);
  }
  return ok ? MpuStatus::kOk : MpuStatus::kBusError;
}

std::optional<ImuReading> Mpu6xxx::read() noexcept {
  std::array<std::uint8_t, kDataLength> data{};
  if (!bus_.readRegisters(address_, mpu_register::kAccelXoutH, data.data(), data.size())) {
    return std::nullopt;
  }

  ImuReading reading;
  reading.accel = {bigEndianToFloat(data, 0) * kAccelScale, bigEndianToFloat(data, 2) * kAccelScale,
                   bigEndianToFloat(data, 4) * kAccelScale};
  const float raw_temperature = bigEndianToFloat(data, 6);
  reading.gyro = {bigEndianToFloat(data, 8) * kGyroScale, bigEndianToFloat(data, 10) * kGyroScale,
                  bigEndianToFloat(data, 12) * kGyroScale};
  // Datasheet formulas: MPU-6050 register map 4.18, MPU-9250 register map 4.23.
  reading.temperature = variant_ == MpuVariant::kMpu6050 ? raw_temperature / 340.0F + 36.53F
                                                         : raw_temperature / 333.87F + 21.0F;
  return reading;
}

}  // namespace robot::drivers
