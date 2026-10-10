#pragma once

/// @file
/// Driver for the accelerometer/gyroscope of the InvenSense MPU-6050 and MPU-9250 family.
///
/// Both chips share the register map used here; they differ in WHO_AM_I, the accelerometer
/// low-pass filter register and the temperature formula. The MPU-9250's magnetometer (AK8963) is
/// not used by the balance controller and therefore not driven.

#include <cstdint>
#include <optional>

#include "drivers/vector3.hpp"
#include "hal/hal.hpp"

namespace robot::drivers {

enum class MpuVariant : std::uint8_t {
  kMpu6050,  ///< MPU-6050 (WHO_AM_I 0x68)
  kMpu9250,  ///< MPU-9250 / MPU-9255 / MPU-6500 (WHO_AM_I 0x71 / 0x73 / 0x70)
};

enum class MpuStatus : std::uint8_t {
  kOk,
  kBusError,     ///< The device did not acknowledge.
  kWrongDevice,  ///< WHO_AM_I does not match the expected variant.
};

/// One sample in the sensor's own axes.
struct ImuReading {
  Vector3 accel;             ///< [m/s^2]
  Vector3 gyro;              ///< [rad/s]
  float temperature = 0.0F;  ///< [degC]
};

/// Configures the sensor for balancing and reads samples in SI units.
///
/// Configuration: 200 Hz sample rate, ~42 Hz digital low-pass filter (gyro and accel),
/// +-4 g accelerometer range, +-500 deg/s gyroscope range.
class Mpu6xxx {
 public:
  static constexpr float kAccelLsbPerG = 8192.0F;  // +-4 g
  static constexpr float kGyroLsbPerDps = 65.5F;   // +-500 deg/s
  static constexpr float kStandardGravity = 9.80665F;

  Mpu6xxx(hal::I2cBus& bus, hal::Clock& clock, std::uint8_t address, MpuVariant variant) noexcept
      : bus_(bus), clock_(clock), address_(address), variant_(variant) {}

  /// Resets and configures the sensor. Blocks for about 110 ms.
  MpuStatus begin() noexcept;

  /// Reads one sample with a single 14-byte burst, so all axes come from the same instant.
  [[nodiscard]] std::optional<ImuReading> read() noexcept;

  /// WHO_AM_I value read by the last begin(), for diagnostics.
  [[nodiscard]] std::uint8_t whoAmI() const noexcept { return who_am_i_; }

  [[nodiscard]] std::uint8_t address() const noexcept { return address_; }
  [[nodiscard]] MpuVariant variant() const noexcept { return variant_; }

 private:
  [[nodiscard]] bool expectedWhoAmI(std::uint8_t value) const noexcept;

  hal::I2cBus& bus_;
  hal::Clock& clock_;
  std::uint8_t address_;
  MpuVariant variant_;
  std::uint8_t who_am_i_ = 0;
};

/// Register addresses (shared by both variants unless noted).
namespace mpu_register {
inline constexpr std::uint8_t kSampleRateDivider = 0x19;
inline constexpr std::uint8_t kConfig = 0x1A;
inline constexpr std::uint8_t kGyroConfig = 0x1B;
inline constexpr std::uint8_t kAccelConfig = 0x1C;
inline constexpr std::uint8_t kAccelConfig2 = 0x1D;  ///< MPU-9250 family only
inline constexpr std::uint8_t kAccelXoutH = 0x3B;    ///< First of 14 data bytes
inline constexpr std::uint8_t kPowerManagement1 = 0x6B;
inline constexpr std::uint8_t kWhoAmI = 0x75;
}  // namespace mpu_register

}  // namespace robot::drivers
