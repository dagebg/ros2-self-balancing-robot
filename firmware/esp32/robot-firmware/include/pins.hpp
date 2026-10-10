#pragma once

/// @file
/// Board wiring of the robot, derived from the KiCad netlist
/// (see docs: Hardware > Pinout). Change this file only together with the schematic.

#include <cstdint>

/// ESP32 GPIO assignments and bus addresses.
namespace robot::pins {

using Gpio = std::uint8_t;

// --- TB6612FNG dual H-bridge ----------------------------------------------------------------
inline constexpr Gpio kMotorAPwm = 25;     ///< PWMA
inline constexpr Gpio kMotorAIn1 = 26;     ///< AIN1
inline constexpr Gpio kMotorAIn2 = 27;     ///< AIN2
inline constexpr Gpio kMotorBPwm = 32;     ///< PWMB
inline constexpr Gpio kMotorBIn1 = 33;     ///< BIN1
inline constexpr Gpio kMotorBIn2 = 13;     ///< BIN2
inline constexpr Gpio kMotorStandby = 16;  ///< STBY, low disables both outputs

// --- Wheel encoders (GPIO 34/35/36 are input-only, no internal pull-ups) --------------------
inline constexpr Gpio kEncoderJ5A = 34;  ///< Connector J5 (motor M2), phase A
inline constexpr Gpio kEncoderJ5B = 4;   ///< Connector J5 (motor M2), phase B
inline constexpr Gpio kEncoderJ6A = 36;  ///< Connector J6 (motor M1), phase A
inline constexpr Gpio kEncoderJ6B = 35;  ///< Connector J6 (motor M1), phase B

// --- I2C bus with both IMUs -----------------------------------------------------------------
inline constexpr Gpio kI2cSda = 21;
inline constexpr Gpio kI2cScl = 22;
inline constexpr std::uint32_t kI2cClockHz = 400'000;
inline constexpr std::uint8_t kMpu9250Address = 0x69;  ///< AD0 tied high
inline constexpr std::uint8_t kMpu6050Address = 0x68;  ///< AD0 tied low

// --- Debug access ---------------------------------------------------------------------------
inline constexpr std::uint16_t kTelnetPort = 23;  ///< Log stream over the WiFi SoftAP

}  // namespace robot::pins
