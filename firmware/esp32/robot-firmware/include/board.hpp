#pragma once

/// @file
/// Board configuration resolved from pins.hpp and the generated robot_config.hpp.
///
/// The YAML parameters are plain numbers; this header turns them into typed wiring and checks them
/// at compile time, so an invalid value in config/robot_params.yaml fails the build instead of
/// misbehaving on the robot.

#include <algorithm>
#include <cstdint>

#include "drivers/axis_map.hpp"
#include "pins.hpp"
#include "robot_config.hpp"

namespace robot::board {

// --- IMU mounting -------------------------------------------------------------------------------

inline constexpr drivers::AxisMap kMpu9250Mounting{config::kMpu9250BodyX, config::kMpu9250BodyY,
                                                   config::kMpu9250BodyZ};
inline constexpr drivers::AxisMap kMpu6050Mounting{config::kMpu6050BodyX, config::kMpu6050BodyY,
                                                   config::kMpu6050BodyZ};
static_assert(kMpu9250Mounting.isValid(),
              "imu_mounting.mpu9250_body_* in robot_params.yaml is not a rotation");
static_assert(kMpu6050Mounting.isValid(),
              "imu_mounting.mpu6050_body_* in robot_params.yaml is not a rotation");

// --- Motors -------------------------------------------------------------------------------------

/// Pins of one TB6612 channel and the LEDC channel generating its PWM.
struct MotorPins {
  pins::Gpio pwm;
  pins::Gpio in1;
  pins::Gpio in2;
  std::uint8_t ledc_channel;
};

inline constexpr MotorPins kChannelA{pins::kMotorAPwm, pins::kMotorAIn1, pins::kMotorAIn2, 0};
inline constexpr MotorPins kChannelB{pins::kMotorBPwm, pins::kMotorBIn1, pins::kMotorBIn2, 1};

static_assert(config::kLeftMotorChannel == 0 || config::kLeftMotorChannel == 1,
              "wiring.left_motor_channel must be 0 (A) or 1 (B)");
static_assert(config::kLeftMotorInverted == 0 || config::kLeftMotorInverted == 1,
              "wiring.left_motor_inverted must be 0 or 1");
static_assert(config::kRightMotorInverted == 0 || config::kRightMotorInverted == 1,
              "wiring.right_motor_inverted must be 0 or 1");

inline constexpr MotorPins kLeftMotor = config::kLeftMotorChannel == 0 ? kChannelA : kChannelB;
inline constexpr MotorPins kRightMotor = config::kLeftMotorChannel == 0 ? kChannelB : kChannelA;
inline constexpr bool kLeftMotorInverted = config::kLeftMotorInverted == 1;
inline constexpr bool kRightMotorInverted = config::kRightMotorInverted == 1;

/// Largest PWM duty cycle: keeps the effective motor voltage at or below its rating.
inline constexpr float kMaxMotorDuty =
    std::min(1.0F, config::kMotorRatedVoltage / config::kMotorSupplyVoltage);
static_assert(kMaxMotorDuty > 0.0F, "motor voltages in robot_params.yaml must be positive");

// --- Encoders -----------------------------------------------------------------------------------

struct EncoderPins {
  pins::Gpio a;
  pins::Gpio b;
};

inline constexpr EncoderPins kEncoderJ5{pins::kEncoderJ5A, pins::kEncoderJ5B};
inline constexpr EncoderPins kEncoderJ6{pins::kEncoderJ6A, pins::kEncoderJ6B};

static_assert(config::kLeftEncoderConnector == 5 || config::kLeftEncoderConnector == 6,
              "wiring.left_encoder_connector must be 5 (J5) or 6 (J6)");
static_assert(config::kLeftEncoderInverted == 0 || config::kLeftEncoderInverted == 1,
              "wiring.left_encoder_inverted must be 0 or 1");
static_assert(config::kRightEncoderInverted == 0 || config::kRightEncoderInverted == 1,
              "wiring.right_encoder_inverted must be 0 or 1");

inline constexpr EncoderPins kLeftEncoder =
    config::kLeftEncoderConnector == 5 ? kEncoderJ5 : kEncoderJ6;
inline constexpr EncoderPins kRightEncoder =
    config::kLeftEncoderConnector == 5 ? kEncoderJ6 : kEncoderJ5;
inline constexpr bool kLeftEncoderInverted = config::kLeftEncoderInverted == 1;
inline constexpr bool kRightEncoderInverted = config::kRightEncoderInverted == 1;

/// Encoder counts per wheel revolution (x4 quadrature decoding).
inline constexpr float kCountsPerWheelRev =
    4.0F * static_cast<float>(config::kEncoderPulsesPerMotorRev) * config::kGearRatio;

}  // namespace robot::board
