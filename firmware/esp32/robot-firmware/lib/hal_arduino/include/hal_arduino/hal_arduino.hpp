#pragma once

/// @file
/// ESP32 implementations of the hardware abstraction interfaces (Arduino-ESP32 2.x / ESP-IDF 4.4).
///
/// Constructors only store configuration; begin() touches the hardware. Objects can therefore
/// be file-scope statics in the composition root without static-initialization-order problems.

#include <Wire.h>
#include <driver/pcnt.h>

#include <cstdint>

#include "hal/hal.hpp"

namespace robot::hal_arduino {

/// I2C master on an Arduino `TwoWire` instance.
class WireI2cBus final : public hal::I2cBus {
 public:
  explicit WireI2cBus(TwoWire& wire) noexcept : wire_(wire) {}

  bool writeRegisters(std::uint8_t address, std::uint8_t reg, const std::uint8_t* data,
                      std::size_t size) override;
  bool readRegisters(std::uint8_t address, std::uint8_t reg, std::uint8_t* data,
                     std::size_t size) override;

 private:
  TwoWire& wire_;
};

/// PWM output on an LEDC channel.
class LedcPwmOutput final : public hal::PwmOutput {
 public:
  static constexpr std::uint32_t kDefaultFrequencyHz = 20'000;  // above hearing range
  static constexpr std::uint8_t kResolutionBits = 10;

  LedcPwmOutput(std::uint8_t pin, std::uint8_t channel,
                std::uint32_t frequency_hz = kDefaultFrequencyHz) noexcept
      : pin_(pin), channel_(channel), frequency_hz_(frequency_hz) {}

  void begin();
  void setDuty(float duty) override;

 private:
  std::uint8_t pin_;
  std::uint8_t channel_;
  std::uint32_t frequency_hz_;
};

/// Digital output on a GPIO pin; starts low.
class GpioOutput final : public hal::DigitalOutput {
 public:
  explicit GpioOutput(std::uint8_t pin) noexcept : pin_(pin) {}

  void begin();
  void write(bool high) override;

 private:
  std::uint8_t pin_;
};

/// x4 quadrature decoding on one PCNT unit, with the hardware glitch filter enabled.
class PcntQuadratureCounter final : public hal::QuadratureCounter {
 public:
  /// Symmetric counter limit; the count resets to zero when it reaches +/- this value.
  static constexpr std::int16_t kLimit = 30000;
  /// Glitch filter length in APB clock cycles (80 MHz): pulses shorter than 1.25 us are ignored.
  static constexpr std::uint16_t kFilterCycles = 100;

  PcntQuadratureCounter(pcnt_unit_t unit, std::uint8_t pin_a, std::uint8_t pin_b) noexcept
      : unit_(unit), pin_a_(pin_a), pin_b_(pin_b) {}

  /// Configures the unit. Returns false if the ESP-IDF driver rejects the configuration.
  bool begin();
  std::int16_t rawCount() override;

 private:
  pcnt_unit_t unit_;
  std::uint8_t pin_a_;
  std::uint8_t pin_b_;
};

/// Arduino `micros()` and `delay()`.
class ArduinoClock final : public hal::Clock {
 public:
  [[nodiscard]] std::uint32_t micros() override;
  void delayMs(std::uint32_t milliseconds) override;
};

}  // namespace robot::hal_arduino
