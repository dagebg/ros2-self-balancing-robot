#pragma once

/// @file
/// Hardware abstraction interfaces.
///
/// Drivers and application logic depend only on these interfaces. The ESP32 implementations live
/// in `hal_arduino`; unit tests provide fakes. Keeping the interfaces this small is deliberate:
/// each one is exactly what a driver needs and nothing more.

#include <cstddef>
#include <cstdint>

namespace robot::hal {

/// Base for interfaces: polymorphic, non-copyable, non-movable.
class Interface {
 public:
  Interface() = default;
  Interface(const Interface&) = delete;
  Interface& operator=(const Interface&) = delete;
  Interface(Interface&&) = delete;
  Interface& operator=(Interface&&) = delete;
  virtual ~Interface() = default;
};

/// Register-oriented I2C master.
class I2cBus : public Interface {
 public:
  /// Writes `size` bytes starting at register `reg`. Returns false on NACK or bus error.
  virtual bool writeRegisters(std::uint8_t address, std::uint8_t reg, const std::uint8_t* data,
                              std::size_t size) = 0;

  /// Reads `size` bytes starting at register `reg` (repeated start). Returns false on error.
  virtual bool readRegisters(std::uint8_t address, std::uint8_t reg, std::uint8_t* data,
                             std::size_t size) = 0;

  /// Writes a single register.
  bool writeRegister(std::uint8_t address, std::uint8_t reg, std::uint8_t value) {
    return writeRegisters(address, reg, &value, 1);
  }
};

/// PWM output with a normalized duty cycle.
class PwmOutput : public Interface {
 public:
  /// Sets the duty cycle in [0, 1]; implementations clamp out-of-range values.
  virtual void setDuty(float duty) = 0;
};

/// Push-pull digital output.
class DigitalOutput : public Interface {
 public:
  virtual void write(bool high) = 0;
};

/// Hardware quadrature counter, e.g. the ESP32 PCNT peripheral.
class QuadratureCounter : public Interface {
 public:
  /// Raw 16-bit hardware count; wraps around. Use robot::drivers::CounterExtender for 32 bits.
  virtual std::int16_t rawCount() = 0;
};

/// Byte-oriented serial link, e.g. a UART. Both calls are non-blocking.
class ByteStream : public Interface {
 public:
  /// Reads up to `capacity` pending bytes; returns how many were read (possibly 0).
  virtual std::size_t read(std::uint8_t* data, std::size_t capacity) = 0;
  /// Queues bytes for sending; returns how many were accepted.
  virtual std::size_t write(const std::uint8_t* data, std::size_t size) = 0;
};

/// Monotonic time source.
class Clock : public Interface {
 public:
  [[nodiscard]] virtual std::uint32_t micros() = 0;
  virtual void delayMs(std::uint32_t milliseconds) = 0;
};

}  // namespace robot::hal
