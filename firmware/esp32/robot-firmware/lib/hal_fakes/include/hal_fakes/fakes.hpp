#pragma once

/// @file
/// Test doubles for the hardware abstraction interfaces (host unit tests only).

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "hal/hal.hpp"

namespace robot::hal_fakes {

/// One register write as seen on the bus.
struct I2cWrite {
  std::uint8_t address;
  std::uint8_t reg;
  std::uint8_t value;

  bool operator==(const I2cWrite& other) const {
    return address == other.address && reg == other.reg && value == other.value;
  }
};

/// I2C bus with a 256-byte register file per device and a log of every register write.
class FakeI2cBus final : public hal::I2cBus {
 public:
  /// Makes a device answer at `address`.
  void addDevice(std::uint8_t address) { devices_[address]; }

  /// Presets a register value (as the device would report it).
  void setRegister(std::uint8_t address, std::uint8_t reg, std::uint8_t value) {
    devices_[address].at(reg) = value;
  }

  [[nodiscard]] std::uint8_t registerValue(std::uint8_t address, std::uint8_t reg) const {
    return devices_.at(address).at(reg);
  }

  /// When set, every transaction fails as if the bus were disconnected.
  void setFailing(bool failing) { failing_ = failing; }

  [[nodiscard]] const std::vector<I2cWrite>& writes() const { return writes_; }

  bool writeRegisters(std::uint8_t address, std::uint8_t reg, const std::uint8_t* data,
                      std::size_t size) override {
    auto device = devices_.find(address);
    if (failing_ || device == devices_.end()) {
      return false;
    }
    for (std::size_t i = 0; i < size; ++i) {
      const auto target = static_cast<std::uint8_t>(reg + i);
      device->second.at(target) = data[i];
      writes_.push_back({address, target, data[i]});
    }
    return true;
  }

  bool readRegisters(std::uint8_t address, std::uint8_t reg, std::uint8_t* data,
                     std::size_t size) override {
    auto device = devices_.find(address);
    if (failing_ || device == devices_.end()) {
      return false;
    }
    for (std::size_t i = 0; i < size; ++i) {
      data[i] = device->second.at(static_cast<std::uint8_t>(reg + i));
    }
    return true;
  }

 private:
  std::map<std::uint8_t, std::array<std::uint8_t, 256>> devices_;
  std::vector<I2cWrite> writes_;
  bool failing_ = false;
};

/// Records the duty cycle history.
class FakePwm final : public hal::PwmOutput {
 public:
  void setDuty(float duty) override { history.push_back(duty); }
  [[nodiscard]] float duty() const { return history.empty() ? 0.0F : history.back(); }
  std::vector<float> history;
};

/// Records the current level.
class FakeDigitalOutput final : public hal::DigitalOutput {
 public:
  void write(bool high) override {
    level = high;
    ++writes;
  }
  bool level = false;
  int writes = 0;
};

/// Simulates the ESP32 PCNT: counts reset to zero at +/- limit.
class FakePcnt final : public hal::QuadratureCounter {
 public:
  explicit FakePcnt(std::int16_t limit) : limit_(limit) {}

  /// Applies `pulses` counts one at a time, like the hardware would.
  void count(std::int32_t pulses) {
    const int step = pulses >= 0 ? 1 : -1;
    for (std::int32_t i = 0; i != pulses; i += step) {
      raw_ = static_cast<std::int16_t>(raw_ + step);
      if (raw_ >= limit_ || raw_ <= -limit_) {
        raw_ = 0;
      }
    }
  }

  std::int16_t rawCount() override { return raw_; }

 private:
  std::int16_t limit_;
  std::int16_t raw_ = 0;
};

/// Manually advanced clock; delays advance it instantly.
class FakeClock final : public hal::Clock {
 public:
  [[nodiscard]] std::uint32_t micros() override { return now_us; }
  void delayMs(std::uint32_t milliseconds) override {
    now_us += milliseconds * 1000U;
    total_delay_ms += milliseconds;
  }
  std::uint32_t now_us = 0;
  std::uint32_t total_delay_ms = 0;
};

}  // namespace robot::hal_fakes
