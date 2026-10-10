#include "hal_arduino/hal_arduino.hpp"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

namespace robot::hal_arduino {

// --- WireI2cBus --------------------------------------------------------------------------------

bool WireI2cBus::writeRegisters(std::uint8_t address, std::uint8_t reg, const std::uint8_t* data,
                                std::size_t size) {
  wire_.beginTransmission(address);
  wire_.write(reg);
  wire_.write(data, size);
  return wire_.endTransmission() == 0;
}

bool WireI2cBus::readRegisters(std::uint8_t address, std::uint8_t reg, std::uint8_t* data,
                               std::size_t size) {
  wire_.beginTransmission(address);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) {  // repeated start
    return false;
  }
  if (wire_.requestFrom(static_cast<std::uint16_t>(address), size, true) != size) {
    return false;
  }
  for (std::size_t i = 0; i < size; ++i) {
    data[i] = static_cast<std::uint8_t>(wire_.read());
  }
  return true;
}

// --- LedcPwmOutput -----------------------------------------------------------------------------

void LedcPwmOutput::begin() {
  ledcSetup(channel_, frequency_hz_, kResolutionBits);
  ledcAttachPin(pin_, channel_);
  ledcWrite(channel_, 0);
}

void LedcPwmOutput::setDuty(float duty) {
  constexpr auto kMaxCount = static_cast<float>((1U << kResolutionBits) - 1U);
  const float clamped = std::isfinite(duty) ? std::clamp(duty, 0.0F, 1.0F) : 0.0F;
  ledcWrite(channel_, static_cast<std::uint32_t>(std::lround(clamped * kMaxCount)));
}

// --- GpioOutput --------------------------------------------------------------------------------

void GpioOutput::begin() {
  pinMode(pin_, OUTPUT);
  digitalWrite(pin_, LOW);
}

void GpioOutput::write(bool high) { digitalWrite(pin_, high ? HIGH : LOW); }

// --- PcntQuadratureCounter ---------------------------------------------------------------------

bool PcntQuadratureCounter::begin() {
  // Two channels on one unit count every edge of both phases (x4 decoding). Configuration from
  // the ESP-IDF rotary encoder example: each channel counts its own phase and uses the other
  // phase to decide the direction.
  pcnt_config_t config{};
  config.unit = unit_;
  config.counter_h_lim = kLimit;
  config.counter_l_lim = static_cast<std::int16_t>(-kLimit);

  config.channel = PCNT_CHANNEL_0;
  config.pulse_gpio_num = pin_a_;
  config.ctrl_gpio_num = pin_b_;
  config.pos_mode = PCNT_COUNT_DEC;
  config.neg_mode = PCNT_COUNT_INC;
  config.lctrl_mode = PCNT_MODE_REVERSE;
  config.hctrl_mode = PCNT_MODE_KEEP;
  if (pcnt_unit_config(&config) != ESP_OK) {
    return false;
  }

  config.channel = PCNT_CHANNEL_1;
  config.pulse_gpio_num = pin_b_;
  config.ctrl_gpio_num = pin_a_;
  config.pos_mode = PCNT_COUNT_INC;
  config.neg_mode = PCNT_COUNT_DEC;
  if (pcnt_unit_config(&config) != ESP_OK) {
    return false;
  }

  return pcnt_set_filter_value(unit_, kFilterCycles) == ESP_OK &&
         pcnt_filter_enable(unit_) == ESP_OK && pcnt_counter_pause(unit_) == ESP_OK &&
         pcnt_counter_clear(unit_) == ESP_OK && pcnt_counter_resume(unit_) == ESP_OK;
}

std::int16_t PcntQuadratureCounter::rawCount() {
  std::int16_t count = 0;
  pcnt_get_counter_value(unit_, &count);
  return count;
}

// --- SerialByteStream --------------------------------------------------------------------------

void SerialByteStream::begin(std::uint32_t baud) { serial_.begin(baud); }

std::size_t SerialByteStream::read(std::uint8_t* data, std::size_t capacity) {
  const int available = serial_.available();
  if (available <= 0) {
    return 0;
  }
  const auto count = std::min(capacity, static_cast<std::size_t>(available));
  return serial_.readBytes(data, count);
}

std::size_t SerialByteStream::write(const std::uint8_t* data, std::size_t size) {
  return serial_.write(data, size);
}

// --- ArduinoClock ------------------------------------------------------------------------------

std::uint32_t ArduinoClock::micros() { return ::micros(); }

void ArduinoClock::delayMs(std::uint32_t milliseconds) { ::delay(milliseconds); }

}  // namespace robot::hal_arduino
