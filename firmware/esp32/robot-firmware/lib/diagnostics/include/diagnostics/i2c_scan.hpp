#pragma once

/// @file
/// Hardware-independent I2C bus scan used as a boot-time wiring check.

#include <array>
#include <cstddef>
#include <cstdint>

namespace robot::diagnostics {

/// First and last non-reserved 7-bit I2C addresses.
inline constexpr std::uint8_t kFirstI2cAddress = 0x08;
inline constexpr std::uint8_t kLastI2cAddress = 0x77;

/// Addresses that acknowledged during a scan, in ascending order.
class I2cScanResult {
 public:
  static constexpr std::size_t kCapacity = 16;

  /// Records an address. Addresses beyond #kCapacity are counted but not stored.
  void add(std::uint8_t address) noexcept {
    if (count_ < kCapacity) {
      addresses_.at(count_) = address;
    }
    ++count_;
  }

  /// Number of devices that acknowledged (may exceed #kCapacity).
  [[nodiscard]] std::size_t count() const noexcept { return count_; }

  /// Number of stored addresses, i.e. min(count(), kCapacity).
  [[nodiscard]] std::size_t storedCount() const noexcept {
    return count_ < kCapacity ? count_ : kCapacity;
  }

  [[nodiscard]] std::uint8_t at(std::size_t index) const { return addresses_.at(index); }

  [[nodiscard]] bool contains(std::uint8_t address) const noexcept {
    for (std::size_t i = 0; i < storedCount(); ++i) {
      if (addresses_.at(i) == address) {
        return true;
      }
    }
    return false;
  }

 private:
  std::array<std::uint8_t, kCapacity> addresses_{};
  std::size_t count_ = 0;
};

/// Probes every non-reserved address and returns those that acknowledged.
///
/// @param probe Callable `bool(std::uint8_t address)` that returns true on ACK.
///              On the robot it wraps `Wire.beginTransmission()/endTransmission()`.
template <typename Probe>
[[nodiscard]] I2cScanResult scanI2c(Probe&& probe) {
  I2cScanResult result;
  for (unsigned address = kFirstI2cAddress; address <= kLastI2cAddress; ++address) {
    const auto address8 = static_cast<std::uint8_t>(address);
    if (probe(address8)) {
      result.add(address8);
    }
  }
  return result;
}

}  // namespace robot::diagnostics
