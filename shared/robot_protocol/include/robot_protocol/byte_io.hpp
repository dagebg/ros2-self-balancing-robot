#pragma once

/// @file
/// Bounds-checked little-endian serialization of fixed-width values.
///
/// Values are written byte by byte, so the wire format is independent of the host's endianness,
/// struct padding and compiler.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace robot::protocol {

static_assert(std::numeric_limits<float>::is_iec559, "protocol floats are IEEE-754 binary32");
static_assert(sizeof(float) == sizeof(std::uint32_t), "protocol floats are 32 bits wide");

/// Appends values to a caller-owned buffer. After any write that does not fit, ok() stays false
/// and further writes are ignored.
class ByteWriter {
 public:
  ByteWriter(std::uint8_t* data, std::size_t capacity) noexcept
      : data_(data), capacity_(capacity) {}

  void writeU8(std::uint8_t value) noexcept {
    if (!reserve(1)) {
      return;
    }
    data_[size_++] = value;
  }

  void writeU16(std::uint16_t value) noexcept {
    if (!reserve(2)) {
      return;
    }
    data_[size_++] = static_cast<std::uint8_t>(value & 0xFFU);
    data_[size_++] = static_cast<std::uint8_t>(value >> 8U);
  }

  void writeU32(std::uint32_t value) noexcept {
    if (!reserve(4)) {
      return;
    }
    for (unsigned shift = 0; shift < 32; shift += 8) {
      data_[size_++] = static_cast<std::uint8_t>((value >> shift) & 0xFFU);
    }
  }

  void writeI32(std::int32_t value) noexcept { writeU32(static_cast<std::uint32_t>(value)); }

  void writeF32(float value) noexcept {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeU32(bits);
  }

  void writeBytes(const std::uint8_t* bytes, std::size_t count) noexcept {
    if (!reserve(count)) {
      return;
    }
    std::memcpy(data_ + size_, bytes, count);
    size_ += count;
  }

  [[nodiscard]] bool ok() const noexcept { return ok_; }
  [[nodiscard]] std::size_t size() const noexcept { return size_; }

 private:
  bool reserve(std::size_t count) noexcept {
    if (!ok_ || capacity_ - size_ < count) {
      ok_ = false;
    }
    return ok_;
  }

  std::uint8_t* data_;
  std::size_t capacity_;
  std::size_t size_ = 0;
  bool ok_ = true;
};

/// Reads values from a buffer. After any read past the end, ok() stays false and reads return 0.
class ByteReader {
 public:
  ByteReader(const std::uint8_t* data, std::size_t size) noexcept : data_(data), size_(size) {}

  [[nodiscard]] std::uint8_t readU8() noexcept {
    if (!available(1)) {
      return 0;
    }
    return data_[pos_++];
  }

  [[nodiscard]] std::uint16_t readU16() noexcept {
    if (!available(2)) {
      return 0;
    }
    const auto low = static_cast<std::uint16_t>(data_[pos_++]);
    const auto high = static_cast<std::uint16_t>(data_[pos_++]);
    return static_cast<std::uint16_t>(low | static_cast<std::uint16_t>(high << 8U));
  }

  [[nodiscard]] std::uint32_t readU32() noexcept {
    if (!available(4)) {
      return 0;
    }
    std::uint32_t value = 0;
    for (unsigned shift = 0; shift < 32; shift += 8) {
      value |= static_cast<std::uint32_t>(data_[pos_++]) << shift;
    }
    return value;
  }

  [[nodiscard]] std::int32_t readI32() noexcept { return static_cast<std::int32_t>(readU32()); }

  [[nodiscard]] float readF32() noexcept {
    const std::uint32_t bits = readU32();
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  [[nodiscard]] bool ok() const noexcept { return ok_; }
  [[nodiscard]] std::size_t remaining() const noexcept { return size_ - pos_; }
  [[nodiscard]] const std::uint8_t* current() const noexcept { return data_ + pos_; }

  /// Advances past `count` bytes that the caller has consumed via current().
  void skip(std::size_t count) noexcept {
    if (available(count)) {
      pos_ += count;
    }
  }

 private:
  bool available(std::size_t count) noexcept {
    if (!ok_ || remaining() < count) {
      ok_ = false;
    }
    return ok_;
  }

  const std::uint8_t* data_;
  std::size_t size_;
  std::size_t pos_ = 0;
  bool ok_ = true;
};

}  // namespace robot::protocol
