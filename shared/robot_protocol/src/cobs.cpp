#include "robot_protocol/cobs.hpp"

namespace robot::protocol {

namespace {
constexpr std::uint8_t kMaxCode = 0xFF;  // a full block: 254 data bytes, no implicit zero
}  // namespace

std::optional<std::size_t> cobsEncode(const std::uint8_t* input, std::size_t size,
                                      std::uint8_t* output, std::size_t capacity) noexcept {
  std::size_t write = 0;
  std::size_t code_pos = 0;
  std::uint8_t code = 1;
  bool block_open = false;

  // Reserves the code byte of a new block at the current write position.
  const auto open_block = [&]() {
    if (write >= capacity) {
      return false;
    }
    code_pos = write++;
    code = 1;
    block_open = true;
    return true;
  };

  if (!open_block()) {
    return std::nullopt;
  }
  for (std::size_t read = 0; read < size; ++read) {
    const std::uint8_t byte = input[read];
    if (byte == 0) {
      output[code_pos] = code;
      if (!open_block()) {
        return std::nullopt;
      }
      continue;
    }
    if (write >= capacity) {
      return std::nullopt;
    }
    output[write++] = byte;
    ++code;
    if (code == kMaxCode) {
      output[code_pos] = code;
      block_open = false;
      // A full block at the very end needs no further code byte.
      if (read + 1 < size && !open_block()) {
        return std::nullopt;
      }
    }
  }
  if (block_open) {
    output[code_pos] = code;
  }
  return write;
}

std::optional<std::size_t> cobsDecode(const std::uint8_t* input, std::size_t size,
                                      std::uint8_t* output, std::size_t capacity) noexcept {
  std::size_t read = 0;
  std::size_t write = 0;
  while (read < size) {
    const std::uint8_t code = input[read++];
    if (code == 0) {
      return std::nullopt;
    }
    for (std::uint8_t i = 1; i < code; ++i) {
      if (read >= size || input[read] == 0 || write >= capacity) {
        return std::nullopt;
      }
      output[write++] = input[read++];
    }
    // Every block except a full one and the last one ends with an implicit zero.
    if (code != kMaxCode && read < size) {
      if (write >= capacity) {
        return std::nullopt;
      }
      output[write++] = 0;
    }
  }
  return write;
}

}  // namespace robot::protocol
