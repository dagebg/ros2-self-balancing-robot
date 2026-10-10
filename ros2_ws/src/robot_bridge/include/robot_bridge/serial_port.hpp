#pragma once

/// @file
/// Minimal POSIX serial port for the ESP32 link.

#include <cstddef>
#include <cstdint>
#include <string>

namespace robot_bridge {

/// Raw 8N1 serial port with non-blocking reads. Linux only (termios).
class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  SerialPort(SerialPort&&) = delete;
  SerialPort& operator=(SerialPort&&) = delete;

  /// Opens and configures the device. Throws std::system_error on failure, and
  /// std::invalid_argument for an unsupported baud rate.
  void open(const std::string& path, int baud);
  void close() noexcept;
  [[nodiscard]] bool isOpen() const noexcept { return fd_ >= 0; }

  /// Reads whatever is pending (possibly nothing). Throws std::system_error if the device fails,
  /// e.g. because it was unplugged; the port is closed in that case.
  std::size_t read(std::uint8_t* data, std::size_t capacity);

  /// Writes all bytes, waiting up to `timeout_ms` for the device to accept them. Returns the
  /// number written. Throws std::system_error on device failure (and closes the port).
  std::size_t write(const std::uint8_t* data, std::size_t size, int timeout_ms = 50);

 private:
  [[noreturn]] void fail(const char* what);

  int fd_ = -1;
};

}  // namespace robot_bridge
