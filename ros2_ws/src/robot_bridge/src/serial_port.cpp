#include "robot_bridge/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <system_error>

namespace robot_bridge {

namespace {

speed_t toSpeed(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      throw std::invalid_argument("unsupported baud rate: " + std::to_string(baud));
  }
}

}  // namespace

SerialPort::~SerialPort() { close(); }

void SerialPort::open(const std::string& path, int baud) {
  close();
  const speed_t speed = toSpeed(baud);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): POSIX open() is variadic.
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "open " + path);
  }

  termios options{};
  if (tcgetattr(fd_, &options) != 0) {
    fail("tcgetattr");
  }
  cfmakeraw(&options);  // 8 data bits, no parity, no echo, no line processing
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | CRTSCTS);
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0 ||
      tcsetattr(fd_, TCSANOW, &options) != 0) {
    fail("configure serial port");
  }
  tcflush(fd_, TCIOFLUSH);
}

void SerialPort::close() noexcept {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SerialPort::fail(const char* what) {
  const int error = errno;
  close();
  throw std::system_error(error, std::generic_category(), what);
}

std::size_t SerialPort::read(std::uint8_t* data, std::size_t capacity) {
  if (fd_ < 0) {
    return 0;
  }
  const ssize_t count = ::read(fd_, data, capacity);
  if (count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    fail("read");
  }
  return static_cast<std::size_t>(count);
}

std::size_t SerialPort::write(const std::uint8_t* data, std::size_t size, int timeout_ms) {
  std::size_t written = 0;
  while (fd_ >= 0 && written < size) {
    const ssize_t count = ::write(fd_, data + written, size - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      fail("write");
    }
    pollfd waiter{fd_, POLLOUT, 0};
    if (::poll(&waiter, 1, timeout_ms) <= 0) {
      break;  // device not draining: give up on the rest of this frame
    }
  }
  return written;
}

}  // namespace robot_bridge
