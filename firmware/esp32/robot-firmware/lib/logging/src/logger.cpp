#include "logging/logger.hpp"

#include <cstdio>

namespace robot::logging {

std::string_view toString(Level level) noexcept {
  switch (level) {
    case Level::kDebug:
      return "DBG";
    case Level::kInfo:
      return "INF";
    case Level::kWarn:
      return "WRN";
    case Level::kError:
      return "ERR";
  }
  return "???";
}

bool Logger::addSink(Sink& sink) noexcept {
  const bool has_room = sink_count_ < kMaxSinks;
  if (has_room) {
    sinks_.at(sink_count_) = &sink;
    ++sink_count_;
  }
  return has_room;
}

void Logger::vlog(Level level, const char* format, std::va_list args) noexcept {
  if (level < min_level_ || sink_count_ == 0) {
    return;
  }

  std::array<char, kMaxMessageLength + 1> buffer{};
  const int written = std::vsnprintf(buffer.data(), buffer.size(), format, args);
  if (written < 0) {
    return;
  }

  auto length = static_cast<std::size_t>(written);
  if (length > kMaxMessageLength) {
    constexpr std::string_view kEllipsis = "...";
    length = kMaxMessageLength;
    kEllipsis.copy(buffer.data() + length - kEllipsis.size(), kEllipsis.size());
  }

  const std::string_view message(buffer.data(), length);
  for (std::size_t i = 0; i < sink_count_; ++i) {
    sinks_.at(i)->write(level, message);
  }
}

// The variadic entry points only unpack their arguments and forward to vlog().
// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
void Logger::log(Level level, const char* format, ...) noexcept {
  std::va_list args;
  va_start(args, format);
  vlog(level, format, args);
  va_end(args);
}

void Logger::debug(const char* format, ...) noexcept {
  std::va_list args;
  va_start(args, format);
  vlog(Level::kDebug, format, args);
  va_end(args);
}

void Logger::info(const char* format, ...) noexcept {
  std::va_list args;
  va_start(args, format);
  vlog(Level::kInfo, format, args);
  va_end(args);
}

void Logger::warn(const char* format, ...) noexcept {
  std::va_list args;
  va_start(args, format);
  vlog(Level::kWarn, format, args);
  va_end(args);
}

void Logger::error(const char* format, ...) noexcept {
  std::va_list args;
  va_start(args, format);
  vlog(Level::kError, format, args);
  va_end(args);
}
// NOLINTEND(cppcoreguidelines-pro-type-vararg,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

}  // namespace robot::logging
