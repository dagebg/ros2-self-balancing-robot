#pragma once

/// @file
/// Hardware-independent, allocation-free logger with pluggable sinks.

#include <array>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <string_view>

// An attribute cannot be produced by a constexpr function, so this has to be a macro.
// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#if defined(__GNUC__) || defined(__clang__)
#define ROBOT_PRINTF_FORMAT(fmt_index, args_index) \
  __attribute__((format(printf, fmt_index, args_index)))
#else
#define ROBOT_PRINTF_FORMAT(fmt_index, args_index)
#endif
// NOLINTEND(cppcoreguidelines-macro-usage)

namespace robot::logging {

/// Severity of a log message, in increasing order.
enum class Level : std::uint8_t { kDebug, kInfo, kWarn, kError };

/// Short, fixed-width tag for a level, e.g. `"WRN"`.
[[nodiscard]] std::string_view toString(Level level) noexcept;

/// Destination for formatted log messages (telnet, protocol frames, test recorder, ...).
///
/// Sinks may be called from several FreeRTOS tasks; each sink is responsible for its own
/// thread safety.
class Sink {
 public:
  Sink() = default;
  Sink(const Sink&) = delete;
  Sink& operator=(const Sink&) = delete;
  Sink(Sink&&) = delete;
  Sink& operator=(Sink&&) = delete;
  virtual ~Sink() = default;

  /// Receives one complete message without a trailing newline.
  virtual void write(Level level, std::string_view message) = 0;
};

/// printf-style logger that formats into a stack buffer and fans out to up to #kMaxSinks sinks.
///
/// Formatting never allocates. Messages longer than #kMaxMessageLength are truncated and end
/// in `"..."`. The logger itself holds no mutable state while logging, so it can be shared
/// between tasks once all sinks are registered.
class Logger {
 public:
  static constexpr std::size_t kMaxSinks = 4;
  static constexpr std::size_t kMaxMessageLength = 160;

  /// Registers a sink. Returns false if #kMaxSinks sinks are already registered.
  bool addSink(Sink& sink) noexcept;

  /// Messages below this level are dropped. Default: Level::kInfo.
  void setMinLevel(Level level) noexcept { min_level_ = level; }
  [[nodiscard]] Level minLevel() const noexcept { return min_level_; }

  void log(Level level, const char* format, ...) noexcept ROBOT_PRINTF_FORMAT(3, 4);
  void vlog(Level level, const char* format, std::va_list args) noexcept;

  void debug(const char* format, ...) noexcept ROBOT_PRINTF_FORMAT(2, 3);
  void info(const char* format, ...) noexcept ROBOT_PRINTF_FORMAT(2, 3);
  void warn(const char* format, ...) noexcept ROBOT_PRINTF_FORMAT(2, 3);
  void error(const char* format, ...) noexcept ROBOT_PRINTF_FORMAT(2, 3);

 private:
  std::array<Sink*, kMaxSinks> sinks_{};
  std::size_t sink_count_ = 0;
  Level min_level_ = Level::kInfo;
};

}  // namespace robot::logging
