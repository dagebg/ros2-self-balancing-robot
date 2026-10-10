#pragma once

/// @file
/// Latest-value exchange between tasks.

#include <cstdint>
#include <mutex>

namespace robot::app {

/// Holds the most recent value of T. Writers overwrite, readers copy; nobody waits for anybody
/// except during the short copy under the lock. Suited to state where only the latest value
/// matters (velocity targets, telemetry snapshots), unlike SpscQueue where every element counts.
template <typename T>
class Mailbox {
 public:
  void publish(const T& value) {
    const std::lock_guard<std::mutex> lock(mutex_);
    value_ = value;
    ++version_;
  }

  [[nodiscard]] T read() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  /// Copies the value only if it changed since `seen_version`; updates `seen_version`.
  bool readIfNew(T& out, std::uint32_t& seen_version) const {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (version_ == seen_version) {
      return false;
    }
    out = value_;
    seen_version = version_;
    return true;
  }

 private:
  mutable std::mutex mutex_;
  T value_{};
  std::uint32_t version_ = 0;
};

}  // namespace robot::app
