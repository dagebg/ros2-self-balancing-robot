#pragma once

/// @file
/// Lock-free single-producer, single-consumer queue for passing messages between two tasks.

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace robot::app {

/// Fixed-capacity ring buffer. Exactly one task may call push() and exactly one (other) task may
/// call pop(); neither ever blocks. One slot is kept free to tell "full" from "empty", so the
/// usable capacity is `Size - 1`.
///
/// The producer publishes an element by storing `head_` with release ordering after writing the
/// slot; the consumer reads `head_` with acquire ordering before reading the slot (and vice versa
/// for `tail_`), so the element is fully visible when it is popped.
template <typename T, std::size_t Size>
class SpscQueue {
  static_assert(Size >= 2, "SpscQueue needs at least two slots");

 public:
  /// @return false if the queue is full; the element is dropped.
  bool push(const T& value) noexcept {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) % Size;
    if (next == tail_.load(std::memory_order_acquire)) {
      return false;
    }
    slots_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }

  std::optional<T> pop() noexcept {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return std::nullopt;
    }
    T value = slots_[tail];
    tail_.store((tail + 1) % Size, std::memory_order_release);
    return value;
  }

  [[nodiscard]] bool empty() const noexcept {
    return tail_.load(std::memory_order_acquire) == head_.load(std::memory_order_acquire);
  }

  static constexpr std::size_t capacity() noexcept { return Size - 1; }

 private:
  std::array<T, Size> slots_{};
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
};

}  // namespace robot::app
