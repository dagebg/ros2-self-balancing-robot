#pragma once

/// @file
/// Protocol framing on top of a byte stream, plus the firmware's communication service.

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string_view>

#include "app/mailbox.hpp"
#include "app/spsc_queue.hpp"
#include "hal/hal.hpp"
#include "logging/logger.hpp"
#include "robot_protocol/codec.hpp"
#include "robot_protocol/frame_parser.hpp"

namespace robot::app {

/// Sends and receives protocol frames over a ByteStream.
class Link {
 public:
  explicit Link(hal::ByteStream& stream) noexcept : stream_(stream) {}

  /// Encodes and sends a message with the next sequence number. Returns false if the stream did
  /// not accept the whole frame.
  bool send(const protocol::Message& message) noexcept;

  /// Sends a lone frame delimiter so the receiver starts the next frame cleanly.
  void sendResync() noexcept;

  /// Decodes all pending bytes and calls `on_frame(const protocol::DecodeResult&)` for each frame
  /// that decoded successfully. Errors are only counted.
  template <typename OnFrame>
  void poll(OnFrame&& on_frame) noexcept {
    std::array<std::uint8_t, 64> buffer{};
    std::size_t count = 0;
    while ((count = stream_.read(buffer.data(), buffer.size())) > 0) {
      for (std::size_t i = 0; i < count; ++i) {
        const auto result = parser_.push(buffer[i]);
        if (result && result->ok()) {
          rx_sequence_.observe(result->sequence);
          on_frame(*result);
        }
      }
    }
  }

  [[nodiscard]] const protocol::FrameParser::Stats& rxStats() const noexcept {
    return parser_.stats();
  }
  [[nodiscard]] std::uint32_t rxLost() const noexcept { return rx_sequence_.lost(); }
  [[nodiscard]] std::uint32_t txIncomplete() const noexcept { return tx_incomplete_; }

 private:
  hal::ByteStream& stream_;
  protocol::FrameParser parser_;
  protocol::SequenceTracker rx_sequence_;
  std::uint8_t tx_sequence_ = 0;
  std::uint32_t tx_incomplete_ = 0;
};

/// Bounded, thread-safe queue of log lines waiting to be sent to the Pi. Any task may push;
/// lines are dropped (and counted) when it is full, so logging never blocks the control loop.
class LogQueue {
 public:
  static constexpr std::size_t kCapacity = 16;

  void push(protocol::LogMessage message);
  std::optional<protocol::LogMessage> pop();
  [[nodiscard]] std::uint32_t dropped() const;

 private:
  mutable std::mutex mutex_;
  std::array<protocol::LogMessage, kCapacity> items_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
  std::uint32_t dropped_ = 0;
};

/// Log sink that forwards firmware log lines to the Pi, where they appear in /rosout.
class ProtocolLogSink final : public logging::Sink {
 public:
  explicit ProtocolLogSink(LogQueue& queue) noexcept : queue_(queue) {}
  void write(logging::Level level, std::string_view message) override;

 private:
  LogQueue& queue_;
};

/// Velocity command and link liveness, written by the comms task, read by the control task.
struct MotionTarget {
  float linear = 0.0F;
  float angular = 0.0F;
  std::uint32_t last_rx_us = 0;  ///< time of the last valid frame from the Pi
  bool received_any = false;
};

/// A command that needs the control task and an acknowledgement.
struct Request {
  protocol::Message message{};
  std::uint8_t sequence = 0;
};

/// What the control task shares with the comms task after every tick.
struct TelemetrySnapshot {
  protocol::Telemetry telemetry{};
  std::optional<protocol::ImuSample> primary_sample;
  std::optional<protocol::ImuSample> secondary_sample;
};

using RequestQueue = SpscQueue<Request, 8>;
using AckQueue = SpscQueue<protocol::Ack, 8>;

/// The comms task's logic, independent of FreeRTOS:
/// - decodes frames from the Pi: CmdVel and Heartbeat update the motion target and link
///   liveness, SetArmed and SetGains are queued for the control task;
/// - sends acknowledgements from the control task as soon as they appear;
/// - sends telemetry at a fixed period, alternating the two IMU samples, and pending log lines.
class CommsService {
 public:
  struct Channels {
    Mailbox<MotionTarget>& motion;
    RequestQueue& requests;
    AckQueue& acks;
    Mailbox<TelemetrySnapshot>& telemetry;
    LogQueue& logs;
  };

  CommsService(Link& link, Channels channels, std::uint32_t telemetry_period_us) noexcept
      : link_(link), channels_(channels), telemetry_period_us_(telemetry_period_us) {}

  /// Runs one iteration. Call every few milliseconds.
  void poll(std::uint32_t now_us) noexcept;

  [[nodiscard]] std::uint32_t requestsDropped() const noexcept { return requests_dropped_; }

 private:
  void onFrame(const protocol::DecodeResult& frame, std::uint32_t now_us) noexcept;

  static constexpr int kMaxLogsPerPoll = 4;

  Link& link_;
  Channels channels_;
  std::uint32_t telemetry_period_us_;
  std::uint32_t last_telemetry_us_ = 0;
  bool telemetry_started_ = false;
  bool send_secondary_next_ = false;
  std::uint32_t requests_dropped_ = 0;
};

}  // namespace robot::app
