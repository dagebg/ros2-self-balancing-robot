#include "app/link.hpp"

#include <variant>

namespace robot::app {

// --- Link --------------------------------------------------------------------------------------

bool Link::send(const protocol::Message& message) noexcept {
  std::array<std::uint8_t, protocol::kMaxFrameSize> frame{};
  const auto size = protocol::encodeFrame(message, tx_sequence_, frame.data(), frame.size());
  if (!size) {
    return false;
  }
  ++tx_sequence_;
  if (stream_.write(frame.data(), *size) != *size) {
    ++tx_incomplete_;
    return false;
  }
  return true;
}

void Link::sendResync() noexcept {
  const std::uint8_t delimiter = protocol::kFrameDelimiter;
  stream_.write(&delimiter, 1);
}

// --- LogQueue ----------------------------------------------------------------------------------

void LogQueue::push(protocol::LogMessage message) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (size_ == kCapacity) {
    ++dropped_;
    return;
  }
  items_[(head_ + size_) % kCapacity] = message;
  ++size_;
}

std::optional<protocol::LogMessage> LogQueue::pop() {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (size_ == 0) {
    return std::nullopt;
  }
  const protocol::LogMessage message = items_[head_];
  head_ = (head_ + 1) % kCapacity;
  --size_;
  return message;
}

std::uint32_t LogQueue::dropped() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return dropped_;
}

void ProtocolLogSink::write(logging::Level level, std::string_view message) {
  // logging::Level and protocol::LogLevel use the same order (debug, info, warn, error).
  queue_.push(protocol::LogMessage::make(static_cast<protocol::LogLevel>(level), message));
}

// --- CommsService ------------------------------------------------------------------------------

void CommsService::onFrame(const protocol::DecodeResult& frame, std::uint32_t now_us) noexcept {
  MotionTarget target = channels_.motion.read();
  target.last_rx_us = now_us;
  target.received_any = true;
  if (const auto* cmd = std::get_if<protocol::CmdVel>(&frame.message)) {
    target.linear = cmd->linear_x;
    target.angular = cmd->angular_z;
  } else if (std::holds_alternative<protocol::SetArmed>(frame.message) ||
             std::holds_alternative<protocol::SetGains>(frame.message)) {
    if (!channels_.requests.push({frame.message, frame.sequence})) {
      ++requests_dropped_;
    }
  }
  channels_.motion.publish(target);
}

void CommsService::poll(std::uint32_t now_us) noexcept {
  link_.poll([this, now_us](const protocol::DecodeResult& frame) { onFrame(frame, now_us); });

  while (const auto ack = channels_.acks.pop()) {
    link_.send(*ack);
  }

  if (!telemetry_started_ || now_us - last_telemetry_us_ >= telemetry_period_us_) {
    telemetry_started_ = true;
    last_telemetry_us_ = now_us;
    const TelemetrySnapshot snapshot = channels_.telemetry.read();
    link_.send(snapshot.telemetry);
    const auto& sample = send_secondary_next_ ? snapshot.secondary_sample : snapshot.primary_sample;
    if (sample) {
      link_.send(*sample);
    }
    send_secondary_next_ = !send_secondary_next_;
  }

  for (int i = 0; i < kMaxLogsPerPoll; ++i) {
    const auto log = channels_.logs.pop();
    if (!log) {
      break;
    }
    link_.send(*log);
  }
}

}  // namespace robot::app
