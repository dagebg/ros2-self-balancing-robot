#include "robot_bridge/framed_link.hpp"

namespace robot_bridge {

std::optional<std::uint8_t> FramedLink::send(const robot::protocol::Message& message) {
  std::array<std::uint8_t, robot::protocol::kMaxFrameSize> frame{};
  const std::uint8_t sequence = next_sequence_;
  const auto size = robot::protocol::encodeFrame(message, sequence, frame.data(), frame.size());
  if (!size || port_.write(frame.data(), *size) != *size) {
    return std::nullopt;
  }
  ++next_sequence_;
  return sequence;
}

std::vector<robot::protocol::DecodeResult> FramedLink::receive() {
  std::vector<robot::protocol::DecodeResult> frames;
  std::array<std::uint8_t, 256> buffer{};
  std::size_t count = 0;
  while ((count = port_.read(buffer.data(), buffer.size())) > 0) {
    for (std::size_t i = 0; i < count; ++i) {
      if (auto result = parser_.push(buffer[i]); result && result->ok()) {
        sequence_tracker_.observe(result->sequence);
        frames.push_back(*result);
      }
    }
  }
  return frames;
}

}  // namespace robot_bridge
