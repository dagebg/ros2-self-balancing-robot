#include "robot_protocol/messages.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace robot::protocol {

namespace {

// Bitwise float comparison: a round trip must preserve the exact bit pattern, including NaN.
std::uint32_t bitsOf(float value) noexcept {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

bool sameBits(float a, float b) noexcept { return bitsOf(a) == bitsOf(b); }

bool sameBits(const std::array<float, 3>& a, const std::array<float, 3>& b) noexcept {
  return std::equal(a.begin(), a.end(), b.begin(), [](float x, float y) { return sameBits(x, y); });
}

}  // namespace

// std::visit throws only for a valueless variant, which cannot occur: every alternative is
// trivially copyable.
// NOLINTNEXTLINE(bugprone-exception-escape)
MessageType typeOf(const Message& message) noexcept {
  return std::visit(
      [](const auto& m) {
        using T = std::decay_t<decltype(m)>;
        if constexpr (std::is_same_v<T, Heartbeat>) {
          return MessageType::kHeartbeat;
        } else if constexpr (std::is_same_v<T, CmdVel>) {
          return MessageType::kCmdVel;
        } else if constexpr (std::is_same_v<T, SetArmed>) {
          return MessageType::kSetArmed;
        } else if constexpr (std::is_same_v<T, SetGains>) {
          return MessageType::kSetGains;
        } else if constexpr (std::is_same_v<T, Telemetry>) {
          return MessageType::kTelemetry;
        } else if constexpr (std::is_same_v<T, ImuSample>) {
          return MessageType::kImuSample;
        } else if constexpr (std::is_same_v<T, LogMessage>) {
          return MessageType::kLog;
        } else {
          static_assert(std::is_same_v<T, Ack>, "unhandled message type");
          return MessageType::kAck;
        }
      },
      message);
}

bool operator==(const Heartbeat& /*a*/, const Heartbeat& /*b*/) noexcept { return true; }

bool operator==(const CmdVel& a, const CmdVel& b) noexcept {
  return sameBits(a.linear_x, b.linear_x) && sameBits(a.angular_z, b.angular_z);
}

bool operator==(const SetArmed& a, const SetArmed& b) noexcept { return a.armed == b.armed; }

bool operator==(const SetGains& a, const SetGains& b) noexcept {
  return a.loop == b.loop && sameBits(a.kp, b.kp) && sameBits(a.ki, b.ki) && sameBits(a.kd, b.kd);
}

bool operator==(const Telemetry& a, const Telemetry& b) noexcept {
  return a.timestamp_us == b.timestamp_us && a.state == b.state && a.faults == b.faults &&
         sameBits(a.roll, b.roll) && sameBits(a.pitch, b.pitch) &&
         sameBits(a.pitch_rate, b.pitch_rate) && sameBits(a.yaw_rate, b.yaw_rate) &&
         a.left_ticks == b.left_ticks && a.right_ticks == b.right_ticks &&
         sameBits(a.left_wheel_velocity, b.left_wheel_velocity) &&
         sameBits(a.right_wheel_velocity, b.right_wheel_velocity) &&
         sameBits(a.left_duty, b.left_duty) && sameBits(a.right_duty, b.right_duty) &&
         a.max_loop_time_us == b.max_loop_time_us;
}

bool operator==(const ImuSample& a, const ImuSample& b) noexcept {
  return a.timestamp_us == b.timestamp_us && a.sensor == b.sensor && sameBits(a.accel, b.accel) &&
         sameBits(a.gyro, b.gyro);
}

bool operator==(const LogMessage& a, const LogMessage& b) noexcept {
  return a.level == b.level && a.view() == b.view();
}

bool operator==(const Ack& a, const Ack& b) noexcept {
  return a.acked_type == b.acked_type && a.acked_sequence == b.acked_sequence &&
         a.status == b.status;
}

}  // namespace robot::protocol
