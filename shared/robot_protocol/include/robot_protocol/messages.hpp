#pragma once

/// @file
/// Messages exchanged between the ESP32 and the Raspberry Pi (protocol version 1).
///
/// Units are SI throughout: metres, radians, seconds. Angles follow REP-103: x forward, y left,
/// z up; positive pitch tilts the robot's nose down (forward).

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <variant>

namespace robot::protocol {

/// Bumped on any incompatible change to the wire format.
inline constexpr std::uint8_t kProtocolVersion = 1;

/// Message type identifiers. 0x01-0x3F: Pi -> ESP32. 0x40-0x7F: ESP32 -> Pi.
enum class MessageType : std::uint8_t {
  kHeartbeat = 0x01,
  kCmdVel = 0x02,
  kSetArmed = 0x03,
  kSetGains = 0x04,
  kTelemetry = 0x40,
  kImuSample = 0x41,
  kLog = 0x42,
  kAck = 0x43,
};

/// High-level state of the robot's safety state machine.
enum class RobotState : std::uint8_t {
  kDisarmed = 0,   ///< Motors off; the robot may be handled.
  kArmed = 1,      ///< Armed and waiting to be upright before balancing.
  kBalancing = 2,  ///< Balance loop is driving the motors.
  kFault = 3,      ///< A fault latched; motors off until disarmed.
};

/// Bit flags reported in Telemetry::faults.
namespace fault {
inline constexpr std::uint16_t kTiltLimit = 1U << 0U;        ///< Pitch beyond the cut-off angle.
inline constexpr std::uint16_t kImuFailure = 1U << 1U;       ///< An IMU stopped responding.
inline constexpr std::uint16_t kLinkTimeout = 1U << 2U;      ///< No message from the Pi in time.
inline constexpr std::uint16_t kImuDisagreement = 1U << 3U;  ///< The two IMUs disagree.
inline constexpr std::uint16_t kLoopOverrun = 1U << 4U;      ///< Control loop missed its period.
}  // namespace fault

/// Controller loops whose gains can be tuned at runtime.
enum class GainLoop : std::uint8_t { kPitch = 0, kVelocity = 1, kYawRate = 2 };

/// Result of a command that the ESP32 acknowledges.
enum class AckStatus : std::uint8_t {
  kOk = 0,        ///< Applied.
  kRejected = 1,  ///< Valid, but refused in the current state (e.g. arming while tilted).
  kInvalid = 2,   ///< Malformed or out-of-range values.
};

enum class LogLevel : std::uint8_t { kDebug = 0, kInfo = 1, kWarn = 2, kError = 3 };

enum class ImuSensor : std::uint8_t { kMpu9250 = 0, kMpu6050 = 1 };

/// Keeps the link alive when no other command is due. Pi -> ESP32.
struct Heartbeat {};

/// Body velocity target. Pi -> ESP32.
struct CmdVel {
  float linear_x = 0.0F;   ///< Forward speed [m/s].
  float angular_z = 0.0F;  ///< Yaw rate, counter-clockwise positive [rad/s].
};

/// Arms or disarms the motors. Pi -> ESP32; answered with an Ack.
struct SetArmed {
  bool armed = false;
};

/// Replaces the PID gains of one controller loop. Pi -> ESP32; answered with an Ack.
struct SetGains {
  GainLoop loop = GainLoop::kPitch;
  float kp = 0.0F;
  float ki = 0.0F;
  float kd = 0.0F;
};

/// Controller state, sent at the telemetry rate. ESP32 -> Pi.
struct Telemetry {
  std::uint32_t timestamp_us = 0;  ///< ESP32 monotonic time; wraps after ~71 minutes.
  RobotState state = RobotState::kDisarmed;
  std::uint16_t faults = 0;            ///< Bitwise OR of robot::protocol::fault flags.
  float roll = 0.0F;                   ///< [rad]
  float pitch = 0.0F;                  ///< [rad]
  float pitch_rate = 0.0F;             ///< [rad/s]
  float yaw_rate = 0.0F;               ///< [rad/s]
  std::int32_t left_ticks = 0;         ///< Cumulative encoder count, x4 decoding.
  std::int32_t right_ticks = 0;        ///< Cumulative encoder count, x4 decoding.
  float left_wheel_velocity = 0.0F;    ///< [rad/s]
  float right_wheel_velocity = 0.0F;   ///< [rad/s]
  float left_duty = 0.0F;              ///< Motor command in [-1, 1].
  float right_duty = 0.0F;             ///< Motor command in [-1, 1].
  std::uint16_t max_loop_time_us = 0;  ///< Longest control-loop iteration since last report.
};

/// One calibrated IMU reading. ESP32 -> Pi.
struct ImuSample {
  std::uint32_t timestamp_us = 0;
  ImuSensor sensor = ImuSensor::kMpu9250;
  std::array<float, 3> accel{};  ///< [m/s^2], robot body frame.
  std::array<float, 3> gyro{};   ///< [rad/s], robot body frame.
};

/// Maximum length of LogMessage text in bytes.
inline constexpr std::size_t kMaxLogTextLength = 96;

/// A firmware log line. ESP32 -> Pi.
struct LogMessage {
  LogLevel level = LogLevel::kInfo;
  std::array<char, kMaxLogTextLength> text{};
  std::uint8_t length = 0;

  /// Creates a message; text longer than kMaxLogTextLength is truncated.
  [[nodiscard]] static LogMessage make(LogLevel level, std::string_view message) noexcept {
    LogMessage log;
    log.level = level;
    log.length = static_cast<std::uint8_t>(std::min(message.size(), kMaxLogTextLength));
    std::copy(message.begin(), message.begin() + log.length, log.text.begin());
    return log;
  }

  [[nodiscard]] std::string_view view() const noexcept { return {text.data(), length}; }
};

/// Acknowledges a command. ESP32 -> Pi.
struct Ack {
  MessageType acked_type = MessageType::kHeartbeat;
  std::uint8_t acked_sequence = 0;
  AckStatus status = AckStatus::kOk;
};

/// Any protocol message.
using Message =
    std::variant<Heartbeat, CmdVel, SetArmed, SetGains, Telemetry, ImuSample, LogMessage, Ack>;

/// The wire type identifier of a message.
[[nodiscard]] MessageType typeOf(const Message& message) noexcept;

// Equality, mainly for tests. Floats compare bitwise-equal after a round trip.
bool operator==(const Heartbeat& a, const Heartbeat& b) noexcept;
bool operator==(const CmdVel& a, const CmdVel& b) noexcept;
bool operator==(const SetArmed& a, const SetArmed& b) noexcept;
bool operator==(const SetGains& a, const SetGains& b) noexcept;
bool operator==(const Telemetry& a, const Telemetry& b) noexcept;
bool operator==(const ImuSample& a, const ImuSample& b) noexcept;
bool operator==(const LogMessage& a, const LogMessage& b) noexcept;
bool operator==(const Ack& a, const Ack& b) noexcept;

}  // namespace robot::protocol
