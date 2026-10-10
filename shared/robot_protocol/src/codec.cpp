#include "robot_protocol/codec.hpp"

#include <array>

#include "robot_protocol/byte_io.hpp"
#include "robot_protocol/crc16.hpp"

namespace robot::protocol {

namespace {

// --- Body serialization ----------------------------------------------------------------------

void writeBody(const Heartbeat& /*message*/, ByteWriter& /*writer*/) noexcept {}

void writeBody(const CmdVel& m, ByteWriter& w) noexcept {
  w.writeF32(m.linear_x);
  w.writeF32(m.angular_z);
}

void writeBody(const SetArmed& m, ByteWriter& w) noexcept { w.writeU8(m.armed ? 1 : 0); }

void writeBody(const SetGains& m, ByteWriter& w) noexcept {
  w.writeU8(static_cast<std::uint8_t>(m.loop));
  w.writeF32(m.kp);
  w.writeF32(m.ki);
  w.writeF32(m.kd);
}

void writeBody(const Telemetry& m, ByteWriter& w) noexcept {
  w.writeU32(m.timestamp_us);
  w.writeU8(static_cast<std::uint8_t>(m.state));
  w.writeU16(m.faults);
  w.writeF32(m.roll);
  w.writeF32(m.pitch);
  w.writeF32(m.pitch_rate);
  w.writeF32(m.yaw_rate);
  w.writeI32(m.left_ticks);
  w.writeI32(m.right_ticks);
  w.writeF32(m.left_wheel_velocity);
  w.writeF32(m.right_wheel_velocity);
  w.writeF32(m.left_duty);
  w.writeF32(m.right_duty);
  w.writeU16(m.max_loop_time_us);
}

void writeBody(const ImuSample& m, ByteWriter& w) noexcept {
  w.writeU32(m.timestamp_us);
  w.writeU8(static_cast<std::uint8_t>(m.sensor));
  for (const float value : m.accel) {
    w.writeF32(value);
  }
  for (const float value : m.gyro) {
    w.writeF32(value);
  }
}

void writeBody(const LogMessage& m, ByteWriter& w) noexcept {
  w.writeU8(static_cast<std::uint8_t>(m.level));
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): char and uint8_t alias.
  w.writeBytes(reinterpret_cast<const std::uint8_t*>(m.text.data()), m.length);
}

void writeBody(const Ack& m, ByteWriter& w) noexcept {
  w.writeU8(static_cast<std::uint8_t>(m.acked_type));
  w.writeU8(m.acked_sequence);
  w.writeU8(static_cast<std::uint8_t>(m.status));
}

// --- Enum validation -------------------------------------------------------------------------

constexpr std::array<MessageType, 8> kMessageTypes{
    MessageType::kHeartbeat, MessageType::kCmdVel,    MessageType::kSetArmed,
    MessageType::kSetGains,  MessageType::kTelemetry, MessageType::kImuSample,
    MessageType::kLog,       MessageType::kAck,
};

/// Maps a wire byte to a message type without casting unchecked values to the enum.
std::optional<MessageType> toMessageType(std::uint8_t value) noexcept {
  for (const MessageType type : kMessageTypes) {
    if (static_cast<std::uint8_t>(type) == value) {
      return type;
    }
  }
  return std::nullopt;
}

/// True if `value` is a valid enumerator of an enum whose values are 0..last.
template <typename Enum>
bool inRange(std::uint8_t value, Enum last) noexcept {
  return value <= static_cast<std::uint8_t>(last);
}

// --- Body parsing ----------------------------------------------------------------------------
// Each parser reads every field before validating any of them, so the caller's length check sees
// the whole body; enum values are validated before they are cast.

DecodeError readBody(ByteReader& /*r*/, Heartbeat& /*m*/) noexcept { return DecodeError::kNone; }

DecodeError readBody(ByteReader& r, CmdVel& m) noexcept {
  m.linear_x = r.readF32();
  m.angular_z = r.readF32();
  return DecodeError::kNone;
}

DecodeError readBody(ByteReader& r, SetArmed& m) noexcept {
  const std::uint8_t armed = r.readU8();
  if (armed > 1) {
    return DecodeError::kBadValue;
  }
  m.armed = armed == 1;
  return DecodeError::kNone;
}

DecodeError readBody(ByteReader& r, SetGains& m) noexcept {
  const std::uint8_t loop = r.readU8();
  m.kp = r.readF32();
  m.ki = r.readF32();
  m.kd = r.readF32();
  if (!inRange(loop, GainLoop::kYawRate)) {
    return DecodeError::kBadValue;
  }
  m.loop = static_cast<GainLoop>(loop);
  return DecodeError::kNone;
}

DecodeError readBody(ByteReader& r, Telemetry& m) noexcept {
  m.timestamp_us = r.readU32();
  const std::uint8_t state = r.readU8();
  m.faults = r.readU16();
  m.roll = r.readF32();
  m.pitch = r.readF32();
  m.pitch_rate = r.readF32();
  m.yaw_rate = r.readF32();
  m.left_ticks = r.readI32();
  m.right_ticks = r.readI32();
  m.left_wheel_velocity = r.readF32();
  m.right_wheel_velocity = r.readF32();
  m.left_duty = r.readF32();
  m.right_duty = r.readF32();
  m.max_loop_time_us = r.readU16();
  if (!inRange(state, RobotState::kFault)) {
    return DecodeError::kBadValue;
  }
  m.state = static_cast<RobotState>(state);
  return DecodeError::kNone;
}

DecodeError readBody(ByteReader& r, ImuSample& m) noexcept {
  m.timestamp_us = r.readU32();
  const std::uint8_t sensor = r.readU8();
  for (float& value : m.accel) {
    value = r.readF32();
  }
  for (float& value : m.gyro) {
    value = r.readF32();
  }
  if (!inRange(sensor, ImuSensor::kMpu6050)) {
    return DecodeError::kBadValue;
  }
  m.sensor = static_cast<ImuSensor>(sensor);
  return DecodeError::kNone;
}

DecodeError readBody(ByteReader& r, LogMessage& m) noexcept {
  const std::uint8_t level = r.readU8();
  if (!r.ok() || r.remaining() > kMaxLogTextLength) {
    return DecodeError::kBadLength;
  }
  m.length = static_cast<std::uint8_t>(r.remaining());
  for (std::size_t i = 0; i < m.length; ++i) {
    m.text[i] = static_cast<char>(r.current()[i]);
  }
  r.skip(m.length);
  if (!inRange(level, LogLevel::kError)) {
    return DecodeError::kBadValue;
  }
  m.level = static_cast<LogLevel>(level);
  return DecodeError::kNone;
}

DecodeError readBody(ByteReader& r, Ack& m) noexcept {
  const auto type = toMessageType(r.readU8());
  m.acked_sequence = r.readU8();
  const std::uint8_t status = r.readU8();
  if (!r.ok()) {
    return DecodeError::kBadLength;
  }
  if (!type || !inRange(status, AckStatus::kInvalid)) {
    return DecodeError::kBadValue;
  }
  m.acked_type = *type;
  m.status = static_cast<AckStatus>(status);
  return DecodeError::kNone;
}

template <typename T>
DecodeError parseAs(ByteReader& reader, Message& out) noexcept {
  T message{};
  const DecodeError error = readBody(reader, message);
  if (!reader.ok() || reader.remaining() != 0) {
    return DecodeError::kBadLength;
  }
  if (error != DecodeError::kNone) {
    return error;
  }
  out = message;
  return DecodeError::kNone;
}

DecodeError parseBody(MessageType type, ByteReader& reader, Message& out) noexcept {
  switch (type) {
    case MessageType::kHeartbeat:
      return parseAs<Heartbeat>(reader, out);
    case MessageType::kCmdVel:
      return parseAs<CmdVel>(reader, out);
    case MessageType::kSetArmed:
      return parseAs<SetArmed>(reader, out);
    case MessageType::kSetGains:
      return parseAs<SetGains>(reader, out);
    case MessageType::kTelemetry:
      return parseAs<Telemetry>(reader, out);
    case MessageType::kImuSample:
      return parseAs<ImuSample>(reader, out);
    case MessageType::kLog:
      return parseAs<LogMessage>(reader, out);
    case MessageType::kAck:
      return parseAs<Ack>(reader, out);
  }
  return DecodeError::kUnknownType;
}

}  // namespace

std::string_view toString(DecodeError error) noexcept {
  switch (error) {
    case DecodeError::kNone:
      return "none";
    case DecodeError::kCobs:
      return "invalid COBS encoding";
    case DecodeError::kTooShort:
      return "frame too short";
    case DecodeError::kCrcMismatch:
      return "CRC mismatch";
    case DecodeError::kUnsupportedVersion:
      return "unsupported protocol version";
    case DecodeError::kUnknownType:
      return "unknown message type";
    case DecodeError::kBadLength:
      return "body length does not match message type";
    case DecodeError::kBadValue:
      return "field value out of range";
    case DecodeError::kOverflow:
      return "frame exceeds maximum size";
  }
  return "unknown error";
}

// std::visit cannot throw here; see typeOf().
// NOLINTNEXTLINE(bugprone-exception-escape)
std::optional<std::size_t> encodeFrame(const Message& message, std::uint8_t sequence,
                                       std::uint8_t* out, std::size_t capacity) noexcept {
  std::array<std::uint8_t, kMaxPacketSize> packet{};
  ByteWriter writer(packet.data(), packet.size());
  writer.writeU8(kProtocolVersion);
  writer.writeU8(static_cast<std::uint8_t>(typeOf(message)));
  writer.writeU8(sequence);
  std::visit([&writer](const auto& m) { writeBody(m, writer); }, message);
  writer.writeU16(crc16(packet.data(), writer.size()));
  if (!writer.ok() || capacity == 0) {
    return std::nullopt;
  }

  const auto encoded = cobsEncode(packet.data(), writer.size(), out, capacity - 1);
  if (!encoded) {
    return std::nullopt;
  }
  out[*encoded] = kFrameDelimiter;
  return *encoded + 1;
}

DecodeResult decodeFrame(const std::uint8_t* frame, std::size_t size) noexcept {
  DecodeResult result;
  if (size > cobsMaxEncodedSize(kMaxPacketSize)) {
    result.error = DecodeError::kBadLength;
    return result;
  }

  std::array<std::uint8_t, kMaxPacketSize> packet{};
  const auto decoded = cobsDecode(frame, size, packet.data(), packet.size());
  if (!decoded) {
    result.error = DecodeError::kCobs;
    return result;
  }
  const std::size_t length = *decoded;
  if (length < kHeaderSize + kCrcSize) {
    result.error = DecodeError::kTooShort;
    return result;
  }

  const std::size_t crc_offset = length - kCrcSize;
  ByteReader crc_reader(packet.data() + crc_offset, kCrcSize);
  if (crc_reader.readU16() != crc16(packet.data(), crc_offset)) {
    result.error = DecodeError::kCrcMismatch;
    return result;
  }

  ByteReader reader(packet.data(), crc_offset);
  const std::uint8_t version = reader.readU8();
  const auto type = toMessageType(reader.readU8());
  result.sequence = reader.readU8();
  if (version != kProtocolVersion) {
    result.error = DecodeError::kUnsupportedVersion;
    return result;
  }
  if (!type) {
    result.error = DecodeError::kUnknownType;
    return result;
  }
  result.error = parseBody(*type, reader, result.message);
  return result;
}

}  // namespace robot::protocol
