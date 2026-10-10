#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "robot_protocol/byte_io.hpp"
#include "robot_protocol/cobs.hpp"
#include "robot_protocol/codec.hpp"
#include "robot_protocol/crc16.hpp"

namespace {

namespace rp = robot::protocol;
using Bytes = std::vector<std::uint8_t>;

/// Encodes a message and returns the frame without its trailing delimiter.
Bytes frameOf(const rp::Message& message, std::uint8_t sequence = 0) {
  std::array<std::uint8_t, rp::kMaxFrameSize> buffer{};
  const auto size = rp::encodeFrame(message, sequence, buffer.data(), buffer.size());
  EXPECT_TRUE(size.has_value());
  EXPECT_EQ(buffer.at(size.value_or(1) - 1), rp::kFrameDelimiter);
  return Bytes(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(size.value_or(1) - 1));
}

/// Builds a frame from raw header fields and body, with a valid (or deliberately wrong) CRC.
Bytes rawFrame(std::uint8_t version, std::uint8_t type, std::uint8_t sequence, const Bytes& body,
               bool corrupt_crc = false) {
  Bytes packet{version, type, sequence};
  packet.insert(packet.end(), body.begin(), body.end());
  std::uint16_t crc = rp::crc16(packet.data(), packet.size());
  if (corrupt_crc) {
    crc ^= 0x0001U;
  }
  packet.push_back(static_cast<std::uint8_t>(crc & 0xFFU));
  packet.push_back(static_cast<std::uint8_t>(crc >> 8U));

  Bytes encoded(rp::cobsMaxEncodedSize(packet.size()));
  const auto size = rp::cobsEncode(packet.data(), packet.size(), encoded.data(), encoded.size());
  encoded.resize(size.value_or(0));
  return encoded;
}

rp::DecodeResult decode(const Bytes& frame) { return rp::decodeFrame(frame.data(), frame.size()); }

std::uint8_t typeId(rp::MessageType type) { return static_cast<std::uint8_t>(type); }

// --- Round trips -------------------------------------------------------------------------------

rp::Telemetry sampleTelemetry() {
  rp::Telemetry t;
  t.timestamp_us = 0xFEDCBA98;
  t.state = rp::RobotState::kBalancing;
  t.faults = rp::fault::kLinkTimeout | rp::fault::kLoopOverrun;
  t.roll = 0.01F;
  t.pitch = -0.05F;
  t.pitch_rate = 1.5F;
  t.yaw_rate = -0.25F;
  t.left_ticks = -2000000;
  t.right_ticks = 2000000;
  t.left_wheel_velocity = 12.5F;
  t.right_wheel_velocity = -12.5F;
  t.left_duty = 0.75F;
  t.right_duty = -1.0F;
  t.max_loop_time_us = 4321;
  return t;
}

rp::ImuSample sampleImu() {
  rp::ImuSample s;
  s.timestamp_us = 123456;
  s.sensor = rp::ImuSensor::kMpu6050;
  s.accel = {0.1F, -0.2F, 9.81F};
  s.gyro = {0.01F, 0.02F, -0.03F};
  return s;
}

class RoundTripTest : public ::testing::TestWithParam<rp::Message> {};

std::string messageName(const ::testing::TestParamInfo<rp::Message>& info) {
  constexpr std::array<const char*, std::variant_size_v<rp::Message>> kNames{
      "Heartbeat", "CmdVel", "SetArmed", "SetGains", "Telemetry", "ImuSample", "Log", "Ack"};
  return std::string(kNames.at(info.param.index())) + "_" + std::to_string(info.index);
}

TEST_P(RoundTripTest, DecodesToTheSameMessage) {
  const rp::Message& original = GetParam();
  const rp::DecodeResult result = decode(frameOf(original, 200));

  ASSERT_TRUE(result.ok()) << rp::toString(result.error);
  EXPECT_EQ(result.sequence, 200);
  ASSERT_EQ(result.message.index(), original.index());
  EXPECT_TRUE(result.message == original);
  EXPECT_EQ(rp::typeOf(result.message), rp::typeOf(original));
}

INSTANTIATE_TEST_SUITE_P(
    AllMessageTypes, RoundTripTest,
    ::testing::Values(rp::Message{rp::Heartbeat{}}, rp::Message{rp::CmdVel{0.4F, -1.2F}},
                      rp::Message{rp::SetArmed{true}}, rp::Message{rp::SetArmed{false}},
                      rp::Message{rp::SetGains{rp::GainLoop::kVelocity, 1.0F, 0.5F, 0.01F}},
                      rp::Message{sampleTelemetry()}, rp::Message{sampleImu()},
                      rp::Message{rp::LogMessage::make(rp::LogLevel::kWarn, "IMU 0x68 missing")},
                      rp::Message{rp::LogMessage::make(rp::LogLevel::kDebug, "")},
                      rp::Message{
                          rp::Ack{rp::MessageType::kSetArmed, 17, rp::AckStatus::kRejected}}),
    messageName);

// --- Wire format -------------------------------------------------------------------------------

TEST(CodecTest, CmdVelHasStableWireFormat) {
  // Guards against accidental format changes: both ends must agree byte for byte.
  const Bytes frame = frameOf(rp::CmdVel{0.5F, -1.0F}, 7);
  std::array<std::uint8_t, rp::kMaxPacketSize> packet{};
  const auto size = rp::cobsDecode(frame.data(), frame.size(), packet.data(), packet.size());
  ASSERT_TRUE(size.has_value());

  const Bytes expected_prefix{
      0x01, 0x02, 0x07,        // version 1, type CmdVel, sequence 7
      0x00, 0x00, 0x00, 0x3F,  // 0.5f, little-endian
      0x00, 0x00, 0x80, 0xBF,  // -1.0f, little-endian
  };
  ASSERT_EQ(*size, expected_prefix.size() + rp::kCrcSize);
  EXPECT_TRUE(std::equal(expected_prefix.begin(), expected_prefix.end(), packet.begin()));
}

TEST(CodecTest, TelemetryFitsWellWithinMaximumFrame) {
  EXPECT_LT(frameOf(sampleTelemetry()).size() + 1, rp::kMaxFrameSize);
}

TEST(CodecTest, LongestLogMessageFitsExactlyIntoMaximumFrame) {
  const std::string text(rp::kMaxLogTextLength, 'x');
  const Bytes frame = frameOf(rp::LogMessage::make(rp::LogLevel::kError, text));
  EXPECT_LE(frame.size() + 1, rp::kMaxFrameSize);
  EXPECT_TRUE(decode(frame).ok());
}

TEST(CodecTest, LogTextIsTruncatedToMaximumLength) {
  const std::string text(rp::kMaxLogTextLength + 20, 'y');
  const auto log = rp::LogMessage::make(rp::LogLevel::kInfo, text);
  EXPECT_EQ(log.view().size(), rp::kMaxLogTextLength);
}

TEST(CodecTest, EncodeFailsWhenBufferTooSmall) {
  std::array<std::uint8_t, 4> buffer{};
  EXPECT_FALSE(rp::encodeFrame(sampleTelemetry(), 0, buffer.data(), buffer.size()).has_value());
  EXPECT_FALSE(rp::encodeFrame(rp::Heartbeat{}, 0, buffer.data(), 0).has_value());
}

// --- Decode errors -----------------------------------------------------------------------------

TEST(CodecTest, RejectsInvalidCobs) {
  EXPECT_EQ(decode({0x05, 0x01}).error, rp::DecodeError::kCobs);
}

TEST(CodecTest, RejectsFramesShorterThanHeaderAndCrc) {
  EXPECT_EQ(decode({0x03, 0x01, 0x02}).error, rp::DecodeError::kTooShort);
}

TEST(CodecTest, RejectsCorruptedCrc) {
  const Bytes frame = rawFrame(1, typeId(rp::MessageType::kHeartbeat), 0, {}, true);
  EXPECT_EQ(decode(frame).error, rp::DecodeError::kCrcMismatch);
}

TEST(CodecTest, DetectsAnySingleBitFlipInAFrame) {
  const Bytes frame = frameOf(sampleTelemetry(), 3);
  for (std::size_t byte = 0; byte < frame.size(); ++byte) {
    for (unsigned bit = 0; bit < 8; ++bit) {
      Bytes corrupted = frame;
      corrupted[byte] = static_cast<std::uint8_t>(corrupted[byte] ^ (1U << bit));
      EXPECT_FALSE(decode(corrupted).ok()) << "byte " << byte << " bit " << bit;
    }
  }
}

TEST(CodecTest, RejectsOtherProtocolVersions) {
  const Bytes frame = rawFrame(2, typeId(rp::MessageType::kHeartbeat), 0, {});
  EXPECT_EQ(decode(frame).error, rp::DecodeError::kUnsupportedVersion);
}

TEST(CodecTest, RejectsUnknownMessageTypes) {
  EXPECT_EQ(decode(rawFrame(1, 0x7E, 0, {})).error, rp::DecodeError::kUnknownType);
}

TEST(CodecTest, RejectsBodiesOfTheWrongLength) {
  const std::uint8_t cmd_vel = typeId(rp::MessageType::kCmdVel);
  EXPECT_EQ(decode(rawFrame(1, cmd_vel, 0, Bytes(7))).error, rp::DecodeError::kBadLength);
  EXPECT_EQ(decode(rawFrame(1, cmd_vel, 0, Bytes(9))).error, rp::DecodeError::kBadLength);
  const std::uint8_t heartbeat = typeId(rp::MessageType::kHeartbeat);
  EXPECT_EQ(decode(rawFrame(1, heartbeat, 0, {0x00})).error, rp::DecodeError::kBadLength);
  const std::uint8_t log = typeId(rp::MessageType::kLog);
  EXPECT_EQ(decode(rawFrame(1, log, 0, {})).error, rp::DecodeError::kBadLength);
  const std::uint8_t ack = typeId(rp::MessageType::kAck);
  EXPECT_EQ(decode(rawFrame(1, ack, 0, {0x03, 0x00})).error, rp::DecodeError::kBadLength);
}

TEST(CodecTest, RejectsTooLongLogText) {
  Bytes body{0x01};
  body.resize(1 + rp::kMaxLogTextLength + 1, 'z');
  const Bytes frame = rawFrame(1, typeId(rp::MessageType::kLog), 0, body);
  EXPECT_NE(decode(frame).error, rp::DecodeError::kNone);
}

TEST(CodecTest, RejectsOutOfRangeEnumValues) {
  EXPECT_EQ(decode(rawFrame(1, typeId(rp::MessageType::kSetArmed), 0, {0x02})).error,
            rp::DecodeError::kBadValue);
  EXPECT_EQ(decode(rawFrame(1, typeId(rp::MessageType::kSetGains), 0,
                            Bytes{0x03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}))
                .error,
            rp::DecodeError::kBadValue);
  EXPECT_EQ(decode(rawFrame(1, typeId(rp::MessageType::kLog), 0, {0x04, 'a'})).error,
            rp::DecodeError::kBadValue);
  EXPECT_EQ(decode(rawFrame(1, typeId(rp::MessageType::kAck), 0, {0x7E, 0x00, 0x00})).error,
            rp::DecodeError::kBadValue);
  EXPECT_EQ(decode(rawFrame(1, typeId(rp::MessageType::kAck), 0, {0x03, 0x00, 0x03})).error,
            rp::DecodeError::kBadValue);

  // Telemetry with state byte 4 (body otherwise valid).
  std::array<std::uint8_t, rp::kMaxPacketSize> body{};
  rp::ByteWriter writer(body.data(), body.size());
  writer.writeU32(0);
  writer.writeU8(4);
  for (int i = 0; i < 22; ++i) {
    writer.writeU16(0);
  }
  const Bytes telemetry_body(body.begin(),
                             body.begin() + static_cast<std::ptrdiff_t>(writer.size()));
  EXPECT_EQ(decode(rawFrame(1, typeId(rp::MessageType::kTelemetry), 0, telemetry_body)).error,
            rp::DecodeError::kBadValue);

  // ImuSample with sensor id 2.
  Bytes imu_body(4 + 1 + 24, 0);
  imu_body[4] = 2;
  EXPECT_EQ(decode(rawFrame(1, typeId(rp::MessageType::kImuSample), 0, imu_body)).error,
            rp::DecodeError::kBadValue);
}

TEST(CodecTest, RejectsOversizedFrames) {
  const Bytes frame(rp::kMaxFrameSize + 10, 0x01);
  EXPECT_EQ(decode(frame).error, rp::DecodeError::kBadLength);
}

TEST(CodecTest, EveryErrorHasADescription) {
  for (auto error : {rp::DecodeError::kNone, rp::DecodeError::kCobs, rp::DecodeError::kTooShort,
                     rp::DecodeError::kCrcMismatch, rp::DecodeError::kUnsupportedVersion,
                     rp::DecodeError::kUnknownType, rp::DecodeError::kBadLength,
                     rp::DecodeError::kBadValue, rp::DecodeError::kOverflow}) {
    EXPECT_FALSE(rp::toString(error).empty());
  }
}

}  // namespace
