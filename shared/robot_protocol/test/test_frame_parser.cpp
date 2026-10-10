#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <random>
#include <string_view>
#include <vector>

#include "robot_protocol/codec.hpp"
#include "robot_protocol/frame_parser.hpp"

namespace {

namespace rp = robot::protocol;
using Bytes = std::vector<std::uint8_t>;

Bytes frameOf(const rp::Message& message, std::uint8_t sequence) {
  std::array<std::uint8_t, rp::kMaxFrameSize> buffer{};
  const auto size = rp::encodeFrame(message, sequence, buffer.data(), buffer.size());
  return Bytes(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(size.value_or(0)));
}

std::vector<rp::DecodeResult> feed(rp::FrameParser& parser, const Bytes& stream) {
  std::vector<rp::DecodeResult> results;
  for (const std::uint8_t byte : stream) {
    if (auto result = parser.push(byte)) {
      results.push_back(*result);
    }
  }
  return results;
}

void append(Bytes& stream, const Bytes& more) {
  stream.insert(stream.end(), more.begin(), more.end());
}

TEST(FrameParserTest, ExtractsConsecutiveFrames) {
  rp::FrameParser parser;
  Bytes stream;
  for (std::uint8_t seq = 0; seq < 5; ++seq) {
    append(stream, frameOf(rp::CmdVel{0.1F * seq, 0.0F}, seq));
  }

  const auto results = feed(parser, stream);

  ASSERT_EQ(results.size(), 5U);
  for (std::uint8_t seq = 0; seq < 5; ++seq) {
    EXPECT_TRUE(results[seq].ok());
    EXPECT_EQ(results[seq].sequence, seq);
  }
  EXPECT_EQ(parser.stats().frames_ok, 5U);
}

TEST(FrameParserTest, IgnoresEmptyFrames) {
  rp::FrameParser parser;
  Bytes stream{0x00, 0x00};
  append(stream, frameOf(rp::Heartbeat{}, 1));
  stream.push_back(0x00);

  const auto results = feed(parser, stream);

  ASSERT_EQ(results.size(), 1U);
  EXPECT_TRUE(results[0].ok());
}

TEST(FrameParserTest, RecoversAfterBootRomTextWithoutZeroBytes) {
  // The ESP32 boot ROM prints text on UART0 at reset. It contains no 0x00, so it merges with the
  // first frame; everything after the first delimiter must decode.
  constexpr std::string_view kBootText = "ets Jun  8 2016 00:22:57\r\nrst:0x1 (POWERON_RESET)\r\n";
  rp::FrameParser parser;
  Bytes stream(kBootText.begin(), kBootText.end());
  stream.push_back(0x00);  // firmware emits a resynchronization delimiter at start-up
  append(stream, frameOf(rp::Heartbeat{}, 1));
  append(stream, frameOf(rp::Heartbeat{}, 2));

  const auto results = feed(parser, stream);

  ASSERT_EQ(results.size(), 3U);
  EXPECT_FALSE(results[0].ok());
  EXPECT_TRUE(results[1].ok());
  EXPECT_TRUE(results[2].ok());
}

TEST(FrameParserTest, ReportsOverflowOnceAndResynchronizes) {
  rp::FrameParser parser;
  Bytes stream(rp::kMaxFrameSize * 3, 0x55);  // long run without a delimiter
  stream.push_back(0x00);
  append(stream, frameOf(rp::SetArmed{true}, 9));

  const auto results = feed(parser, stream);

  ASSERT_EQ(results.size(), 2U);
  EXPECT_EQ(results[0].error, rp::DecodeError::kOverflow);
  EXPECT_TRUE(results[1].ok());
  EXPECT_EQ(parser.stats().overflows, 1U);
}

TEST(FrameParserTest, FrameOfExactlyMaximumSizeIsAccepted) {
  rp::FrameParser parser;
  const std::string text(rp::kMaxLogTextLength, 'q');
  const auto results = feed(parser, frameOf(rp::LogMessage::make(rp::LogLevel::kInfo, text), 0));
  ASSERT_EQ(results.size(), 1U);
  EXPECT_TRUE(results[0].ok());
}

TEST(FrameParserTest, CountsErrorsByCategory) {
  rp::FrameParser parser;
  Bytes stream{0x05, 0x01, 0x00};  // invalid COBS
  Bytes corrupted = frameOf(rp::Heartbeat{}, 0);
  corrupted[1] ^= 0x10U;  // flip a bit inside the packet
  append(stream, corrupted);
  append(stream, {0x02, 0x01, 0x00});  // valid COBS, too short

  (void)feed(parser, stream);

  EXPECT_EQ(parser.stats().cobs_errors, 1U);
  EXPECT_EQ(parser.stats().crc_errors + parser.stats().cobs_errors + parser.stats().other_errors,
            3U);
}

TEST(FrameParserTest, ResetDropsPartialFrame) {
  rp::FrameParser parser;
  const Bytes frame = frameOf(rp::Heartbeat{}, 4);
  (void)feed(parser, Bytes(frame.begin(), frame.begin() + 2));
  parser.reset();

  const auto results = feed(parser, frame);

  ASSERT_EQ(results.size(), 1U);
  EXPECT_TRUE(results[0].ok());
}

// Fuzz-style: random garbage bursts (each followed by a delimiter) interleaved with valid frames.
// Every valid frame must come through, and garbage must never crash the parser.
TEST(FrameParserTest, RecoversEveryFrameFromRandomGarbage) {
  for (unsigned seed = 0; seed < 50; ++seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> byte(0, 255);
    std::uniform_int_distribution<int> burst(0, 300);

    rp::FrameParser parser;
    Bytes stream;
    constexpr int kFrames = 40;
    for (int i = 0; i < kFrames; ++i) {
      const int garbage = burst(rng);
      for (int g = 0; g < garbage; ++g) {
        stream.push_back(static_cast<std::uint8_t>(byte(rng)));
      }
      stream.push_back(0x00);
      append(stream,
             frameOf(rp::CmdVel{static_cast<float>(i), 1.0F}, static_cast<std::uint8_t>(i)));
    }

    int recovered = 0;
    for (const auto& result : feed(parser, stream)) {
      if (result.ok() && std::holds_alternative<rp::CmdVel>(result.message) &&
          std::get<rp::CmdVel>(result.message).angular_z == 1.0F) {
        ++recovered;
      }
    }
    EXPECT_GE(recovered, kFrames) << "seed " << seed;
  }
}

TEST(SequenceTrackerTest, CountsGapsIncludingWrapAround) {
  rp::SequenceTracker tracker;
  EXPECT_EQ(tracker.observe(250), 0);
  EXPECT_EQ(tracker.observe(251), 0);
  EXPECT_EQ(tracker.observe(254), 2);
  EXPECT_EQ(tracker.observe(1), 2);  // 255 and 0 lost
  EXPECT_EQ(tracker.lost(), 4U);
  EXPECT_EQ(tracker.received(), 4U);
}

}  // namespace
