#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "app/link.hpp"
#include "app/mailbox.hpp"
#include "app/spsc_queue.hpp"
#include "hal_fakes/fakes.hpp"
#include "robot_protocol/codec.hpp"
#include "robot_protocol/frame_parser.hpp"

namespace {

namespace rp = robot::protocol;
using robot::app::AckQueue;
using robot::app::CommsService;
using robot::app::Link;
using robot::app::LogQueue;
using robot::app::Mailbox;
using robot::app::MotionTarget;
using robot::app::RequestQueue;
using robot::app::SpscQueue;
using robot::app::TelemetrySnapshot;
using robot::hal_fakes::FakeByteStream;
using Bytes = std::vector<std::uint8_t>;

Bytes frame(const rp::Message& message, std::uint8_t sequence = 0) {
  std::array<std::uint8_t, rp::kMaxFrameSize> buffer{};
  const auto size = rp::encodeFrame(message, sequence, buffer.data(), buffer.size());
  return Bytes(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(size.value_or(0)));
}

std::vector<rp::Message> decodeAll(const Bytes& bytes) {
  rp::FrameParser parser;
  std::vector<rp::Message> messages;
  for (const auto byte : bytes) {
    if (auto result = parser.push(byte); result && result->ok()) {
      messages.push_back(result->message);
    }
  }
  return messages;
}

// --- Queues
// ----------------------------------------------------------------------------------------

TEST(SpscQueueTest, IsFifoWithUsableCapacityOfSizeMinusOne) {
  SpscQueue<int, 4> queue;
  EXPECT_TRUE(queue.empty());
  EXPECT_TRUE(queue.push(1));
  EXPECT_TRUE(queue.push(2));
  EXPECT_TRUE(queue.push(3));
  EXPECT_FALSE(queue.push(4));  // full
  EXPECT_EQ(queue.pop(), 1);
  EXPECT_TRUE(queue.push(4));
  EXPECT_EQ(queue.pop(), 2);
  EXPECT_EQ(queue.pop(), 3);
  EXPECT_EQ(queue.pop(), 4);
  EXPECT_FALSE(queue.pop().has_value());
}

TEST(SpscQueueTest, TransfersEveryElementBetweenThreadsInOrder) {
  SpscQueue<std::uint32_t, 16> queue;
  constexpr std::uint32_t kCount = 200000;
  std::thread producer([&queue] {
    for (std::uint32_t i = 0; i < kCount;) {
      if (queue.push(i)) {
        ++i;
      }
    }
  });
  std::uint32_t expected = 0;
  bool in_order = true;
  while (expected < kCount) {
    if (const auto value = queue.pop()) {
      in_order = in_order && *value == expected;
      ++expected;
    }
  }
  producer.join();
  EXPECT_TRUE(in_order);
}

TEST(MailboxTest, ReadIfNewReportsOnlyChanges) {
  Mailbox<int> mailbox;
  std::uint32_t seen = 0;
  int value = 0;
  EXPECT_FALSE(mailbox.readIfNew(value, seen));
  mailbox.publish(5);
  EXPECT_TRUE(mailbox.readIfNew(value, seen));
  EXPECT_EQ(value, 5);
  EXPECT_FALSE(mailbox.readIfNew(value, seen));
  EXPECT_EQ(mailbox.read(), 5);
}

TEST(LogQueueTest, DropsWhenFullAndCountsDrops) {
  LogQueue queue;
  for (std::size_t i = 0; i < LogQueue::kCapacity + 3; ++i) {
    queue.push(rp::LogMessage::make(rp::LogLevel::kInfo, "x"));
  }
  EXPECT_EQ(queue.dropped(), 3U);
  std::size_t popped = 0;
  while (queue.pop()) {
    ++popped;
  }
  EXPECT_EQ(popped, LogQueue::kCapacity);
}

// --- CommsService
// ----------------------------------------------------------------------------------

class CommsTest : public ::testing::Test {
 protected:
  CommsTest() : link(stream), service(link, {motion, requests, acks, telemetry, logs}, 20000) {}

  FakeByteStream stream;
  Link link;
  Mailbox<MotionTarget> motion;
  RequestQueue requests;
  AckQueue acks;
  Mailbox<TelemetrySnapshot> telemetry;
  LogQueue logs;
  CommsService service;
};

TEST_F(CommsTest, CmdVelUpdatesMotionTargetAndLiveness) {
  stream.receive(frame(rp::CmdVel{0.12F, -0.5F}));
  service.poll(1000);
  const MotionTarget target = motion.read();
  EXPECT_FLOAT_EQ(target.linear, 0.12F);
  EXPECT_FLOAT_EQ(target.angular, -0.5F);
  EXPECT_TRUE(target.received_any);
  EXPECT_EQ(target.last_rx_us, 1000U);
}

TEST_F(CommsTest, HeartbeatKeepsLinkAliveWithoutChangingTargets) {
  stream.receive(frame(rp::CmdVel{0.1F, 0.0F}));
  service.poll(1000);
  stream.receive(frame(rp::Heartbeat{}));
  service.poll(2000);
  EXPECT_EQ(motion.read().last_rx_us, 2000U);
  EXPECT_FLOAT_EQ(motion.read().linear, 0.1F);
}

TEST_F(CommsTest, CommandsNeedingAcksAreQueuedForTheControlTask) {
  stream.receive(frame(rp::SetArmed{true}, 42));
  stream.receive(frame(rp::SetGains{rp::GainLoop::kVelocity, 1.0F, 0.1F, 0.0F}, 43));
  service.poll(0);

  const auto first = requests.pop();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->sequence, 42);
  EXPECT_TRUE(std::holds_alternative<rp::SetArmed>(first->message));
  const auto second = requests.pop();
  ASSERT_TRUE(second.has_value());
  EXPECT_TRUE(std::holds_alternative<rp::SetGains>(second->message));
}

TEST_F(CommsTest, SendsAcksFromTheControlTask) {
  acks.push({rp::MessageType::kSetArmed, 42, rp::AckStatus::kRejected});
  service.poll(0);
  bool found = false;
  for (const auto& message : decodeAll(stream.tx)) {
    if (const auto* ack = std::get_if<rp::Ack>(&message)) {
      found = ack->acked_sequence == 42 && ack->status == rp::AckStatus::kRejected;
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(CommsTest, SendsTelemetryAtItsPeriodAlternatingImuSamples) {
  TelemetrySnapshot snapshot;
  snapshot.telemetry.state = rp::RobotState::kBalancing;
  snapshot.primary_sample = rp::ImuSample{1, rp::ImuSensor::kMpu9250, {}, {}};
  snapshot.secondary_sample = rp::ImuSample{1, rp::ImuSensor::kMpu6050, {}, {}};
  telemetry.publish(snapshot);

  for (std::uint32_t t = 0; t <= 100000; t += 2000) {  // 100 ms, polled every 2 ms
    service.poll(t);
  }

  int telemetry_frames = 0;
  int mpu9250 = 0;
  int mpu6050 = 0;
  for (const auto& message : decodeAll(stream.tx)) {
    telemetry_frames += std::holds_alternative<rp::Telemetry>(message) ? 1 : 0;
    if (const auto* sample = std::get_if<rp::ImuSample>(&message)) {
      (sample->sensor == rp::ImuSensor::kMpu9250 ? mpu9250 : mpu6050)++;
    }
  }
  EXPECT_EQ(telemetry_frames, 6);  // t = 0, 20, 40, 60, 80, 100 ms
  EXPECT_EQ(mpu9250, 3);
  EXPECT_EQ(mpu6050, 3);
}

TEST_F(CommsTest, ForwardsLogLines) {
  robot::app::ProtocolLogSink sink(logs);
  sink.write(robot::logging::Level::kWarn, "IMU disagreement");
  service.poll(0);
  bool found = false;
  for (const auto& message : decodeAll(stream.tx)) {
    if (const auto* log = std::get_if<rp::LogMessage>(&message)) {
      found = log->level == rp::LogLevel::kWarn && log->view() == "IMU disagreement";
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(CommsTest, IgnoresGarbageAndKeepsWorking) {
  stream.receive({'e', 't', 's', ' ', 'J', 'u', 'n', 0x00});
  stream.receive(frame(rp::CmdVel{0.05F, 0.0F}));
  service.poll(500);
  EXPECT_FLOAT_EQ(motion.read().linear, 0.05F);
  EXPECT_EQ(link.rxStats().frames_ok, 1U);
}

TEST_F(CommsTest, OutgoingFramesCarryIncreasingSequenceNumbers) {
  link.send(rp::Heartbeat{});
  link.send(rp::Heartbeat{});
  rp::FrameParser parser;
  std::vector<std::uint8_t> sequences;
  for (const auto byte : stream.tx) {
    if (auto result = parser.push(byte); result && result->ok()) {
      sequences.push_back(result->sequence);
    }
  }
  ASSERT_EQ(sequences.size(), 2U);
  EXPECT_EQ(static_cast<std::uint8_t>(sequences[1] - sequences[0]), 1);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
