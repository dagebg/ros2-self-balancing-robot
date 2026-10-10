#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <stdexcept>
#include <system_error>
#include <thread>

#include "fake_esp32.hpp"
#include "robot_bridge/framed_link.hpp"
#include "robot_bridge/serial_port.hpp"

namespace {

using robot_bridge::FramedLink;
using robot_bridge::SerialPort;
using robot_bridge::testing::FakeEsp32;
namespace rp = robot::protocol;

TEST(SerialPortTest, OpeningAMissingDeviceThrows) {
  SerialPort port;
  EXPECT_THROW(port.open("/dev/does-not-exist", 460800), std::system_error);
  EXPECT_FALSE(port.isOpen());
}

TEST(SerialPortTest, RejectsUnsupportedBaudRates) {
  FakeEsp32 esp;
  SerialPort port;
  EXPECT_THROW(port.open(esp.path(), 12345), std::invalid_argument);
}

TEST(SerialPortTest, TransfersBytesInBothDirections) {
  FakeEsp32 esp;
  SerialPort port;
  port.open(esp.path(), 460800);
  ASSERT_TRUE(port.isOpen());

  const std::array<std::uint8_t, 4> outgoing{0x01, 0x00, 0xFF, 0x7E};
  EXPECT_EQ(port.write(outgoing.data(), outgoing.size()), outgoing.size());
  std::array<std::uint8_t, 8> received{};
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(read(esp.masterFd(), received.data(), received.size()), 4);
  EXPECT_EQ(received[2], 0xFF);

  const std::array<std::uint8_t, 3> incoming{0x10, 0x20, 0x30};
  ASSERT_EQ(write(esp.masterFd(), incoming.data(), incoming.size()), 3);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::array<std::uint8_t, 8> buffer{};
  EXPECT_EQ(port.read(buffer.data(), buffer.size()), 3U);
  EXPECT_EQ(buffer[1], 0x20);
}

TEST(SerialPortTest, ReadWithNothingPendingReturnsZeroImmediately) {
  FakeEsp32 esp;
  SerialPort port;
  port.open(esp.path(), 460800);
  std::array<std::uint8_t, 8> buffer{};
  EXPECT_EQ(port.read(buffer.data(), buffer.size()), 0U);
}

TEST(FramedLinkTest, RoundTripsMessagesWithSequenceNumbers) {
  FakeEsp32 esp;
  SerialPort port;
  port.open(esp.path(), 460800);
  FramedLink link(port);

  EXPECT_EQ(link.send(rp::CmdVel{0.1F, 0.2F}), 0);
  EXPECT_EQ(link.send(rp::Heartbeat{}), 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const auto at_esp = esp.receive();
  ASSERT_EQ(at_esp.size(), 2U);
  EXPECT_TRUE(std::holds_alternative<rp::CmdVel>(at_esp[0].message));

  esp.send(rp::Ack{rp::MessageType::kSetArmed, 5, rp::AckStatus::kOk}, 9);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const auto at_pi = link.receive();
  ASSERT_EQ(at_pi.size(), 1U);
  EXPECT_EQ(at_pi[0].sequence, 9);
  EXPECT_EQ(link.stats().frames_ok, 1U);
}

}  // namespace
