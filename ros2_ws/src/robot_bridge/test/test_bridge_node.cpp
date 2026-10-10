// Integration tests: the real BridgeNode on one side of a pseudo-terminal, a fake ESP32 on the
// other, ROS 2 publishers/subscribers/clients on the test side.
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "fake_esp32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_bridge/bridge_node.hpp"
#include "robot_interfaces/msg/balance_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace {

namespace rp = robot::protocol;
using namespace std::chrono_literals;
using robot_bridge::testing::FakeEsp32;
using std_srvs::srv::SetBool;

class BridgeNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override { start({}); }

  void start(const std::vector<rclcpp::Parameter>& extra) {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters{
        {"port", esp_.path()},  {"wheel_radius", 0.0325},
        {"track_width", 0.15},  {"counts_per_wheel_rev", 4200.0},
        {"command_rate", 50.0}, {"publish_tf", false},
    };
    parameters.insert(parameters.end(), extra.begin(), extra.end());
    options.parameter_overrides(parameters);
    bridge_ = std::make_shared<robot_bridge::BridgeNode>(options);
    tester_ = std::make_shared<rclcpp::Node>("bridge_tester");
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(bridge_);
    executor_->add_node(tester_);
    // Wait until the bridge has opened the port: its first heartbeat arrives at the fake ESP32.
    ASSERT_TRUE(waitForFrame(
        [](const rp::DecodeResult& f) { return std::holds_alternative<rp::CmdVel>(f.message); }));
  }

  void TearDown() override {
    executor_.reset();
    tester_.reset();
    bridge_.reset();
  }

  /// Spins until `done()` or the timeout.
  bool spinUntil(const std::function<bool()>& done, std::chrono::milliseconds timeout = 2s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(5ms);
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  /// Spins until the fake ESP32 receives a frame matching `predicate`.
  std::optional<rp::DecodeResult> waitForFrame(
      const std::function<bool(const rp::DecodeResult&)>& predicate,
      std::chrono::milliseconds timeout = 2s) {
    std::optional<rp::DecodeResult> found;
    spinUntil(
        [&] {
          for (const auto& frame : esp_.receive()) {
            if (!found && predicate(frame)) {
              found = frame;
            }
          }
          return found.has_value();
        },
        timeout);
    return found;
  }

  FakeEsp32 esp_;
  std::shared_ptr<robot_bridge::BridgeNode> bridge_;
  std::shared_ptr<rclcpp::Node> tester_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(BridgeNodeTest, PublishesStateAndOdometryFromTelemetry) {
  std::optional<robot_interfaces::msg::BalanceState> state;
  std::optional<nav_msgs::msg::Odometry> odom;
  auto state_sub = tester_->create_subscription<robot_interfaces::msg::BalanceState>(
      "balance/state", 10, [&](const robot_interfaces::msg::BalanceState& msg) { state = msg; });
  auto odom_sub = tester_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [&](const nav_msgs::msg::Odometry& msg) { odom = msg; });

  rp::Telemetry telemetry;
  telemetry.state = rp::RobotState::kBalancing;
  telemetry.pitch = 0.02F;
  telemetry.left_wheel_velocity = 2.0F;
  telemetry.right_wheel_velocity = 2.0F;
  esp_.send(telemetry);

  ASSERT_TRUE(spinUntil([&] { return state && odom; }));
  EXPECT_EQ(state->state, robot_interfaces::msg::BalanceState::STATE_BALANCING);
  EXPECT_FLOAT_EQ(state->pitch, 0.02F);
  EXPECT_NEAR(odom->twist.twist.linear.x, 2.0 * 0.0325, 1e-5);
}

TEST_F(BridgeNodeTest, PublishesBodyPitchAndWheelsAsJointStates) {
  std::optional<sensor_msgs::msg::JointState> joints;
  auto sub = tester_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, [&](const sensor_msgs::msg::JointState& msg) { joints = msg; });
  rp::Telemetry telemetry;
  telemetry.pitch = 0.1F;
  telemetry.left_ticks = 4200;  // one wheel revolution
  esp_.send(telemetry);

  ASSERT_TRUE(spinUntil([&] { return joints.has_value(); }));
  ASSERT_EQ(joints->name.size(), 3U);
  EXPECT_EQ(joints->name[0], "body_pitch_joint");
  EXPECT_NEAR(joints->position[0], 0.1, 1e-6);
  EXPECT_NEAR(joints->position[1], 2.0 * M_PI, 1e-6);
}

TEST_F(BridgeNodeTest, ForwardsCmdVel) {
  auto pub = tester_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.15;
  twist.angular.z = -0.5;
  const auto frame = waitForFrame([&](const rp::DecodeResult& f) {
    pub->publish(twist);
    const auto* cmd = std::get_if<rp::CmdVel>(&f.message);
    return cmd != nullptr && cmd->linear_x > 0.1F;
  });
  ASSERT_TRUE(frame.has_value());
  EXPECT_FLOAT_EQ(std::get<rp::CmdVel>(frame->message).angular_z, -0.5F);
}

TEST_F(BridgeNodeTest, StopsWhenCmdVelGoesStale) {
  TearDown();
  start({{"cmd_vel_timeout", 0.2}});
  auto pub = tester_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.15;
  pub->publish(twist);
  ASSERT_TRUE(waitForFrame([](const rp::DecodeResult& f) {
    const auto* cmd = std::get_if<rp::CmdVel>(&f.message);
    return cmd != nullptr && cmd->linear_x > 0.1F;
  }));
  // No further cmd_vel: after the timeout the bridge keeps sending, but zero.
  std::this_thread::sleep_for(300ms);
  esp_.receive();
  const auto frame = waitForFrame(
      [](const rp::DecodeResult& f) { return std::holds_alternative<rp::CmdVel>(f.message); });
  ASSERT_TRUE(frame.has_value());
  EXPECT_FLOAT_EQ(std::get<rp::CmdVel>(frame->message).linear_x, 0.0F);
}

TEST_F(BridgeNodeTest, ArmServiceAnswersWithTheEsp32Acknowledgement) {
  auto client = tester_->create_client<SetBool>("robot_bridge/arm");
  ASSERT_TRUE(spinUntil([&] { return client->service_is_ready(); }));
  auto request = std::make_shared<SetBool::Request>();
  request->data = true;
  auto future = client->async_send_request(request);

  const auto frame = waitForFrame(
      [](const rp::DecodeResult& f) { return std::holds_alternative<rp::SetArmed>(f.message); });
  ASSERT_TRUE(frame.has_value());
  EXPECT_TRUE(std::get<rp::SetArmed>(frame->message).armed);
  esp_.send(rp::Ack{rp::MessageType::kSetArmed, frame->sequence, rp::AckStatus::kOk});

  ASSERT_TRUE(spinUntil([&] { return future.wait_for(0s) == std::future_status::ready; }));
  EXPECT_TRUE(future.get()->success);
}

TEST_F(BridgeNodeTest, ArmServiceReportsRejection) {
  auto client = tester_->create_client<SetBool>("robot_bridge/arm");
  ASSERT_TRUE(spinUntil([&] { return client->service_is_ready(); }));
  auto future = client->async_send_request(std::make_shared<SetBool::Request>());
  const auto frame = waitForFrame(
      [](const rp::DecodeResult& f) { return std::holds_alternative<rp::SetArmed>(f.message); });
  ASSERT_TRUE(frame.has_value());
  esp_.send(rp::Ack{rp::MessageType::kSetArmed, frame->sequence, rp::AckStatus::kRejected});
  ASSERT_TRUE(spinUntil([&] { return future.wait_for(0s) == std::future_status::ready; }));
  const auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("rejected"), std::string::npos);
}

TEST_F(BridgeNodeTest, ArmServiceTimesOutWithoutAcknowledgement) {
  TearDown();
  start({{"ack_timeout", 0.2}});
  auto client = tester_->create_client<SetBool>("robot_bridge/arm");
  ASSERT_TRUE(spinUntil([&] { return client->service_is_ready(); }));
  auto future = client->async_send_request(std::make_shared<SetBool::Request>());
  ASSERT_TRUE(spinUntil([&] { return future.wait_for(0s) == std::future_status::ready; }));
  const auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("timeout"), std::string::npos);
}

TEST_F(BridgeNodeTest, PushesGainsOnParameterChange) {
  const auto result = bridge_->set_parameter(rclcpp::Parameter("gains.velocity.kp", 0.45));
  ASSERT_TRUE(result.successful);
  const auto frame = waitForFrame([](const rp::DecodeResult& f) {
    const auto* gains = std::get_if<rp::SetGains>(&f.message);
    return gains != nullptr && gains->loop == rp::GainLoop::kVelocity;
  });
  ASSERT_TRUE(frame.has_value());
  EXPECT_FLOAT_EQ(std::get<rp::SetGains>(frame->message).kp, 0.45F);
}

TEST_F(BridgeNodeTest, RejectsInvalidGains) {
  EXPECT_FALSE(bridge_->set_parameter(rclcpp::Parameter("gains.pitch.kd", -1.0)).successful);
}

TEST_F(BridgeNodeTest, PushesAllGainsWhenTelemetryFirstArrives) {
  esp_.send(rp::Telemetry{});
  int loops_seen = 0;
  bool seen[3] = {false, false, false};
  waitForFrame([&](const rp::DecodeResult& f) {
    if (const auto* gains = std::get_if<rp::SetGains>(&f.message)) {
      seen[static_cast<int>(gains->loop)] = true;
    }
    loops_seen = static_cast<int>(seen[0]) + static_cast<int>(seen[1]) + static_cast<int>(seen[2]);
    return loops_seen == 3;
  });
  EXPECT_EQ(loops_seen, 3);
}

}  // namespace
