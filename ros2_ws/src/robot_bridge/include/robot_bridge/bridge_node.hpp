#pragma once

/// @file
/// ROS 2 node connecting the ESP32 to the ROS graph.

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_bridge/conversions.hpp"
#include "robot_bridge/framed_link.hpp"
#include "robot_bridge/serial_port.hpp"
#include "robot_interfaces/msg/balance_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace robot_bridge {

/// Bridges the ESP32's UART protocol and ROS 2.
///
/// - Subscribes to `cmd_vel` and forwards it at a fixed rate; when commands stop (or are older than
///   `cmd_vel_timeout`), it sends zero velocity, which doubles as the link heartbeat.
/// - Publishes `balance/state`, `odom` (+ TF odom -> base_link), `joint_states`, `imu/data`
///   (MPU-9250 with the ESP32's orientation estimate), `imu/secondary` (MPU-6050), and
///   `diagnostics`. Firmware log lines go to the ROS log (`/rosout`).
/// - Service `~/arm` (std_srvs/SetBool) arms or disarms; it answers once the ESP32 acknowledges.
/// - Parameters `gains.<loop>.{kp,ki,kd}` are pushed to the ESP32 on change and after every
///   (re)connection, so the firmware always runs the gains the ROS side shows.
class BridgeNode : public rclcpp::Node {
 public:
  explicit BridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~BridgeNode() override;
  BridgeNode(const BridgeNode&) = delete;
  BridgeNode& operator=(const BridgeNode&) = delete;
  BridgeNode(BridgeNode&&) = delete;
  BridgeNode& operator=(BridgeNode&&) = delete;

 private:
  using SetBool = std_srvs::srv::SetBool;

  struct PendingArm {
    std::shared_ptr<rmw_request_id_t> request_id;
    rclcpp::Time deadline;
    bool armed;
  };

  void declareParameters();
  void poll();
  void sendCommand();
  void tryConnect();
  void disconnect(const std::string& reason);
  bool send(const robot::protocol::Message& message, std::uint8_t* sequence = nullptr);
  void sendAllGains();
  void sendGains(robot::protocol::GainLoop loop);

  void onFrame(const robot::protocol::DecodeResult& frame);
  void onTelemetry(const robot::protocol::Telemetry& telemetry);
  void onImu(const robot::protocol::ImuSample& sample);
  void onLog(const robot::protocol::LogMessage& log);
  void onAck(const robot::protocol::Ack& ack);
  void onArmRequest(const std::shared_ptr<rclcpp::Service<SetBool>>& service,
                    const std::shared_ptr<rmw_request_id_t>& request_id,
                    const std::shared_ptr<SetBool::Request>& request);
  void expirePendingArms();
  rcl_interfaces::msg::SetParametersResult validateParameters(
      const std::vector<rclcpp::Parameter>& parameters);
  void onParametersSet(const std::vector<rclcpp::Parameter>& parameters);
  void produceLinkDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);
  void produceControllerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

  // Configuration
  std::string port_path_;
  int baud_ = 0;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string left_joint_;
  std::string right_joint_;
  bool publish_tf_ = true;
  std::chrono::nanoseconds cmd_vel_timeout_{};
  std::chrono::nanoseconds reconnect_period_{};
  std::chrono::nanoseconds ack_timeout_{};

  // Link
  SerialPort port_;
  FramedLink link_{port_};
  rclcpp::Time last_connect_attempt_;
  std::optional<rclcpp::Time> last_telemetry_time_;
  bool gains_sent_since_connect_ = false;

  // State
  std::optional<OdometryTracker> odometry_;
  std::optional<robot::protocol::Telemetry> last_telemetry_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  std::optional<rclcpp::Time> last_cmd_vel_time_;
  std::map<std::uint8_t, PendingArm> pending_arms_;
  std::shared_ptr<rclcpp::Service<SetBool>> arm_service_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<robot_interfaces::msg::BalanceState>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_secondary_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  diagnostic_updater::Updater diagnostics_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;
};

}  // namespace robot_bridge
