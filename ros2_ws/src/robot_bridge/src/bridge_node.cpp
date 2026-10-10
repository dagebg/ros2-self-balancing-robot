#include "robot_bridge/bridge_node.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <system_error>
#include <utility>
#include <variant>

#include "balance_core/balance_controller.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robot_bridge {

namespace rp = robot::protocol;
using namespace std::chrono_literals;

namespace {

constexpr auto kPollPeriod = 2ms;
constexpr double kTelemetryStaleSeconds = 0.5;

struct LoopName {
  rp::GainLoop loop;
  const char* name;
};
constexpr std::array<LoopName, 3> kLoops{{{rp::GainLoop::kPitch, "pitch"},
                                          {rp::GainLoop::kVelocity, "velocity"},
                                          {rp::GainLoop::kYawRate, "yaw_rate"}}};

std::string gainParameter(const char* loop, const char* term) {
  return std::string("gains.") + loop + "." + term;
}

robot::balance::PidGains defaultGains(rp::GainLoop loop) {
  const robot::balance::BalanceConfig defaults;
  switch (loop) {
    case rp::GainLoop::kPitch:
      return defaults.pitch_gains;
    case rp::GainLoop::kVelocity:
      return defaults.velocity_gains;
    case rp::GainLoop::kYawRate:
      return defaults.yaw_rate_gains;
  }
  return {};
}

std::chrono::nanoseconds seconds(double value) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(value));
}

const char* stateName(std::uint8_t state) {
  switch (state) {
    case robot_interfaces::msg::BalanceState::STATE_DISARMED:
      return "disarmed";
    case robot_interfaces::msg::BalanceState::STATE_ARMED:
      return "armed";
    case robot_interfaces::msg::BalanceState::STATE_BALANCING:
      return "balancing";
    case robot_interfaces::msg::BalanceState::STATE_FAULT:
      return "fault";
    default:
      return "unknown";
  }
}

}  // namespace

BridgeNode::BridgeNode(const rclcpp::NodeOptions& options)
    : Node("robot_bridge", options), diagnostics_(this) {
  declareParameters();

  const double wheel_radius = get_parameter("wheel_radius").as_double();
  const double track_width = get_parameter("track_width").as_double();
  const double counts = get_parameter("counts_per_wheel_rev").as_double();
  if (wheel_radius <= 0.0 || track_width <= 0.0 || counts <= 0.0) {
    throw std::invalid_argument("wheel_radius, track_width and counts_per_wheel_rev must be > 0");
  }
  odometry_.emplace(robot::balance::WheelGeometry{static_cast<float>(wheel_radius),
                                                  static_cast<float>(track_width)},
                    counts);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  state_pub_ = create_publisher<robot_interfaces::msg::BalanceState>("balance/state", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", sensor_qos);
  imu_secondary_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/secondary", sensor_qos);
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist& msg) {
        last_cmd_vel_ = msg;
        last_cmd_vel_time_ = now();
      });

  arm_service_ = create_service<SetBool>(
      "~/arm", [this](const std::shared_ptr<rclcpp::Service<SetBool>>& service,
                      const std::shared_ptr<rmw_request_id_t>& request_id,
                      const std::shared_ptr<SetBool::Request>& request) {
        onArmRequest(service, request_id, request);
      });

  validate_handle_ =
      add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
        return validateParameters(parameters);
      });
  post_set_handle_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) { onParametersSet(parameters); });

  diagnostics_.setHardwareID("esp32");
  diagnostics_.add("link", this, &BridgeNode::produceLinkDiagnostics);
  diagnostics_.add("controller", this, &BridgeNode::produceControllerDiagnostics);

  const double command_rate = get_parameter("command_rate").as_double();
  poll_timer_ = create_wall_timer(kPollPeriod, [this] { poll(); });
  command_timer_ = create_wall_timer(seconds(1.0 / command_rate), [this] { sendCommand(); });
  last_connect_attempt_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

BridgeNode::~BridgeNode() { port_.close(); }

void BridgeNode::declareParameters() {
  port_path_ = declare_parameter<std::string>("port", "/dev/serial0");
  baud_ = static_cast<int>(declare_parameter<int>("baud", 460800));
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
  pitch_joint_ = declare_parameter<std::string>("pitch_joint", "body_pitch_joint");
  left_joint_ = declare_parameter<std::string>("left_wheel_joint", "left_wheel_joint");
  right_joint_ = declare_parameter<std::string>("right_wheel_joint", "right_wheel_joint");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  cmd_vel_timeout_ = seconds(declare_parameter<double>("cmd_vel_timeout", 0.5));
  reconnect_period_ = seconds(declare_parameter<double>("reconnect_period", 1.0));
  ack_timeout_ = seconds(declare_parameter<double>("ack_timeout", 1.0));
  declare_parameter<double>("command_rate", 20.0);
  // Robot geometry: provided by the launch file from config/robot_params.yaml.
  declare_parameter<double>("wheel_radius", 0.0);
  declare_parameter<double>("track_width", 0.0);
  declare_parameter<double>("counts_per_wheel_rev", 0.0);

  for (const auto& [loop, name] : kLoops) {
    const robot::balance::PidGains gains = defaultGains(loop);
    declare_parameter<double>(gainParameter(name, "kp"), gains.kp);
    declare_parameter<double>(gainParameter(name, "ki"), gains.ki);
    declare_parameter<double>(gainParameter(name, "kd"), gains.kd);
  }
}

// --- Link handling -------------------------------------------------------------------------------

void BridgeNode::tryConnect() {
  const rclcpp::Time now_time = now();
  if (now_time - last_connect_attempt_ < rclcpp::Duration(reconnect_period_)) {
    return;
  }
  last_connect_attempt_ = now_time;
  try {
    port_.open(port_path_, baud_);
    link_.reset();
    gains_sent_since_connect_ = false;
    odometry_->reset();
    RCLCPP_INFO(get_logger(), "opened %s at %d Bd", port_path_.c_str(), baud_);
  } catch (const std::exception& error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "cannot open %s: %s",
                         port_path_.c_str(), error.what());
  }
}

void BridgeNode::disconnect(const std::string& reason) {
  RCLCPP_ERROR(get_logger(), "serial link lost: %s", reason.c_str());
  port_.close();
  last_telemetry_time_.reset();
}

bool BridgeNode::send(const rp::Message& message, std::uint8_t* sequence) {
  if (!port_.isOpen()) {
    return false;
  }
  try {
    const auto used = link_.send(message);
    if (used && sequence != nullptr) {
      *sequence = *used;
    }
    return used.has_value();
  } catch (const std::system_error& error) {
    disconnect(error.what());
    return false;
  }
}

void BridgeNode::poll() {
  if (!port_.isOpen()) {
    tryConnect();
    return;
  }
  try {
    for (const auto& frame : link_.receive()) {
      onFrame(frame);
    }
  } catch (const std::system_error& error) {
    disconnect(error.what());
  }
  expirePendingArms();
}

void BridgeNode::sendCommand() {
  rp::CmdVel command;
  const bool fresh =
      last_cmd_vel_time_ && now() - *last_cmd_vel_time_ < rclcpp::Duration(cmd_vel_timeout_);
  if (fresh) {
    command.linear_x = static_cast<float>(last_cmd_vel_.linear.x);
    command.angular_z = static_cast<float>(last_cmd_vel_.angular.z);
  }
  send(command);  // also the heartbeat that keeps the ESP32's link supervision happy
}

void BridgeNode::sendGains(rp::GainLoop loop) {
  for (const auto& [id, name] : kLoops) {
    if (id != loop) {
      continue;
    }
    rp::SetGains gains;
    gains.loop = loop;
    gains.kp = static_cast<float>(get_parameter(gainParameter(name, "kp")).as_double());
    gains.ki = static_cast<float>(get_parameter(gainParameter(name, "ki")).as_double());
    gains.kd = static_cast<float>(get_parameter(gainParameter(name, "kd")).as_double());
    send(gains);
  }
}

void BridgeNode::sendAllGains() {
  for (const auto& [loop, name] : kLoops) {
    sendGains(loop);
  }
  gains_sent_since_connect_ = true;
}

// --- Incoming frames -----------------------------------------------------------------------------

void BridgeNode::onFrame(const rp::DecodeResult& frame) {
  std::visit(
      [this](const auto& message) {
        using T = std::decay_t<decltype(message)>;
        if constexpr (std::is_same_v<T, rp::Telemetry>) {
          onTelemetry(message);
        } else if constexpr (std::is_same_v<T, rp::ImuSample>) {
          onImu(message);
        } else if constexpr (std::is_same_v<T, rp::LogMessage>) {
          onLog(message);
        } else if constexpr (std::is_same_v<T, rp::Ack>) {
          onAck(message);
        }
        // Pi -> ESP32 message types arriving here would be a wiring loopback; ignore them.
      },
      frame.message);
}

void BridgeNode::onTelemetry(const rp::Telemetry& telemetry) {
  const rclcpp::Time stamp = now();
  if (!gains_sent_since_connect_) {
    sendAllGains();  // the ESP32 may have rebooted: make it run the gains ROS shows
  }
  if (last_telemetry_ && last_telemetry_->state != telemetry.state) {
    RCLCPP_INFO(get_logger(), "controller state: %s",
                stateName(static_cast<std::uint8_t>(telemetry.state)));
  }
  last_telemetry_ = telemetry;
  last_telemetry_time_ = stamp;

  state_pub_->publish(toBalanceState(telemetry, stamp, base_frame_));

  odometry_->update(telemetry);
  odom_pub_->publish(toOdometry(*odometry_, stamp, odom_frame_, base_frame_));
  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odometry_->pose().x;
    transform.transform.translation.y = odometry_->pose().y;
    transform.transform.rotation = quaternionFromRpy(0.0, 0.0, odometry_->pose().theta);
    tf_broadcaster_->sendTransform(transform);
  }

  sensor_msgs::msg::JointState joints;
  joints.header.stamp = stamp;
  // The body's pitch is a joint of the robot model (base_footprint -> base_link), so the model
  // in RViz leans exactly as measured; wheel angles are relative to the pitching body.
  joints.name = {pitch_joint_, left_joint_, right_joint_};
  joints.position = {telemetry.pitch, odometry_->leftWheelAngle(), odometry_->rightWheelAngle()};
  joints.velocity = {telemetry.pitch_rate, telemetry.left_wheel_velocity,
                     telemetry.right_wheel_velocity};
  joint_pub_->publish(joints);

  diagnostics_.force_update();
}

void BridgeNode::onImu(const rp::ImuSample& sample) {
  const rclcpp::Time stamp = now();
  if (sample.sensor == rp::ImuSensor::kMpu9250) {
    const bool has_attitude = last_telemetry_.has_value();
    const double roll = has_attitude ? last_telemetry_->roll : 0.0;
    const double pitch = has_attitude ? last_telemetry_->pitch : 0.0;
    imu_pub_->publish(
        toImu(sample, has_attitude, roll, pitch, odometry_->gyroYaw(), stamp, imu_frame_));
  } else {
    imu_secondary_pub_->publish(toImu(sample, false, 0.0, 0.0, 0.0, stamp, imu_frame_));
  }
}

void BridgeNode::onLog(const rp::LogMessage& log) {
  const std::string text(log.view());
  switch (log.level) {
    case rp::LogLevel::kDebug:
      RCLCPP_DEBUG(get_logger(), "[esp32] %s", text.c_str());
      break;
    case rp::LogLevel::kInfo:
      RCLCPP_INFO(get_logger(), "[esp32] %s", text.c_str());
      break;
    case rp::LogLevel::kWarn:
      RCLCPP_WARN(get_logger(), "[esp32] %s", text.c_str());
      break;
    case rp::LogLevel::kError:
      RCLCPP_ERROR(get_logger(), "[esp32] %s", text.c_str());
      break;
  }
}

void BridgeNode::onAck(const rp::Ack& ack) {
  if (ack.acked_type == rp::MessageType::kSetGains && ack.status != rp::AckStatus::kOk) {
    RCLCPP_WARN(get_logger(), "ESP32 rejected a gain update (sequence %u)", ack.acked_sequence);
    return;
  }
  if (ack.acked_type != rp::MessageType::kSetArmed) {
    return;
  }
  const auto pending = pending_arms_.find(ack.acked_sequence);
  if (pending == pending_arms_.end()) {
    return;
  }
  SetBool::Response response;
  response.success = ack.status == rp::AckStatus::kOk;
  if (response.success) {
    response.message = pending->second.armed ? "armed" : "disarmed";
  } else if (ack.status == rp::AckStatus::kRejected) {
    response.message = "rejected: the robot is in a fault state or the gyro is not calibrated";
  } else {
    response.message = "invalid request";
  }
  arm_service_->send_response(*pending->second.request_id, response);
  pending_arms_.erase(pending);
}

void BridgeNode::onArmRequest(const std::shared_ptr<rclcpp::Service<SetBool>>& service,
                              const std::shared_ptr<rmw_request_id_t>& request_id,
                              const std::shared_ptr<SetBool::Request>& request) {
  std::uint8_t sequence = 0;
  if (!send(rp::SetArmed{request->data}, &sequence)) {
    SetBool::Response response;
    response.success = false;
    response.message = "serial link not connected";
    service->send_response(*request_id, response);
    return;
  }
  pending_arms_[sequence] =
      PendingArm{request_id, now() + rclcpp::Duration(ack_timeout_), request->data};
}

void BridgeNode::expirePendingArms() {
  const rclcpp::Time now_time = now();
  for (auto it = pending_arms_.begin(); it != pending_arms_.end();) {
    if (now_time < it->second.deadline) {
      ++it;
      continue;
    }
    SetBool::Response response;
    response.success = false;
    response.message = "timeout: no acknowledgement from the ESP32";
    arm_service_->send_response(*it->second.request_id, response);
    it = pending_arms_.erase(it);
  }
}

// --- Parameters ----------------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult BridgeNode::validateParameters(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto& parameter : parameters) {
    if (parameter.get_name().rfind("gains.", 0) != 0) {
      continue;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.successful = false;
      result.reason = parameter.get_name() + " must be a double";
      break;
    }
    const double value = parameter.as_double();
    if (!std::isfinite(value) || value < 0.0) {
      result.successful = false;
      result.reason = parameter.get_name() + " must be finite and non-negative";
      break;
    }
  }
  return result;
}

void BridgeNode::onParametersSet(const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& [loop, name] : kLoops) {
    const std::string prefix = std::string("gains.") + name + ".";
    for (const auto& parameter : parameters) {
      if (parameter.get_name().rfind(prefix, 0) == 0) {
        sendGains(loop);
        break;
      }
    }
  }
}

// --- Diagnostics ---------------------------------------------------------------------------------

void BridgeNode::produceLinkDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto& stats = link_.stats();
  if (!port_.isOpen()) {
    status.summary(DiagnosticStatus::ERROR, "serial port not open");
  } else if (!last_telemetry_time_ ||
             (now() - *last_telemetry_time_).seconds() > kTelemetryStaleSeconds) {
    status.summary(DiagnosticStatus::ERROR, "no telemetry from the ESP32");
  } else if (stats.crc_errors + stats.cobs_errors + stats.other_errors > 0) {
    status.summary(DiagnosticStatus::WARN, "receiving, with corrupted frames");
  } else {
    status.summary(DiagnosticStatus::OK, "receiving");
  }
  status.add("port", port_path_);
  status.add("frames ok", stats.frames_ok);
  status.add("crc errors", stats.crc_errors);
  status.add("cobs errors", stats.cobs_errors);
  status.add("other errors", stats.other_errors);
  status.add("overflows", stats.overflows);
  status.add("frames lost", link_.lost());
}

void BridgeNode::produceControllerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!last_telemetry_) {
    status.summary(DiagnosticStatus::STALE, "no telemetry yet");
    return;
  }
  const auto state = static_cast<std::uint8_t>(last_telemetry_->state);
  if (state == robot_interfaces::msg::BalanceState::STATE_FAULT) {
    status.summary(DiagnosticStatus::ERROR, "fault");
  } else if (last_telemetry_->faults != 0) {
    status.summary(DiagnosticStatus::WARN, stateName(state));
  } else {
    status.summary(DiagnosticStatus::OK, stateName(state));
  }
  status.add("faults", last_telemetry_->faults);
  status.add("pitch [deg]", last_telemetry_->pitch * 180.0 / M_PI);
  status.add("max loop time [us]", last_telemetry_->max_loop_time_us);
}

}  // namespace robot_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(robot_bridge::BridgeNode)
