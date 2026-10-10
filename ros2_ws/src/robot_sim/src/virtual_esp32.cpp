// Virtual ESP32: the firmware's own ControlLoop and CommsService, compiled for the host and driven
// by Gazebo instead of real sensors and motors.
//
//   Gazebo IMU (200 Hz) + wheel joint states --> SensorFrame --> ControlLoop --> motor commands
//   motor commands --> DC-motor model --> wheel torques --> Gazebo
//   CommsService <--> pseudo-terminal <--> the unchanged robot_bridge (as on the real robot)
//
// Everything above the hardware abstraction is the code that runs on the robot.

#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "app/control_loop.hpp"
#include "app/link.hpp"
#include "app/mailbox.hpp"
#include "hal/hal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace robot_sim {

namespace app = robot::app;

/// The master side of a pseudo-terminal as the firmware's UART. The slave side is exposed under a
/// fixed path (a symlink) for robot_bridge to open like a serial port.
class PtyStream final : public robot::hal::ByteStream {
 public:
  explicit PtyStream(std::string link_path) : link_path_(std::move(link_path)) {
    std::array<char, 256> name{};
    if (openpty(&master_, &slave_, name.data(), nullptr, nullptr) != 0) {
      throw std::runtime_error("openpty failed");
    }
    termios raw{};
    tcgetattr(slave_, &raw);
    cfmakeraw(&raw);
    tcsetattr(slave_, TCSANOW, &raw);
    fcntl(master_, F_SETFL, fcntl(master_, F_GETFL) | O_NONBLOCK);
    unlink(link_path_.c_str());
    if (symlink(name.data(), link_path_.c_str()) != 0) {
      throw std::runtime_error("cannot create " + link_path_);
    }
  }

  ~PtyStream() override {
    unlink(link_path_.c_str());
    close(master_);
    close(slave_);
  }

  PtyStream(const PtyStream&) = delete;
  PtyStream& operator=(const PtyStream&) = delete;
  PtyStream(PtyStream&&) = delete;
  PtyStream& operator=(PtyStream&&) = delete;

  std::size_t read(std::uint8_t* data, std::size_t capacity) override {
    const ssize_t count = ::read(master_, data, capacity);
    return count > 0 ? static_cast<std::size_t>(count) : 0;
  }

  std::size_t write(const std::uint8_t* data, std::size_t size) override {
    const ssize_t count = ::write(master_, data, size);
    return count > 0 ? static_cast<std::size_t>(count) : 0;
  }

 private:
  std::string link_path_;
  int master_ = -1;
  int slave_ = -1;
};

class VirtualEsp32 : public rclcpp::Node {
 public:
  VirtualEsp32()
      : Node("virtual_esp32"),
        stream_(declare_parameter<std::string>("uart_link", "/tmp/robot_sim_uart")),
        control_(makeConfig()),
        link_(stream_),
        comms_(
            link_, {motion_, requests_, acks_, telemetry_, logs_},
            static_cast<std::uint32_t>(1e6 / declare_parameter<double>("telemetry_rate", 50.0))) {
    // Members declared after control_ are initialized after makeConfig() ran, so read here.
    counts_per_rev_ = get_parameter("counts_per_wheel_rev").as_double();
    stall_torque_ = declare_parameter<double>("stall_torque", 0.15);
    no_load_speed_ = declare_parameter<double>("no_load_speed", 10.5);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    left_joint_ = declare_parameter<std::string>("left_wheel_joint", "left_wheel_joint");
    right_joint_ = declare_parameter<std::string>("right_wheel_joint", "right_wheel_joint");

    left_torque_ = create_publisher<std_msgs::msg::Float64>("sim/left_wheel_torque", 10);
    right_torque_ = create_publisher<std_msgs::msg::Float64>("sim/right_wheel_torque", 10);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "sim/joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState& msg) { onJointStates(msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "sim/imu", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu& msg) { onImu(msg); });

    link_.sendResync();
    RCLCPP_INFO(get_logger(), "virtual ESP32 UART at %s",
                get_parameter("uart_link").as_string().c_str());
  }

 private:
  app::ControlConfig makeConfig() {
    app::ControlConfig config;
    // Gazebo's IMU sits on base_link with body axes: identity mounting (the defaults).
    config.geometry = {static_cast<float>(declare_parameter<double>("wheel_radius", 0.0325)),
                       static_cast<float>(declare_parameter<double>("track_width", 0.15))};
    config.counts_per_wheel_rev =
        static_cast<float>(declare_parameter<double>("counts_per_wheel_rev", 4200.0));
    config.nominal_period = 0.005F;
    // The simulated gyro has no bias drift; a short window gets the robot armable quickly.
    config.bias.samples = static_cast<std::uint32_t>(declare_parameter<int>("bias_samples", 20));
    return config;
  }

  void onJointStates(const sensor_msgs::msg::JointState& msg) {
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      const double position = i < msg.position.size() ? msg.position[i] : 0.0;
      const double velocity = i < msg.velocity.size() ? msg.velocity[i] : 0.0;
      if (msg.name[i] == left_joint_) {
        left_angle_ = position;
        left_rate_ = velocity;
      } else if (msg.name[i] == right_joint_) {
        right_angle_ = position;
        right_rate_ = velocity;
      }
    }
  }

  [[nodiscard]] std::int32_t ticks(double angle) const {
    return static_cast<std::int32_t>(std::floor(angle * counts_per_rev_ / (2.0 * M_PI)));
  }

  /// DC-motor curve, as in balance_core's plant model: torque falls with speed (back-EMF).
  [[nodiscard]] double motorTorque(float command, double relative_rate) const {
    const double u = std::clamp(static_cast<double>(command), -1.0, 1.0);
    return stall_torque_ * (u - relative_rate / no_load_speed_);
  }

  /// One control tick per IMU sample (200 Hz in simulation time), like the firmware's task.
  void onImu(const sensor_msgs::msg::Imu& msg) {
    const auto now_us =
        static_cast<std::uint32_t>(rclcpp::Time(msg.header.stamp).nanoseconds() / 1000);
    robot::drivers::ImuReading reading;
    reading.accel = {static_cast<float>(msg.linear_acceleration.x),
                     static_cast<float>(msg.linear_acceleration.y),
                     static_cast<float>(msg.linear_acceleration.z)};
    reading.gyro = {static_cast<float>(msg.angular_velocity.x),
                    static_cast<float>(msg.angular_velocity.y),
                    static_cast<float>(msg.angular_velocity.z)};
    reading.temperature = 25.0F;

    app::SensorFrame frame;
    frame.time_us = now_us;
    frame.primary = reading;
    frame.left_ticks = ticks(left_angle_);
    frame.right_ticks = ticks(right_angle_);

    while (const auto request = requests_.pop()) {
      acks_.push(control_.handle(request->message, request->sequence));
    }
    // On the robot a hand holds it upright while it is armed. The simulated robot spawns upright,
    // so arming as soon as the gyro is calibrated plays that role.
    if (auto_arm_ && !auto_armed_ && control_.calibrated()) {
      auto_armed_ = control_.handle(robot::protocol::SetArmed{true}, 0).status ==
                    robot::protocol::AckStatus::kOk;
      if (auto_armed_) {
        RCLCPP_INFO(get_logger(), "armed automatically after calibration");
      }
    }
    const app::MotionTarget target = motion_.read();
    const bool link_alive = target.received_any && now_us - target.last_rx_us < kLinkTimeoutUs;
    const app::ControlOutput out =
        control_.step(frame, {target.linear, target.angular, link_alive});
    telemetry_.publish({out.telemetry, out.primary_sample, out.secondary_sample});
    comms_.poll(now_us);

    std_msgs::msg::Float64 left;
    std_msgs::msg::Float64 right;
    left.data = out.motors_enabled ? motorTorque(out.left, left_rate_) : 0.0;
    right.data = out.motors_enabled ? motorTorque(out.right, right_rate_) : 0.0;
    left_torque_->publish(left);
    right_torque_->publish(right);
  }

  static constexpr std::uint32_t kLinkTimeoutUs = 500'000;

  PtyStream stream_;
  app::ControlLoop control_;
  app::Mailbox<app::MotionTarget> motion_;
  app::RequestQueue requests_;
  app::AckQueue acks_;
  app::Mailbox<app::TelemetrySnapshot> telemetry_;
  app::LogQueue logs_;
  app::Link link_;
  app::CommsService comms_;

  double counts_per_rev_ = 4200.0;
  bool auto_arm_ = false;
  bool auto_armed_ = false;
  double stall_torque_ = 0.15;
  double no_load_speed_ = 10.5;
  std::string left_joint_;
  std::string right_joint_;
  double left_angle_ = 0.0;
  double right_angle_ = 0.0;
  double left_rate_ = 0.0;
  double right_rate_ = 0.0;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_torque_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_torque_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

}  // namespace robot_sim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sim::VirtualEsp32>());
  rclcpp::shutdown();
  return 0;
}
