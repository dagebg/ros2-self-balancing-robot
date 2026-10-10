#include "app/control_loop.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace robot::app {

namespace {

constexpr float kTwoPi = 2.0F * balance::kPi;

// The two firmware layers define the same states and fault bits independently; keep them aligned.
static_assert(static_cast<int>(balance::SafetyState::kDisarmed) ==
              static_cast<int>(protocol::RobotState::kDisarmed));
static_assert(static_cast<int>(balance::SafetyState::kArmed) ==
              static_cast<int>(protocol::RobotState::kArmed));
static_assert(static_cast<int>(balance::SafetyState::kBalancing) ==
              static_cast<int>(protocol::RobotState::kBalancing));
static_assert(static_cast<int>(balance::SafetyState::kFault) ==
              static_cast<int>(protocol::RobotState::kFault));
static_assert(balance::fault::kTiltLimit == protocol::fault::kTiltLimit);
static_assert(balance::fault::kImuFailure == protocol::fault::kImuFailure);
static_assert(static_cast<int>(balance::Loop::kPitch) ==
              static_cast<int>(protocol::GainLoop::kPitch));
static_assert(static_cast<int>(balance::Loop::kVelocity) ==
              static_cast<int>(protocol::GainLoop::kVelocity));
static_assert(static_cast<int>(balance::Loop::kYawRate) ==
              static_cast<int>(protocol::GainLoop::kYawRate));

balance::Vec3 toVec(const drivers::Vector3& v) noexcept { return {v.x, v.y, v.z}; }

std::array<float, 3> toArray(const balance::Vec3& v) noexcept { return {v.x, v.y, v.z}; }

/// Rotates a reading into the body frame. The bias is removed separately.
balance::ImuSampleBody toBody(const drivers::ImuReading& reading,
                              const drivers::AxisMap& mounting) noexcept {
  return {toVec(mounting.apply(reading.accel)), toVec(mounting.apply(reading.gyro))};
}

/// Calibrates while disarmed and returns the bias-corrected sample, or nothing until calibrated.
std::optional<balance::ImuSampleBody> calibrate(const std::optional<balance::ImuSampleBody>& raw,
                                                balance::GyroBiasEstimator& bias,
                                                bool learning) noexcept {
  if (!raw) {
    return std::nullopt;
  }
  if (learning) {
    bias.update(raw->gyro);
  }
  if (!bias.ready()) {
    return std::nullopt;
  }
  return balance::ImuSampleBody{raw->accel, bias.correct(raw->gyro)};
}

protocol::ImuSample toSample(const balance::ImuSampleBody& sample, protocol::ImuSensor sensor,
                             std::uint32_t time_us) noexcept {
  protocol::ImuSample out;
  out.timestamp_us = time_us;
  out.sensor = sensor;
  out.accel = toArray(sample.accel);
  out.gyro = toArray(sample.gyro);
  return out;
}

bool validGains(const protocol::SetGains& gains) noexcept {
  return std::isfinite(gains.kp) && std::isfinite(gains.ki) && std::isfinite(gains.kd) &&
         gains.kp >= 0.0F && gains.ki >= 0.0F && gains.kd >= 0.0F;
}

}  // namespace

ControlLoop::ControlLoop(const ControlConfig& config) noexcept
    : config_(config),
      primary_bias_(config.bias),
      secondary_bias_(config.bias),
      safety_(config.safety),
      controller_(config.balance) {}

bool ControlLoop::calibrated() const noexcept {
  return primary_bias_.ready() || secondary_bias_.ready();
}

float ControlLoop::wheelAngle(std::int32_t ticks, bool inverted) const noexcept {
  const float angle = static_cast<float>(ticks) * kTwoPi / config_.counts_per_wheel_rev;
  return inverted ? -angle : angle;
}

ControlOutput ControlLoop::step(const SensorFrame& frame, const MotionCommand& command) noexcept {
  ControlOutput out;

  // Time step from the frame timestamps (unsigned subtraction handles micros() wrap-around),
  // limited so a stall cannot inject a huge dt into the integrators.
  const bool first_frame = !has_last_time_;
  float dt = config_.nominal_period;
  if (!first_frame) {
    const auto elapsed_us = static_cast<float>(frame.time_us - last_time_us_);
    dt = std::clamp(elapsed_us * 1e-6F, 0.25F * config_.nominal_period,
                    4.0F * config_.nominal_period);
  }
  last_time_us_ = frame.time_us;
  has_last_time_ = true;

  // IMUs: body frame, bias learned only while disarmed (the robot must not be balancing).
  const bool learning = safety_.state() == balance::SafetyState::kDisarmed;
  std::optional<balance::ImuSampleBody> primary;
  std::optional<balance::ImuSampleBody> secondary;
  if (frame.primary) {
    primary = calibrate(toBody(*frame.primary, config_.primary_mounting), primary_bias_, learning);
  }
  if (frame.secondary) {
    secondary =
        calibrate(toBody(*frame.secondary, config_.secondary_mounting), secondary_bias_, learning);
  }
  if (primary) {
    out.primary_sample = toSample(*primary, protocol::ImuSensor::kMpu9250, frame.time_us);
  }
  if (secondary) {
    out.secondary_sample = toSample(*secondary, protocol::ImuSensor::kMpu6050, frame.time_us);
  }

  const auto fused = fusion_.update(primary, secondary, dt);
  balance::Attitude attitude = filter_.attitude();
  if (fused) {
    attitude = filter_.update(fused->accel, fused->gyro, dt);
  }
  const float yaw_rate = fused ? fused->gyro.z : 0.0F;
  const float pitch = fused ? attitude.pitch : std::numeric_limits<float>::quiet_NaN();

  const balance::SafetyUpdate safety = safety_.update(pitch, fused.has_value(), dt);
  if (safety.started_balancing) {
    controller_.reset();
  }

  // Wheels: angles relative to the chassis, rates, and ground velocity.
  const float left_angle = wheelAngle(frame.left_ticks, config_.left_encoder_inverted);
  const float right_angle = wheelAngle(frame.right_ticks, config_.right_encoder_inverted);
  if (first_frame) {
    last_left_angle_ = left_angle;  // no velocity spike from a non-zero starting count
    last_right_angle_ = right_angle;
  }
  const float left_rate = (left_angle - last_left_angle_) / dt;
  const float right_rate = (right_angle - last_right_angle_) / dt;
  last_left_angle_ = left_angle;
  last_right_angle_ = right_angle;
  const balance::BodyTwist twist =
      balance::bodyTwist(config_.geometry, left_rate, right_rate, attitude.pitch_rate);

  if (safety_.motorsEnabled()) {
    balance::BalanceInput input;
    input.pitch = attitude.pitch;
    input.pitch_rate = attitude.pitch_rate;
    input.yaw_rate = yaw_rate;
    input.forward_velocity = twist.linear;
    input.target_velocity = command.link_alive ? command.linear : 0.0F;
    input.target_yaw_rate = command.link_alive ? command.angular : 0.0F;
    input.dt = dt;
    const balance::BalanceOutput result = controller_.update(input);
    out.left = result.left;
    out.right = result.right;
    out.motors_enabled = true;
  }

  protocol::Telemetry& t = out.telemetry;
  t.timestamp_us = frame.time_us;
  t.state = static_cast<protocol::RobotState>(safety_.state());
  t.faults = safety_.faults();
  if (fusion_.disagreement()) {
    t.faults |= protocol::fault::kImuDisagreement;
  }
  if (!command.link_alive) {
    t.faults |= protocol::fault::kLinkTimeout;
  }
  if (overrun_) {
    t.faults |= protocol::fault::kLoopOverrun;
  }
  t.roll = attitude.roll;
  t.pitch = attitude.pitch;
  t.pitch_rate = attitude.pitch_rate;
  t.yaw_rate = yaw_rate;
  t.left_ticks = config_.left_encoder_inverted ? -frame.left_ticks : frame.left_ticks;
  t.right_ticks = config_.right_encoder_inverted ? -frame.right_ticks : frame.right_ticks;
  t.left_wheel_velocity = left_rate;
  t.right_wheel_velocity = right_rate;
  t.left_duty = out.left;
  t.right_duty = out.right;
  t.max_loop_time_us = max_tick_us_;

  out.state_changed = safety_.state() != last_state_;
  last_state_ = safety_.state();
  return out;
}

void ControlLoop::reportTickDuration(std::uint32_t duration_us) noexcept {
  const auto clamped = static_cast<std::uint16_t>(std::min<std::uint32_t>(duration_us, 0xFFFFU));
  max_tick_us_ = std::max(max_tick_us_, clamped);
  overrun_ = static_cast<float>(duration_us) * 1e-6F > config_.nominal_period;
}

protocol::Ack ControlLoop::handle(const protocol::Message& message,
                                  std::uint8_t sequence) noexcept {
  protocol::Ack ack;
  ack.acked_type = protocol::typeOf(message);
  ack.acked_sequence = sequence;
  ack.status = protocol::AckStatus::kInvalid;

  if (const auto* arm = std::get_if<protocol::SetArmed>(&message)) {
    if (arm->armed) {
      ack.status = safety_.requestArm() ? protocol::AckStatus::kOk : protocol::AckStatus::kRejected;
    } else {
      safety_.requestDisarm();
      ack.status = protocol::AckStatus::kOk;
    }
  } else if (const auto* gains = std::get_if<protocol::SetGains>(&message)) {
    if (validGains(*gains)) {
      controller_.setGains(static_cast<balance::Loop>(gains->loop),
                           {gains->kp, gains->ki, gains->kd});
      ack.status = protocol::AckStatus::kOk;
    }
  }
  return ack;
}

}  // namespace robot::app
