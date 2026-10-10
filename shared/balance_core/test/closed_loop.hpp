#pragma once

// Closed-loop harness: the WheeledPendulum plant with realistic sensors, run through the same
// pipeline as the firmware (bias calibration -> complementary filter -> safety -> controller).

#include <cmath>
#include <cstdint>
#include <random>

#include "balance_core/attitude.hpp"
#include "balance_core/balance_controller.hpp"
#include "balance_core/odometry.hpp"
#include "balance_core/safety.hpp"
#include "balance_core/wheeled_pendulum.hpp"

namespace robot::balance::testing {

struct SensorModel {
  float gyro_bias_y = 0.02F;       ///< [rad/s] uncalibrated offset
  float gyro_bias_z = -0.01F;      ///< [rad/s]
  float gyro_noise = 0.003F;       ///< [rad/s] standard deviation
  float accel_noise = 0.05F;       ///< [m/s^2] standard deviation
  float counts_per_rev = 4200.0F;  ///< encoder counts per wheel revolution (x4)
  unsigned seed = 1;
};

class ClosedLoop {
 public:
  static constexpr float kDt = 0.005F;  // 200 Hz

  explicit ClosedLoop(sim::PendulumState initial = {}, BalanceConfig config = {},
                      SensorModel sensors = {}, sim::PendulumParams params = {})
      : plant_(params, initial),
        controller_(config),
        sensors_(sensors),
        rng_(sensors.seed),
        geometry_{params.wheel_radius, params.track_width} {}

  /// Robot held still: calibrate the gyro bias and let the attitude filter converge.
  void calibrate() {
    for (int i = 0; i < 2000 && !bias_.ready(); ++i) {
      bias_.update(rawGyro());
    }
    for (int i = 0; i < 400; ++i) {
      const Attitude attitude = estimate();
      (void)safety_.update(attitude.pitch, true, kDt);
    }
  }

  /// Arms, then runs until balancing starts or `timeout` passes.
  bool armAndStart(float timeout = 2.0F) {
    safety_.requestArm();
    for (float t = 0.0F; t < timeout; t += kDt) {
      tick(0.0F, 0.0F);
      if (safety_.state() == SafetyState::kBalancing) {
        return true;
      }
    }
    return false;
  }

  /// Runs for `seconds` with the given commands. Tracks the largest |pitch| seen.
  void run(float seconds, float target_velocity = 0.0F, float target_yaw_rate = 0.0F) {
    const int ticks = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < ticks; ++i) {
      tick(target_velocity, target_yaw_rate);
    }
  }

  void tick(float target_velocity, float target_yaw_rate) {
    const Attitude attitude = estimate();
    const SafetyUpdate safety = safety_.update(attitude.pitch, true, kDt);
    if (safety.started_balancing) {
      controller_.reset();
    }

    // Encoders: quantized wheel angles relative to the chassis.
    const float left = quantize(plant_.state().left_wheel_angle);
    const float right = quantize(plant_.state().right_wheel_angle);
    const float left_rate = (left - last_left_) / kDt;
    const float right_rate = (right - last_right_) / kDt;
    last_left_ = left;
    last_right_ = right;
    const BodyTwist twist = bodyTwist(geometry_, left_rate, right_rate, attitude.pitch_rate);

    BalanceOutput command{};
    if (safety_.motorsEnabled()) {
      BalanceInput input;
      input.pitch = attitude.pitch;
      input.pitch_rate = attitude.pitch_rate;
      input.yaw_rate = last_gyro_.z;
      input.forward_velocity = twist.linear;
      input.target_velocity = target_velocity;
      input.target_yaw_rate = target_yaw_rate;
      input.dt = kDt;
      command = controller_.update(input);
    }
    last_command_ = command;
    plant_.step(command.left, command.right, kDt);
    time_ += kDt;
    if (safety_.state() == SafetyState::kBalancing) {
      max_abs_pitch_ = std::fmax(max_abs_pitch_, std::fabs(plant_.state().pitch));
    }
  }

  sim::WheeledPendulum& plant() { return plant_; }
  BalanceController& controller() { return controller_; }
  SafetyStateMachine& safety() { return safety_; }
  [[nodiscard]] float maxAbsPitch() const { return max_abs_pitch_; }
  void resetMaxAbsPitch() { max_abs_pitch_ = 0.0F; }
  [[nodiscard]] const BalanceOutput& lastCommand() const { return last_command_; }
  [[nodiscard]] float time() const { return time_; }

 private:
  Vec3 rawGyro() {
    std::normal_distribution<float> noise(0.0F, sensors_.gyro_noise);
    const Vec3 ideal = plant_.gyroscope();
    return {ideal.x + noise(rng_), ideal.y + sensors_.gyro_bias_y + noise(rng_),
            ideal.z + sensors_.gyro_bias_z + noise(rng_)};
  }

  Vec3 rawAccel() {
    std::normal_distribution<float> noise(0.0F, sensors_.accel_noise);
    const Vec3 ideal = plant_.accelerometer();
    return {ideal.x + noise(rng_), ideal.y + noise(rng_), ideal.z + noise(rng_)};
  }

  // One tick of sensor delay, as the IMU low-pass filter and the read itself take time.
  Attitude estimate() {
    const Vec3 gyro = bias_.correct(delayed_gyro_);
    const Attitude attitude = filter_.update(delayed_accel_, gyro, kDt);
    last_gyro_ = gyro;
    delayed_gyro_ = rawGyro();
    delayed_accel_ = rawAccel();
    return attitude;
  }

  [[nodiscard]] float quantize(float angle) const {
    const float counts = std::floor(angle * sensors_.counts_per_rev / (2.0F * kPi));
    return counts * 2.0F * kPi / sensors_.counts_per_rev;
  }

  sim::WheeledPendulum plant_;
  BalanceController controller_;
  ComplementaryFilter filter_;
  GyroBiasEstimator bias_;
  SafetyStateMachine safety_;
  SensorModel sensors_;
  std::mt19937 rng_;
  WheelGeometry geometry_;
  Vec3 delayed_gyro_{};
  Vec3 delayed_accel_{0.0F, 0.0F, kGravity};
  Vec3 last_gyro_{};
  BalanceOutput last_command_{};
  float last_left_ = 0.0F;
  float last_right_ = 0.0F;
  float max_abs_pitch_ = 0.0F;
  float time_ = 0.0F;
};

}  // namespace robot::balance::testing
