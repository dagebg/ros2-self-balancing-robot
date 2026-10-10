#include "drivers/tb6612.hpp"

#include <algorithm>
#include <cmath>

namespace robot::drivers {

HBridgeChannel::HBridgeChannel(hal::PwmOutput& pwm, hal::DigitalOutput& in1,
                               hal::DigitalOutput& in2, bool inverted, float max_duty) noexcept
    : pwm_(pwm),
      in1_(in1),
      in2_(in2),
      inverted_(inverted),
      max_duty_(std::isfinite(max_duty) ? std::clamp(max_duty, 0.0F, 1.0F) : 0.0F) {}

void HBridgeChannel::set(float command) noexcept {
  if (!std::isfinite(command) || command == 0.0F) {
    coast();
    return;
  }
  command_ = std::clamp(command, -1.0F, 1.0F);
  applied_duty_ = std::fabs(command_) * max_duty_;

  const bool forward = (command_ > 0.0F) != inverted_;
  pwm_.setDuty(0.0F);  // never drive while the direction pins change
  in1_.write(forward);
  in2_.write(!forward);
  pwm_.setDuty(applied_duty_);
}

void HBridgeChannel::brake() noexcept {
  command_ = 0.0F;
  applied_duty_ = 0.0F;
  pwm_.setDuty(0.0F);
  in1_.write(true);
  in2_.write(true);
}

void HBridgeChannel::coast() noexcept {
  command_ = 0.0F;
  applied_duty_ = 0.0F;
  pwm_.setDuty(0.0F);
  in1_.write(false);
  in2_.write(false);
}

void Tb6612::enable() noexcept {
  standby_.write(true);
  enabled_ = true;
}

void Tb6612::disable() noexcept {
  standby_.write(false);
  enabled_ = false;
}

}  // namespace robot::drivers
