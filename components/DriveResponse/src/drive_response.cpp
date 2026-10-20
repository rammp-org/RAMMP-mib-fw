#include "drive_response.hpp"

#include <algorithm>
#include <cmath>

namespace mib {

namespace {

float clamp_unit(float value) {
  if (std::isnan(value)) {
    return 0.0f;
  }
  return std::clamp(value, -1.0f, 1.0f);
}

// Move current toward target. Speeding up in the same direction uses the accel
// limit; anything else (slowing, stopping, reversing) uses the decel limit.
float ramp(float current, float target, float accel, float decel, float dt_s) {
  const bool speeding_up = current * target >= 0.0f && std::abs(target) > std::abs(current);
  const float max_step = (speeding_up ? accel : decel) * dt_s;
  return current + std::clamp(target - current, -max_step, max_step);
}

} // namespace

DriveResponse::DriveResponse(const Config &config) : config_(config) {}

bool DriveResponse::is_valid(const Config &config) {
  return config.max_forward_mps > 0.0f && config.max_reverse_mps > 0.0f &&
         config.max_angular_radps > 0.0f && config.turn_reduction >= 0.0f &&
         config.turn_reduction < 1.0f && config.linear_accel_mps2 > 0.0f &&
         config.linear_decel_mps2 > 0.0f && config.angular_accel_radps2 > 0.0f &&
         config.angular_decel_radps2 > 0.0f;
}

bool DriveResponse::set_config(const Config &config) {
  if (!is_valid(config)) {
    return false;
  }
  config_ = config;
  return true;
}

void DriveResponse::set_target(float linear, float angular) {
  linear = clamp_unit(linear);
  angular = clamp_unit(angular);

  target_linear_mps_ = linear * (linear >= 0.0f ? config_.max_forward_mps : config_.max_reverse_mps);
  target_angular_radps_ =
      angular * config_.max_angular_radps * (1.0f - config_.turn_reduction * std::abs(linear));
}

void DriveResponse::update(float dt_s) {
  if (!(dt_s > 0.0f) || !is_valid(config_)) {
    return;
  }
  output_.linear_mps = ramp(output_.linear_mps, target_linear_mps_, config_.linear_accel_mps2,
                            config_.linear_decel_mps2, dt_s);
  output_.angular_radps = ramp(output_.angular_radps, target_angular_radps_,
                               config_.angular_accel_radps2, config_.angular_decel_radps2, dt_s);
}

void DriveResponse::reset() {
  target_linear_mps_ = 0.0f;
  target_angular_radps_ = 0.0f;
  output_ = Output{};
}

} // namespace mib
