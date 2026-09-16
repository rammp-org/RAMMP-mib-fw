#include "differential_drive.hpp"

#include <numbers>

namespace mib {

DifferentialDrive::DifferentialDrive(const Config &config)
    : BaseComponent("DifferentialDrive", espp::Logger::Verbosity::INFO),
      wheel_diameter_m_(config.wheel_diameter_m),
      wheel_separation_m_(config.wheel_separation_m),
      invert_left_(config.invert_left),
      invert_right_(config.invert_right),
      valid_(config.wheel_diameter_m > 0.0f && config.wheel_separation_m > 0.0f) {
  if (!valid_) {
    logger_.warn("Invalid geometry: wheel diameter={} m, separation={} m; both must be > 0. "
                 "Wheel speeds will be zero.",
                 config.wheel_diameter_m, config.wheel_separation_m);
  }
}

DifferentialDrive::WheelSpeeds DifferentialDrive::compute(float linear_mps,
                                                          float angular_radps) const {
  if (!valid_) {
    return WheelSpeeds{};
  }

  // Rotation adds half of the commanded arc speed to one wheel and removes it from the other.
  const float half_separation = wheel_separation_m_ / 2.0f;
  const float left_mps = linear_mps - angular_radps * half_separation;
  const float right_mps = linear_mps + angular_radps * half_separation;

  const float circumference_m = std::numbers::pi_v<float> * wheel_diameter_m_;
  const float mps_to_rpm = 60.0f / circumference_m;

  return WheelSpeeds{
      .left_rpm = left_mps * mps_to_rpm * (invert_left_ ? -1.0f : 1.0f),
      .right_rpm = right_mps * mps_to_rpm * (invert_right_ ? -1.0f : 1.0f),
  };
}

} // namespace mib
