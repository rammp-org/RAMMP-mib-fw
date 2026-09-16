#include "differential_drive.hpp"

#include <numbers>

namespace mib {

DifferentialDrive::DifferentialDrive(const Config &config)
    : wheel_diameter_m_(config.wheel_diameter_m),
      wheel_separation_m_(config.wheel_separation_m) {}

DifferentialDrive::WheelSpeeds DifferentialDrive::compute(float linear_mps,
                                                          float angular_radps) const {
  // Rotation adds half of the commanded arc speed to one wheel and removes it from the other.
  const float half_separation = wheel_separation_m_ / 2.0f;
  const float left_mps = linear_mps - angular_radps * half_separation;
  const float right_mps = linear_mps + angular_radps * half_separation;

  const float circumference_m = std::numbers::pi_v<float> * wheel_diameter_m_;
  const float mps_to_rpm = 60.0f / circumference_m;

  return WheelSpeeds{
      .left_rpm = left_mps * mps_to_rpm,
      .right_rpm = right_mps * mps_to_rpm,
  };
}

} // namespace mib
