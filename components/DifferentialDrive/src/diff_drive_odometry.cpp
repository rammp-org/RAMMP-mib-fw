#include "diff_drive_odometry.hpp"

#include <cmath>
#include <numbers>

namespace mib {

DiffDriveOdometry::DiffDriveOdometry(const Config &config)
    : wheel_radius_m_(config.wheel_diameter_m / 2.0)
    , wheel_separation_m_(config.wheel_separation_m)
    , left_sign_(config.invert_left ? -1.0 : 1.0)
    , right_sign_(config.invert_right ? -1.0 : 1.0) {}

void DiffDriveOdometry::update(float left_rad, float right_rad) {
  if (!synced_) {
    synced_ = true;
    last_left_rad_ = left_rad;
    last_right_rad_ = right_rad;
    return;
  }

  // Difference in double: a large multiturn float angle would lose a small step.
  const double left_m = (double(left_rad) - last_left_rad_) * left_sign_ * wheel_radius_m_;
  const double right_m = (double(right_rad) - last_right_rad_) * right_sign_ * wheel_radius_m_;
  last_left_rad_ = left_rad;
  last_right_rad_ = right_rad;

  // Move along the heading at the middle of the step (second-order integration).
  const double distance_m = (left_m + right_m) / 2.0;
  const double rotation_rad = (right_m - left_m) / wheel_separation_m_;
  const double heading_rad = pose_.yaw_rad + rotation_rad / 2.0;
  pose_.x_m += distance_m * std::cos(heading_rad);
  pose_.y_m += distance_m * std::sin(heading_rad);
  pose_.yaw_rad = std::remainder(pose_.yaw_rad + rotation_rad, 2.0 * std::numbers::pi);
}

DiffDriveOdometry::Velocity DiffDriveOdometry::velocity(float left_radps, float right_radps) const {
  const double left_mps = left_radps * left_sign_ * wheel_radius_m_;
  const double right_mps = right_radps * right_sign_ * wheel_radius_m_;
  return Velocity{
      .linear_mps = (left_mps + right_mps) / 2.0,
      .angular_radps = (right_mps - left_mps) / wheel_separation_m_,
  };
}

} // namespace mib
