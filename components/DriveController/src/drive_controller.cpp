#include "drive_controller.hpp"

#include <algorithm>
#include <cmath>

#include "MIBconfig.hpp"

namespace mib {

DriveController::DriveController() : DriveController(default_config()) {}

DriveController::DriveController(const Config &config)
    : BaseComponent("DriveController", espp::Logger::Verbosity::INFO),
      trajectory_planner_(config.trajectory_planner),
      differential_drive_(config.differential_drive) {
  if (!differential_drive_.is_valid()) {
    logger_.warn("Drive controller has invalid differential-drive geometry");
  }
}

void DriveController::set_target(float linear, float angular) {
  trajectory_planner_.set_target(linear, angular);
}

void DriveController::set_velocity(float linear_mps, float angular_radps) {
  // get_config() returns a reference that set_config() overwrites, and a DriveProfileCommand
  // or DriveCommand can change the profile from another thread. Every set_config() runs
  // under profile_mutex_ (apply_locked), so holding it keeps the caps whole.
  std::lock_guard<std::mutex> lock(profile_mutex_);
  const auto &config = trajectory_planner_.get_config();
  const float max_linear = linear_mps < 0.0f && config.max_reverse_linear_velocity > 0.0f
                               ? config.max_reverse_linear_velocity
                               : config.max_linear_velocity;
  const float linear = linear_mps / max_linear;
  // The setpoint is already in the base frame (REP 103, positive counter-clockwise), the same
  // frame the planner uses. The stick's right-positive x is converted in MibSystem (#59).
  const float angular = angular_radps / config.max_angular_velocity;
  const float scale = std::max({1.0f, std::abs(linear), std::abs(angular)});
  trajectory_planner_.set_target(linear / scale, angular / scale);
}

void DriveController::stop() {
  trajectory_planner_.stop();
}

DriveController::Config DriveController::default_config() {
  return Config{
      .trajectory_planner = planner_config(config::drive_profile_normal),
      .differential_drive = {
          .wheel_diameter_m = config::differential_drive.wheel_diameter_m,
          .wheel_separation_m = config::differential_drive.wheel_separation_m,
          .invert_left = config::differential_drive.invert_left,
          .invert_right = config::differential_drive.invert_right,
      },
  };
}

bool DriveController::update_drive_profile(rammp::DriveProfile profile,
                                           const config::DriveProfileConfig &values) {
  std::lock_guard<std::mutex> lock(profile_mutex_);
  if (!apply_locked(profile, values)) {
    return false;
  }
  active_profile_ = profile;
  logger_.info("Drive profile updated to {}", static_cast<uint8_t>(profile));
  return true;
}

bool DriveController::refresh_drive_profile(rammp::DriveProfile profile,
                                            const config::DriveProfileConfig &values) {
  std::lock_guard<std::mutex> lock(profile_mutex_);
  if (profile != active_profile_) {
    return true;
  }
  if (!apply_locked(profile, values)) {
    return false;
  }
  logger_.info("Drive profile {} limits re-applied", static_cast<uint8_t>(profile));
  return true;
}

bool DriveController::apply_locked(rammp::DriveProfile profile,
                                   const config::DriveProfileConfig &values) {
  // Keep the current velocity so a profile change while driving ramps instead of
  // dropping the wheels to zero.
  const bool accepted =
      trajectory_planner_.set_config(planner_config(values), /*reset_state=*/false);
  if (!accepted) {
    logger_.error("Failed to apply drive profile {}", static_cast<uint8_t>(profile));
  }
  return accepted;
}

DifferentialDrive::WheelSpeeds DriveController::wheel_speeds() const {
  const auto command = trajectory_planner_.output();
  return differential_drive_.compute(command.linear_velocity, command.angular_velocity);
}

void DriveController::set_seat_target(uint8_t axis, float target) {
  logger_.info("Seat target axis={} target={} - actuator control placeholder", axis, target);
}

} // namespace mib
