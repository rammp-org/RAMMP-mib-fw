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
  const auto config = trajectory_planner_.get_config();
  const float max_linear = linear_mps < 0.0f && config.max_reverse_linear_velocity > 0.0f
                               ? config.max_reverse_linear_velocity
                               : config.max_linear_velocity;
  const float linear = linear_mps / max_linear;
  const float angular = angular_radps / config.max_angular_velocity;
  const float scale = std::max({1.0f, std::abs(linear), std::abs(angular)});
  trajectory_planner_.set_target(linear / scale, angular / scale);
}

void DriveController::stop() {
  trajectory_planner_.stop();
}

DriveController::Config DriveController::default_config() {
  return Config{
      .trajectory_planner = profile_config(rammp::DriveProfile::NORMAL),
      .differential_drive = {
          .wheel_diameter_m = config::differential_drive.wheel_diameter_m,
          .wheel_separation_m = config::differential_drive.wheel_separation_m,
          .invert_left = config::differential_drive.invert_left,
          .invert_right = config::differential_drive.invert_right,
      },
  };
}

bool DriveController::update_drive_profile(rammp::DriveProfile profile) {
  const bool accepted = trajectory_planner_.set_config(profile_config(profile));
  if (!accepted) {
    logger_.error("Failed to apply drive profile {}", static_cast<uint8_t>(profile));
    return false;
  }

  active_profile_ = profile;
  logger_.info("Drive profile updated to {}", static_cast<uint8_t>(profile));
  return true;
}

espp::TrajectoryPlanner::Config
DriveController::profile_config(rammp::DriveProfile profile) {
  espp::TrajectoryPlanner::Config config{};
  switch (profile) {
  case rammp::DriveProfile::LOW:
    config.max_linear_velocity = config::drive_profile_low.max_linear_velocity_mps;
    config.max_angular_velocity = config::drive_profile_low.max_angular_velocity_radps;
    config.driving_profile.max_linear_acceleration =
      config::drive_profile_low.max_linear_acceleration_mps2;
    config.driving_profile.max_angular_acceleration =
      config::drive_profile_low.max_angular_acceleration_radps2;
    config.stopping_profile = config.driving_profile;
    break;
  case rammp::DriveProfile::NORMAL:
    config.max_linear_velocity = config::drive_profile_normal.max_linear_velocity_mps;
    config.max_angular_velocity = config::drive_profile_normal.max_angular_velocity_radps;
    config.driving_profile.max_linear_acceleration =
      config::drive_profile_normal.max_linear_acceleration_mps2;
    config.driving_profile.max_angular_acceleration =
      config::drive_profile_normal.max_angular_acceleration_radps2;
    config.stopping_profile = config.driving_profile;
    break;
  case rammp::DriveProfile::HIGH:
    config.max_linear_velocity = config::drive_profile_high.max_linear_velocity_mps;
    config.max_angular_velocity = config::drive_profile_high.max_angular_velocity_radps;
    config.driving_profile.max_linear_acceleration =
      config::drive_profile_high.max_linear_acceleration_mps2;
    config.driving_profile.max_angular_acceleration =
      config::drive_profile_high.max_angular_acceleration_radps2;
    config.stopping_profile = config.driving_profile;
    break;
  }
  return config;
}

DifferentialDrive::WheelSpeeds DriveController::wheel_speeds() const {
  const auto command = trajectory_planner_.output();
  return differential_drive_.compute(command.linear_velocity, command.angular_velocity);
}

void DriveController::set_seat_target(uint8_t axis, float target) {
  logger_.info("Seat target axis={} target={} - actuator control placeholder", axis, target);
}

} // namespace mib
