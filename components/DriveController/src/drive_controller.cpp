#include "drive_controller.hpp"

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

DriveController::Config DriveController::default_config() {
  return Config{
      .trajectory_planner = profile_config(MIB::DriveProfile::NORMAL),
      .differential_drive = {
          .wheel_diameter_m = config::differential_drive.wheel_diameter_m,
          .wheel_separation_m = config::differential_drive.wheel_separation_m,
          .invert_left = config::differential_drive.invert_left,
          .invert_right = config::differential_drive.invert_right,
      },
  };
}

bool DriveController::update_drive_profile(MIB::DriveProfile profile) {
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
DriveController::profile_config(MIB::DriveProfile profile) {
  espp::TrajectoryPlanner::Config config{};
  switch (profile) {
  case MIB::DriveProfile::LOW:
    config.max_linear_velocity = config::drive_profile_low.max_linear_velocity_mps;
    config.max_angular_velocity = config::drive_profile_low.max_angular_velocity_radps;
    config.driving_profile.max_linear_acceleration =
      config::drive_profile_low.max_linear_acceleration_mps2;
    config.driving_profile.max_angular_acceleration =
      config::drive_profile_low.max_angular_acceleration_radps2;
    break;
  case MIB::DriveProfile::NORMAL:
    config.max_linear_velocity = config::drive_profile_normal.max_linear_velocity_mps;
    config.max_angular_velocity = config::drive_profile_normal.max_angular_velocity_radps;
    config.driving_profile.max_linear_acceleration =
      config::drive_profile_normal.max_linear_acceleration_mps2;
    config.driving_profile.max_angular_acceleration =
      config::drive_profile_normal.max_angular_acceleration_radps2;
    break;
  case MIB::DriveProfile::HIGH:
    config.max_linear_velocity = config::drive_profile_high.max_linear_velocity_mps;
    config.max_angular_velocity = config::drive_profile_high.max_angular_velocity_radps;
    config.driving_profile.max_linear_acceleration =
      config::drive_profile_high.max_linear_acceleration_mps2;
    config.driving_profile.max_angular_acceleration =
      config::drive_profile_high.max_angular_acceleration_radps2;
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
