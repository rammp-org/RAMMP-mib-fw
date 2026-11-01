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

const config::DriveProfileConfig &DriveController::profile_values(rammp::DriveProfile profile) {
  switch (profile) {
  case rammp::DriveProfile::LOW:
    return config::drive_profile_low;
  case rammp::DriveProfile::HIGH:
    return config::drive_profile_high;
  case rammp::DriveProfile::NORMAL:
  default:
    return config::drive_profile_normal;
  }
}

espp::TrajectoryPlanner::Config
DriveController::profile_config(rammp::DriveProfile profile) {
  const config::DriveProfileConfig &values = profile_values(profile);
  espp::TrajectoryPlanner::Config config{};
  config.max_linear_velocity = values.max_linear_velocity_mps;
  config.max_angular_velocity = values.max_angular_velocity_radps;
  config.driving_profile.max_linear_acceleration = values.max_linear_acceleration_mps2;
  config.driving_profile.max_angular_acceleration = values.max_angular_acceleration_radps2;
  config.driving_profile.max_linear_jerk = values.max_linear_jerk_mps3;
  config.driving_profile.max_angular_jerk = values.max_angular_jerk_radps3;
  // Same accelerations, trapezoidal: espp recommends jerk 0 when stopping so the
  // S-curve cannot overshoot past zero.
  config.stopping_profile = config.driving_profile;
  config.stopping_profile.max_linear_jerk = 0.0f;
  config.stopping_profile.max_angular_jerk = 0.0f;
  return config;
}

float DriveController::max_wheel_acceleration_radps2() const {
  const config::DriveProfileConfig &values = profile_values(active_profile_);
  const float radius_m = differential_drive_.wheel_diameter() / 2.0f;
  if (radius_m <= 0.0f) {
    return 0.0f;
  }
  const float half_separation_m = differential_drive_.wheel_separation() / 2.0f;
  return (values.max_linear_acceleration_mps2 +
          values.max_angular_acceleration_radps2 * half_separation_m) /
         radius_m;
}

DifferentialDrive::WheelSpeeds DriveController::wheel_speeds() const {
  const auto command = trajectory_planner_.output();
  return differential_drive_.compute(command.linear_velocity, command.angular_velocity);
}

void DriveController::set_seat_target(uint8_t axis, float target) {
  logger_.info("Seat target axis={} target={} - actuator control placeholder", axis, target);
}

} // namespace mib
