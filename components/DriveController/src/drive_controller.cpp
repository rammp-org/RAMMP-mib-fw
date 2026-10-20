#include "drive_controller.hpp"

#include "MIBconfig.hpp"

namespace mib {

DriveController::DriveController() : DriveController(default_config()) {}

DriveController::DriveController(const Config &config)
    : BaseComponent("DriveController", espp::Logger::Verbosity::INFO),
      trajectory_planner_(config.trajectory_planner),
      input_shaping_(config.input_shaping),
      differential_drive_(config.differential_drive) {
  if (!differential_drive_.is_valid()) {
    logger_.warn("Drive controller has invalid differential-drive geometry");
  }
}

void DriveController::set_target(float linear, float angular) {
  DriveInputShaping shaping;
  {
    std::lock_guard<std::mutex> lock(input_shaping_mutex_);
    shaping = input_shaping_;
  }
  const auto shaped = shape_drive_input(linear, angular, shaping);
  trajectory_planner_.set_target(shaped.linear, shaped.angular);
}

void DriveController::stop() {
  trajectory_planner_.stop();
}

DriveController::Config DriveController::default_config() {
  return Config{
      .trajectory_planner = profile_config(MIB::DriveProfile::NORMAL),
      .input_shaping = profile_shaping(MIB::DriveProfile::NORMAL),
      .differential_drive = {
          .wheel_diameter_m = config::differential_drive.wheel_diameter_m,
          .wheel_separation_m = config::differential_drive.wheel_separation_m,
          .invert_left = config::differential_drive.invert_left,
          .invert_right = config::differential_drive.invert_right,
      },
  };
}

bool DriveController::update_drive_profile(MIB::DriveProfile profile) {
  // Keep the current velocity so a profile change while driving ramps instead of
  // dropping the wheels to zero.
  const bool accepted =
      trajectory_planner_.set_config(profile_config(profile), /*reset_state=*/false);
  if (!accepted) {
    logger_.error("Failed to apply drive profile {}", static_cast<uint8_t>(profile));
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(input_shaping_mutex_);
    input_shaping_ = profile_shaping(profile);
  }

  active_profile_ = profile;
  logger_.info("Drive profile updated to {}", static_cast<uint8_t>(profile));
  return true;
}

const config::DriveProfileConfig &DriveController::profile_values(MIB::DriveProfile profile) {
  switch (profile) {
  case MIB::DriveProfile::LOW:
    return config::drive_profile_low;
  case MIB::DriveProfile::HIGH:
    return config::drive_profile_high;
  case MIB::DriveProfile::NORMAL:
  default:
    return config::drive_profile_normal;
  }
}

espp::TrajectoryPlanner::Config
DriveController::profile_config(MIB::DriveProfile profile) {
  return planner_config(profile_values(profile));
}

DriveInputShaping DriveController::profile_shaping(MIB::DriveProfile profile) {
  return input_shaping(profile_values(profile));
}

DifferentialDrive::WheelSpeeds DriveController::wheel_speeds() const {
  const auto command = trajectory_planner_.output();
  return differential_drive_.compute(command.linear_velocity, command.angular_velocity);
}

void DriveController::set_seat_target(uint8_t axis, float target) {
  logger_.info("Seat target axis={} target={} - actuator control placeholder", axis, target);
}

} // namespace mib
