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
      .trajectory_planner = profile_config(MIB::DriveProfile::NORMAL),
      .differential_drive = geometry_from_params(),
  };
}

DifferentialDrive::Config DriveController::geometry_from_params() {
  auto &params = Params::instance();
  return DifferentialDrive::Config{
      .wheel_diameter_m = params.get(ParamId::WHEEL_DIAMETER_M),
      .wheel_separation_m = params.get(ParamId::WHEEL_SEPARATION_M),
      .invert_left = params.get_bool(ParamId::INVERT_LEFT),
      .invert_right = params.get_bool(ParamId::INVERT_RIGHT),
  };
}

bool DriveController::apply_params() {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool geometry_ok = differential_drive_.set_config(geometry_from_params());
  // Keep the planner's current state so a limit change while driving does not
  // snap the output to zero; the new limits take effect on the next update.
  const bool limits_ok = trajectory_planner_.set_config(profile_config(active_profile_), false);
  if (!geometry_ok || !limits_ok) {
    logger_.error("apply_params: geometry_ok={} limits_ok={}", geometry_ok, limits_ok);
  }
  return geometry_ok && limits_ok;
}

bool DriveController::update_drive_profile(MIB::DriveProfile profile) {
  std::lock_guard<std::mutex> lock(mutex_);
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
  // Each profile owns four consecutive rows of the parameter table, in the order
  // max linear velocity, max angular velocity, linear acceleration, angular
  // acceleration, starting at LOW_MAX_LINEAR_V.
  auto &params = Params::instance();
  const auto base = static_cast<uint8_t>(ParamId::LOW_MAX_LINEAR_V) +
                    4 * static_cast<uint8_t>(profile);
  const auto row = [&](uint8_t offset) {
    return params.get(static_cast<ParamId>(base + offset));
  };
  espp::TrajectoryPlanner::Config config{};
  config.max_linear_velocity = row(0);
  config.max_angular_velocity = row(1);
  config.driving_profile.max_linear_acceleration = row(2);
  config.driving_profile.max_angular_acceleration = row(3);
  config.stopping_profile = config.driving_profile;
  return config;
}

DifferentialDrive::WheelSpeeds DriveController::wheel_speeds() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto command = trajectory_planner_.output();
  return differential_drive_.compute(command.linear_velocity, command.angular_velocity);
}

void DriveController::set_seat_target(uint8_t axis, float target) {
  logger_.info("Seat target axis={} target={} - actuator control placeholder", axis, target);
}

} // namespace mib
