#include "drive_controller.hpp"

#include "MIBconfig.hpp"

namespace mib {

DriveController::DriveController()
    : BaseComponent("DriveController", espp::Logger::Verbosity::INFO),
      response_(profile_config(MIB::DriveProfile::NORMAL)),
      differential_drive_({
          .wheel_diameter_m = config::differential_drive.wheel_diameter_m,
          .wheel_separation_m = config::differential_drive.wheel_separation_m,
          .invert_left = config::differential_drive.invert_left,
          .invert_right = config::differential_drive.invert_right,
      }) {
  if (!differential_drive_.is_valid()) {
    logger_.warn("Drive controller has invalid differential-drive geometry");
  }

  update_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "DriveResponse",
      .period = config::drive_response_interval,
      .callback = [this]() -> bool {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(response_mutex_);
        response_.update(std::chrono::duration<float>(now - last_update_time_).count());
        last_update_time_ = now;
        return false;
      },
      .stack_size_bytes = 4 * 1024,
  });
}

void DriveController::set_target(float linear, float angular) {
  std::lock_guard<std::mutex> lock(response_mutex_);
  response_.set_target(linear, angular);
}

void DriveController::stop() {
  set_target(0.0f, 0.0f);
}

bool DriveController::update_drive_profile(MIB::DriveProfile profile) {
  bool accepted = false;
  {
    std::lock_guard<std::mutex> lock(response_mutex_);
    accepted = response_.set_config(profile_config(profile));
  }
  if (!accepted) {
    logger_.error("Failed to apply drive profile {}", static_cast<uint8_t>(profile));
    return false;
  }

  active_profile_ = profile;
  logger_.info("Drive profile updated to {}", static_cast<uint8_t>(profile));
  return true;
}

DriveResponse::Config DriveController::profile_config(MIB::DriveProfile profile) {
  const config::DriveProfileConfig *source = &config::drive_profile_normal;
  switch (profile) {
  case MIB::DriveProfile::LOW:
    source = &config::drive_profile_low;
    break;
  case MIB::DriveProfile::NORMAL:
    source = &config::drive_profile_normal;
    break;
  case MIB::DriveProfile::HIGH:
    source = &config::drive_profile_high;
    break;
  }
  return DriveResponse::Config{
      .max_forward_mps = source->max_forward_mps,
      .max_reverse_mps = source->max_reverse_mps,
      .max_angular_radps = source->max_angular_radps,
      .turn_reduction = source->turn_reduction,
      .linear_accel_mps2 = source->linear_accel_mps2,
      .linear_decel_mps2 = source->linear_decel_mps2,
      .angular_accel_radps2 = source->angular_accel_radps2,
      .angular_decel_radps2 = source->angular_decel_radps2,
  };
}

DifferentialDrive::WheelSpeeds DriveController::wheel_speeds() const {
  DriveResponse::Output command;
  {
    std::lock_guard<std::mutex> lock(response_mutex_);
    command = response_.output();
  }
  return differential_drive_.compute(command.linear_mps, command.angular_radps);
}

void DriveController::set_seat_target(uint8_t axis, float target) {
  logger_.info("Seat target axis={} target={} - actuator control placeholder", axis, target);
}

} // namespace mib
