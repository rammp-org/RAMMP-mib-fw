#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>

#include "base_component.hpp"
#include "differential_drive.hpp"
#include "drive_response.hpp"
#include "MIBconfig.hpp"
#include "messages.hpp"
#include "timer.hpp"

namespace mib {

/// @brief Calculates rate-limited chassis and wheel-speed commands.
///
/// Joystick targets go through a DriveResponse (R-Net style shaping and
/// accel/decel limits), updated on an internal timer, then through the
/// differential-drive kinematics.
///
/// This component does not directly control motors. A motor-command layer must
/// consume wheel_speeds() and send the resulting RPM values to the controllers.
class DriveController : public espp::BaseComponent {
public:
  DriveController();

  /// @brief Set a normalized chassis velocity target.
  /// @param linear Normalized linear command in [-1, 1].
  /// @param angular Normalized angular command in [-1, 1].
  void set_target(float linear, float angular);

  /// @brief Set the chassis target to zero using the profile's decel limits.
  /// @note This does not immediately set motor speed to zero; the response
  ///       decelerates toward the zero target.
  void stop();

  /// @brief Apply one of the predefined drive profiles.
  /// @param profile The desired drive response profile.
  /// @return True if the profile configuration was valid and applied.
  /// @note The current velocity is kept, so switching while driving ramps
  ///       to the new limits instead of stepping.
  bool update_drive_profile(MIB::DriveProfile profile);

  /// @brief Get the current rate-limited wheel speeds.
  /// @return Left and right wheel speeds in RPM.
  /// @note This only calculates the requested output; it does not command motors.
  DifferentialDrive::WheelSpeeds wheel_speeds() const;

  /// @brief Check whether the differential-drive geometry is valid.
  /// @return True when both wheel dimensions are positive.
  bool is_valid() const { return differential_drive_.is_valid(); }

  /// @brief Accept a seat target for the future seat actuator controller.
  /// @param axis Seat actuator identifier.
  /// @param target Target position in the axis' configured units.
  void set_seat_target(uint8_t axis, float target);

private:
  static DriveResponse::Config profile_config(MIB::DriveProfile profile);

  mutable std::mutex response_mutex_;
  DriveResponse response_;
  DifferentialDrive differential_drive_;
  MIB::DriveProfile active_profile_{MIB::DriveProfile::NORMAL};
  std::chrono::steady_clock::time_point last_update_time_{std::chrono::steady_clock::now()};
  std::unique_ptr<espp::Timer> update_timer_;
};

} // namespace mib
