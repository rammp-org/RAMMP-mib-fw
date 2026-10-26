#pragma once

#include <cstdint>
#include <mutex>

#include "base_component.hpp"
#include "differential_drive.hpp"
#include "drive_profile.hpp"
#include "MIBconfig.hpp"
#include "messages.hpp"
#include "trajectory_planner.hpp"

namespace mib {

/// @brief Calculates trajectory-limited chassis and wheel-speed commands.
///
/// This component does not directly control motors. A motor-command layer must
/// consume wheel_speeds() and send the resulting RPM values to the controllers.
class DriveController : public espp::BaseComponent {
public:
  struct Config {
    espp::TrajectoryPlanner::Config trajectory_planner{};
    DifferentialDrive::Config differential_drive{};
  };

  DriveController();
  explicit DriveController(const Config &config);

  /// @brief Set a normalized chassis velocity target.
  /// @param linear Normalized linear command in [-1, 1].
  /// @param angular Normalized angular command in [-1, 1].
  void set_target(float linear, float angular);

  /// @brief Set the chassis target to zero using the planner limits.
  /// @note This does not immediately set motor speed to zero; the planner
  ///       decelerates toward the zero target.
  void stop();

  /// @brief Make a drive profile the active one and apply its limits.
  /// @param profile The desired drive response profile.
  /// @param values The profile's limits, from DriveProfileStore.
  /// @return True if the planner accepted the profile configuration.
  /// @note The planner keeps its current velocity, so switching while driving
  ///       ramps to the new limits instead of stepping to zero.
  bool update_drive_profile(MIB::DriveProfile profile, const config::DriveProfileConfig &values);

  /// @brief Re-apply a profile's limits after they changed, if it is the active one.
  /// @param profile The profile whose limits changed.
  /// @param values Its new limits.
  /// @return True if the profile is not active, or the planner accepted the limits.
  /// @note Like update_drive_profile(), keeps the current velocity.
  bool refresh_drive_profile(MIB::DriveProfile profile, const config::DriveProfileConfig &values);

  /// @brief Get the current trajectory-limited wheel speeds.
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
  static Config default_config();
  bool apply_locked(MIB::DriveProfile profile, const config::DriveProfileConfig &values);

  espp::TrajectoryPlanner trajectory_planner_;
  DifferentialDrive differential_drive_;
  std::mutex profile_mutex_; ///< Orders profile switches against limit changes.
  MIB::DriveProfile active_profile_{MIB::DriveProfile::NORMAL};
};

} // namespace mib
