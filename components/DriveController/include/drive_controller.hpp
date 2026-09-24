#pragma once

#include <cstdint>

#include "base_component.hpp"
#include "differential_drive.hpp"
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

  /// @brief Apply one of the predefined trajectory-planner drive profiles.
  /// @param profile The desired drive response profile.
  /// @return True if the planner accepted the profile configuration.
  bool update_drive_profile(MIB::DriveProfile profile);

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
  static espp::TrajectoryPlanner::Config profile_config(MIB::DriveProfile profile);

  espp::TrajectoryPlanner trajectory_planner_;
  DifferentialDrive differential_drive_;
  MIB::DriveProfile active_profile_{MIB::DriveProfile::NORMAL};
};

} // namespace mib
