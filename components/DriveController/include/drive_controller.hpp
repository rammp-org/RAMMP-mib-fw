#pragma once

#include <cstdint>

#include "base_component.hpp"
#include "differential_drive.hpp"
#include "MIBconfig.hpp"
#include "messages.hpp"
#include "trajectory_planner.hpp"

namespace mib {

/// @brief Coordinates chassis trajectory planning and differential-drive kinematics.
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

  /// @brief Apply one of the predefined trajectory-planner drive profiles.
  /// @param profile The desired drive response profile.
  /// @return True if the planner accepted the profile configuration.
  bool update_drive_profile(MIB::DriveProfile profile);

  /// @brief Get the current trajectory-limited wheel speeds.
  /// @return Left and right wheel speeds in RPM.
  DifferentialDrive::WheelSpeeds wheel_speeds() const;

  /// @brief Check whether the differential-drive geometry is valid.
  /// @return True when both wheel dimensions are positive.
  bool is_valid() const { return differential_drive_.is_valid(); }

  /// @brief Accept a seat target for the future seat actuator controller.
  /// @param axis Seat actuator identifier.
  /// @param target Target position in the axis' configured units.
  void set_seat_target(uint8_t axis, float target);

  /// @brief Access the trajectory planner.
  /// @return Reference to the owned trajectory planner.
  espp::TrajectoryPlanner &trajectory_planner() { return trajectory_planner_; }

  /// @brief Access the differential-drive model.
  /// @return Reference to the owned differential-drive model.
  DifferentialDrive &differential_drive() { return differential_drive_; }

private:
  static Config default_config();
  static espp::TrajectoryPlanner::Config profile_config(MIB::DriveProfile profile);

  espp::TrajectoryPlanner trajectory_planner_;
  DifferentialDrive differential_drive_;
  MIB::DriveProfile active_profile_{MIB::DriveProfile::NORMAL};
};

} // namespace mib
