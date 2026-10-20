#pragma once

// Drive profile mapping and joystick shaping. Header-only and free of ESP-IDF
// APIs beyond the planner, so tools/drive_fit can replay it on a host.

#include <algorithm>
#include <cmath>

#include "MIBconfig.hpp"
#include "trajectory_planner.hpp"

namespace mib {

/// @brief Joystick shaping applied before the trajectory planner.
struct DriveInputShaping {
  float deadband{0.0f};       ///< Stick magnitude snapped to zero, per axis, normalized.
  float reverse_scale{1.0f};  ///< Reverse stick is multiplied by this, in (0, 1].
  float turn_reduction{0.0f}; ///< Turn stick scales by (1 - turn_reduction * |linear|).
};

/// @brief Normalized planner target after shaping.
struct ShapedDriveInput {
  float linear{0.0f};  ///< Normalized linear command in [-1, 1].
  float angular{0.0f}; ///< Normalized angular command in [-1, 1].
};

/// @brief Shape a normalized joystick command for the trajectory planner.
/// @param linear Normalized linear command; +1 is full forward.
/// @param angular Normalized angular command; +1 is a full left turn.
/// @param shaping Deadband, reverse scale, and turn reduction to apply.
/// @return The shaped command. NaN inputs pass through for the planner to reject.
inline ShapedDriveInput shape_drive_input(float linear, float angular,
                                          const DriveInputShaping &shaping) {
  if (std::isnan(linear) || std::isnan(angular)) {
    return {linear, angular};
  }
  linear = std::clamp(linear, -1.0f, 1.0f);
  angular = std::clamp(angular, -1.0f, 1.0f);
  if (std::abs(linear) < shaping.deadband) {
    linear = 0.0f;
  }
  if (std::abs(angular) < shaping.deadband) {
    angular = 0.0f;
  }

  const float turn_scale = 1.0f - shaping.turn_reduction * std::abs(linear);
  return {
      .linear = linear < 0.0f ? linear * shaping.reverse_scale : linear,
      .angular = angular * turn_scale,
  };
}

/// @brief Planner configuration for one drive profile.
/// @param values The profile from MIBconfig.hpp.
/// @return Planner limits with trapezoidal (jerk-free) driving and stopping profiles.
inline espp::TrajectoryPlanner::Config planner_config(const config::DriveProfileConfig &values) {
  espp::TrajectoryPlanner::Config planner{};
  planner.max_linear_velocity = values.max_linear_velocity_mps;
  planner.max_angular_velocity = values.max_angular_velocity_radps;
  planner.driving_profile.max_linear_acceleration = values.max_linear_acceleration_mps2;
  planner.driving_profile.max_angular_acceleration = values.max_angular_acceleration_radps2;
  planner.stopping_profile.max_linear_acceleration = values.stopping_linear_acceleration_mps2;
  planner.stopping_profile.max_angular_acceleration = values.stopping_angular_acceleration_radps2;
  planner.max_centripetal_acceleration = values.max_centripetal_acceleration_mps2;
  return planner;
}

/// @brief Joystick shaping for one drive profile.
/// @param values The profile from MIBconfig.hpp.
inline DriveInputShaping input_shaping(const config::DriveProfileConfig &values) {
  return DriveInputShaping{
      .deadband = config::joystick_deadband,
      .reverse_scale = values.reverse_scale,
      .turn_reduction = values.turn_reduction,
  };
}

} // namespace mib
