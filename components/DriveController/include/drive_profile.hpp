#pragma once

// Maps a MIBconfig drive profile onto the trajectory planner. Header-only and
// free of ESP-IDF APIs beyond the planner, so host builds (tests, replays) can
// use it against the espp cross-platform library.

#include <cmath>

#include "MIBconfig.hpp"
#include "trajectory_planner.hpp"

namespace mib {

/// @brief Check a drive profile before it is used.
/// @param values The profile to check.
/// @return nullptr if the profile is usable, otherwise the name of the first
///         field that is not.
/// @note Mirrors the checks in espp::TrajectoryPlanner::set_config(), which
///       has no way to check a config without applying it, and adds the ones
///       it lacks: every field must be finite (a NaN passes its `<= 0` tests),
///       and jerk and centripetal limits must not be negative.
inline const char *invalid_field(const config::DriveProfileConfig &values) {
  const auto positive = [](float value) { return std::isfinite(value) && value > 0.0f; };
  const auto non_negative = [](float value) { return std::isfinite(value) && value >= 0.0f; };
  if (!positive(values.max_linear_velocity_mps))
    return "max_linear_velocity_mps";
  if (!non_negative(values.max_reverse_linear_velocity_mps))
    return "max_reverse_linear_velocity_mps";
  if (!positive(values.max_angular_velocity_radps))
    return "max_angular_velocity_radps";
  if (!positive(values.max_linear_acceleration_mps2))
    return "max_linear_acceleration_mps2";
  if (!positive(values.max_angular_acceleration_radps2))
    return "max_angular_acceleration_radps2";
  if (!non_negative(values.max_linear_jerk_mps3))
    return "max_linear_jerk_mps3";
  if (!non_negative(values.max_angular_jerk_radps3))
    return "max_angular_jerk_radps3";
  if (!positive(values.stopping_linear_acceleration_mps2))
    return "stopping_linear_acceleration_mps2";
  if (!positive(values.stopping_angular_acceleration_radps2))
    return "stopping_angular_acceleration_radps2";
  if (!non_negative(values.max_centripetal_acceleration_mps2))
    return "max_centripetal_acceleration_mps2";
  return nullptr;
}

/// @brief Planner configuration for one drive profile.
/// @param values The profile from MIBconfig.hpp.
/// @return Planner limits: S-curve driving where the profile sets a jerk limit,
///         trapezoidal stopping.
inline espp::TrajectoryPlanner::Config planner_config(const config::DriveProfileConfig &values) {
  espp::TrajectoryPlanner::Config planner{};
  planner.max_linear_velocity = values.max_linear_velocity_mps;
  planner.max_reverse_linear_velocity = values.max_reverse_linear_velocity_mps;
  planner.max_angular_velocity = values.max_angular_velocity_radps;
  planner.driving_profile.max_linear_acceleration = values.max_linear_acceleration_mps2;
  planner.driving_profile.max_angular_acceleration = values.max_angular_acceleration_radps2;
  planner.driving_profile.max_linear_jerk = values.max_linear_jerk_mps3;
  planner.driving_profile.max_angular_jerk = values.max_angular_jerk_radps3;
  planner.stopping_profile.max_linear_acceleration = values.stopping_linear_acceleration_mps2;
  planner.stopping_profile.max_angular_acceleration = values.stopping_angular_acceleration_radps2;
  planner.max_centripetal_acceleration = values.max_centripetal_acceleration_mps2;
  // The planner warns on every set_target() that hits the centripetal limit,
  // which is routine while turning at speed and would flood the log at the
  // joystick rate.
  planner.log_level = espp::Logger::Verbosity::ERROR;
  return planner;
}

} // namespace mib
