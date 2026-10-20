#pragma once

// Maps a MIBconfig drive profile onto the trajectory planner. Header-only and
// free of ESP-IDF APIs beyond the planner, so tools/drive_fit can replay it on
// a host against the espp cross-platform library.

#include <algorithm>
#include <cmath>

#include "MIBconfig.hpp"
#include "trajectory_planner.hpp"

namespace mib {

/// @brief Scale reverse joystick input to the profile's reverse speed.
/// @param linear Normalized linear command; +1 is full forward.
/// @param reverse_scale Reverse speed as a fraction of forward, in (0, 1].
/// @return The command with negative values scaled. NaN passes through for the
///         planner to reject.
/// TODO(esp-cpp/espp#852): replace with the planner's reverse velocity limit
///      once it exists.
inline float scale_reverse_input(float linear, float reverse_scale) {
  if (std::isnan(linear)) {
    return linear;
  }
  linear = std::clamp(linear, -1.0f, 1.0f);
  return linear < 0.0f ? linear * reverse_scale : linear;
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
  // The planner warns on every set_target() that hits the centripetal limit,
  // which is routine while turning at speed and would flood the log at the
  // joystick rate.
  planner.log_level = espp::Logger::Verbosity::ERROR;
  return planner;
}

} // namespace mib
