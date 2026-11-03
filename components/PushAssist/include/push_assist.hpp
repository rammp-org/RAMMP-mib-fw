#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

/// Push assist: an attendant drives the chair from the two push handles. The push-assist
/// board measures each handle's force (rammp-org/space-bar-push-assist); the MIB turns the
/// forces into a drive target. This header is that conversion and its tuning, with no
/// ESP-IDF in it, so it builds on a host (test/run.sh).
///
/// The conversion is the reference in space-bar-push-assist docs/MIB-INPUT-DEVICES.md
/// ("The conversion"), which the board's dashboard (pushToTwist()) and the MIB control panel
/// also run, so all three show the same answer for the same forces.
namespace mib::push_assist {

/// Tuning for push assist: the force-to-drive mapping and the planner limits while pushing.
/// Set from the control panel (Push assist tab) and kept in NVS (Settings).
struct Gains {
  float dead_band_n;              ///< Per handle: a handle under this reads as 0. N.
  float forward_full_n;           ///< Forward force past the dead bands for linear = 1. N.
  float turn_full_n;              ///< left - right past the dead bands for |angular| = 1. N.
  float reverse_max;              ///< Cap on backing up when the handles are pulled, 0..1.
  float max_linear_mps;           ///< Planner: linear speed at linear = 1. m/s.
  float max_angular_radps;        ///< Planner: angular speed at |angular| = 1. rad/s.
  float max_linear_accel_mps2;    ///< Planner: linear acceleration. m/s^2.
  float max_angular_accel_radps2; ///< Planner: angular acceleration. rad/s^2.
};

/// Starting values (MIB-INPUT-DEVICES.md): a firm two-handed push reaches full speed, and
/// the planner limits are the LOW drive profile's.
inline constexpr Gains kDefaultGains{
    .dead_band_n = 10.0f,
    .forward_full_n = 60.0f,
    .turn_full_n = 30.0f,
    .reverse_max = 0.3f,
    .max_linear_mps = 0.35f,
    .max_angular_radps = 1.0f,
    .max_linear_accel_mps2 = 0.5f,
    .max_angular_accel_radps2 = 1.0f,
};

/// One tunable field of Gains, as the control panel lists it.
struct ParamSpec {
  std::string_view key;   ///< Wire and NVS key (NVS keys are at most 15 characters).
  std::string_view label; ///< For the panel.
  std::string_view unit;
  std::string_view group;
  float min;
  float max;
  float step;
  float Gains::*field;
};

/// Every tunable field. The ceilings are the HIGH drive profile's limits: push assist never
/// goes faster than the joystick can.
inline constexpr std::array<ParamSpec, 8> kParams{{
    {"dead_band_n", "Dead band per handle", "N", "Force to drive", 0.0f, 50.0f, 0.5f,
     &Gains::dead_band_n},
    {"forward_full_n", "Forward force for full speed", "N", "Force to drive", 10.0f, 300.0f, 5.0f,
     &Gains::forward_full_n},
    {"turn_full_n", "Turn force for full turn", "N", "Force to drive", 5.0f, 200.0f, 5.0f,
     &Gains::turn_full_n},
    {"reverse_max", "Reverse cap (pull)", "", "Force to drive", 0.0f, 1.0f, 0.05f,
     &Gains::reverse_max},
    {"max_lin_mps", "Max linear speed", "m/s", "Planner limits", 0.05f, 1.0f, 0.05f,
     &Gains::max_linear_mps},
    {"max_ang_rps", "Max angular speed", "rad/s", "Planner limits", 0.1f, 3.14159f, 0.05f,
     &Gains::max_angular_radps},
    {"lin_acc_mps2", "Linear acceleration", "m/s²", "Planner limits", 0.1f, 2.0f, 0.05f,
     &Gains::max_linear_accel_mps2},
    {"ang_acc_rps2", "Angular acceleration", "rad/s²", "Planner limits", 0.1f, 4.0f, 0.05f,
     &Gains::max_angular_accel_radps2},
}};

/// \return The spec for \p key, or nullptr.
inline constexpr const ParamSpec *find_param(std::string_view key) {
  for (const auto &p : kParams) {
    if (p.key == key) {
      return &p;
    }
  }
  return nullptr;
}

/// \return \p value clamped to \p spec's range; NaN becomes the minimum.
inline float clamp_param(const ParamSpec &spec, float value) {
  if (std::isnan(value)) {
    return spec.min;
  }
  return std::clamp(value, spec.min, spec.max);
}

/// A normalized drive target for DriveController::set_target().
struct Twist {
  float linear;  ///< [-1, 1], forward +.
  float angular; ///< [-1, 1], counter-clockwise + (REP 103).
};

/// Handle forces to a drive target.
/// \param left_n, right_n HandleForce fields: N, push +.
/// \return {linear, angular} in [-1, 1]. Left harder turns clockwise, so angular < 0.
///         Over full scale both are scaled down together, so the turn radius holds (as
///         DriveController::set_velocity() does). NaN in either force returns {0, 0}: the
///         caller should stop on it anyway.
inline Twist push_to_twist(float left_n, float right_n, const Gains &g) {
  if (std::isnan(left_n) || std::isnan(right_n)) {
    return {0.0f, 0.0f};
  }
  const auto band = [&](float f) { // remove the dead band, keep the sign
    return std::fabs(f) <= g.dead_band_n ? 0.0f : f - std::copysign(g.dead_band_n, f);
  };
  const float l = band(left_n);
  const float r = band(right_n);
  float linear = (l + r) / g.forward_full_n;
  float angular = -(l - r) / g.turn_full_n;
  linear = std::max(linear, -g.reverse_max);
  const float scale = std::max({1.0f, std::fabs(linear), std::fabs(angular)});
  return {linear / scale, angular / scale};
}

} // namespace mib::push_assist
