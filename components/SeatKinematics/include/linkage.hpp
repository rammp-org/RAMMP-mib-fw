#pragma once

#include <cmath>
#include <numbers>
#include <optional>

#include "trig_equation.hpp"

namespace mib {

/// @brief A linear actuator pinned between a swing arm and a fixed plate (main legs).
///
/// Side view, in the swing arm's plane (x forward, z up), origin at the arm's pivot O:
///
///                      B  pin on the carriage plate (does not move)
///                     /
///       actuator, length L
///                   /
///        A  pin on the swing arm (rotates with the arm about O)
///         \  swing arm, A to O
///          O ─────────────────── wheel
///
/// As the arm turns by q, A moves to rotate(A, q) and the actuator length becomes
///
///     L(q) = | B − rotate(A, q) |
///
/// Arm rotation follows the URDF: about −y, so (x, z) -> (x·cos q − z·sin q, x·sin q + z·cos q).
struct TriangleLinkage {
  float arm_pin_x_mm{0.0f};        ///< A, on the swing arm, relative to the pivot, at q = 0.
  float arm_pin_z_mm{0.0f};        ///< A, on the swing arm, relative to the pivot, at q = 0.
  float plate_pin_x_mm{0.0f};      ///< B, on the carriage plate, relative to the pivot.
  float plate_pin_z_mm{0.0f};      ///< B, on the carriage plate, relative to the pivot.
  float retracted_length_mm{0.0f}; ///< Pin-to-pin length when the actuator reads zero stroke.

  /// @brief Pin-to-pin actuator length for a swing-arm angle.
  float length_mm(float arm_deg) const {
    const float q = arm_deg * std::numbers::pi_v<float> / 180.0f;
    const float moved_arm_pin_x = arm_pin_x_mm * std::cos(q) - arm_pin_z_mm * std::sin(q);
    const float moved_arm_pin_z = arm_pin_x_mm * std::sin(q) + arm_pin_z_mm * std::cos(q);
    return std::hypot(plate_pin_x_mm - moved_arm_pin_x, plate_pin_z_mm - moved_arm_pin_z);
  }

  /// @brief Actuator stroke (extension past fully retracted) for a swing-arm angle.
  float stroke_mm(float arm_deg) const { return length_mm(arm_deg) - retracted_length_mm; }

  /// @brief Swing-arm angle for an actuator stroke (the inverse of stroke_mm()).
  ///
  /// Expanding  L² = |B − rotate(A, q)|²  gives  |A|² + |B|² − 2·B·rotate(A, q) = L², i.e.
  ///
  ///     (B·A)·cos q + (Bz·Ax − Bx·Az)·sin q = (|A|² + |B|² − L²) / 2
  ///
  /// which is the a·cos q + b·sin q = c form.
  ///
  /// @param stroke_mm Actuator stroke.
  /// @param min_arm_deg, max_arm_deg Swing-arm travel; picks the root inside it.
  /// @return Swing-arm angle in degrees, or empty if no angle in range gives this stroke.
  std::optional<float> arm_deg(float stroke_mm, float min_arm_deg, float max_arm_deg) const {
    const float length = retracted_length_mm + stroke_mm;
    const float arm_pin_sq = arm_pin_x_mm * arm_pin_x_mm + arm_pin_z_mm * arm_pin_z_mm;
    const float plate_pin_sq = plate_pin_x_mm * plate_pin_x_mm + plate_pin_z_mm * plate_pin_z_mm;

    const float a = plate_pin_x_mm * arm_pin_x_mm + plate_pin_z_mm * arm_pin_z_mm;
    const float b = plate_pin_z_mm * arm_pin_x_mm - plate_pin_x_mm * arm_pin_z_mm;
    const float c = (arm_pin_sq + plate_pin_sq - length * length) / 2.0f;

    constexpr float deg_to_rad = std::numbers::pi_v<float> / 180.0f;
    auto q = solve_cos_sin_equation(a, b, c, min_arm_deg * deg_to_rad, max_arm_deg * deg_to_rad);
    if (!q) {
      return std::nullopt;
    }
    return *q / deg_to_rad;
  }
};

} // namespace mib
