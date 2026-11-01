#pragma once

#include <cmath>
#include <numbers>
#include <optional>

#include "seat_geometry.hpp"
#include "trig_equation.hpp"

namespace mib {

// Stroke <-> swing-arm angle for a TriangleLinkage (seat_geometry.hpp has the sketch).
//
// As the arm turns by q, pin A moves to rotate(A, q) and the actuator length becomes
//
//     L(q) = | B − rotate(A, q) |
//
// Arm rotation follows the URDF: about −y, so (x, z) -> (x·cos q − z·sin q, x·sin q + z·cos q).

/// @brief Pin-to-pin actuator length for a swing-arm angle.
inline float linkage_length_mm(const TriangleLinkage &linkage, float arm_deg) {
  const float q = arm_deg * std::numbers::pi_v<float> / 180.0f;
  const float moved_arm_pin_x =
      linkage.arm_pin_x_mm * std::cos(q) - linkage.arm_pin_z_mm * std::sin(q);
  const float moved_arm_pin_z =
      linkage.arm_pin_x_mm * std::sin(q) + linkage.arm_pin_z_mm * std::cos(q);
  return std::hypot(linkage.plate_pin_x_mm - moved_arm_pin_x,
                    linkage.plate_pin_z_mm - moved_arm_pin_z);
}

/// @brief Actuator stroke (extension past fully retracted) for a swing-arm angle.
inline float linkage_stroke_mm(const TriangleLinkage &linkage, float arm_deg) {
  return linkage_length_mm(linkage, arm_deg) - linkage.retracted_length_mm;
}

/// @brief Swing-arm angle for an actuator stroke (the inverse of linkage_stroke_mm()).
///
/// Expanding  L² = |B − rotate(A, q)|²  gives  |A|² + |B|² − 2·B·rotate(A, q) = L², i.e.
///
///     (B·A)·cos q + (Bz·Ax − Bx·Az)·sin q = (|A|² + |B|² − L²) / 2
///
/// which is the a·cos q + b·sin q = c form.
///
/// @param extension_mm Actuator stroke: extension past fully retracted.
/// @param min_arm_deg, max_arm_deg Swing-arm travel; picks the root inside it.
/// @return Swing-arm angle in degrees, or empty if no angle in range gives this stroke.
inline std::optional<float> linkage_arm_deg(const TriangleLinkage &linkage, float extension_mm,
                                            float min_arm_deg, float max_arm_deg) {
  const float ax = linkage.arm_pin_x_mm;
  const float az = linkage.arm_pin_z_mm;
  const float bx = linkage.plate_pin_x_mm;
  const float bz = linkage.plate_pin_z_mm;
  const float length = linkage.retracted_length_mm + extension_mm;
  const float arm_pin_sq = ax * ax + az * az;
  const float plate_pin_sq = bx * bx + bz * bz;

  const float a = bx * ax + bz * az;
  const float b = bz * ax - bx * az;
  const float c = (arm_pin_sq + plate_pin_sq - length * length) / 2.0f;

  constexpr float deg_to_rad = std::numbers::pi_v<float> / 180.0f;
  auto q = solve_cos_sin_equation(a, b, c, min_arm_deg * deg_to_rad, max_arm_deg * deg_to_rad);
  if (!q) {
    return std::nullopt;
  }
  return *q / deg_to_rad;
}

} // namespace mib
