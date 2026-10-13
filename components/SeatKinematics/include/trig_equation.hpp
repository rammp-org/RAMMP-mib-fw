#pragma once

#include <cmath>
#include <numbers>
#include <optional>

namespace mib {

/// Rounding tolerance at a hard stop, see within_limits(). Far smaller than any real resolution.
inline constexpr float limit_tolerance_deg = 1e-3f;
inline constexpr float limit_tolerance_mm = 1e-3f;

/// @brief Check a value against [min, max], absorbing float rounding at the limits.
///
/// A joint resting on its hard stop converts through actuator degrees and back with a little float
/// rounding, so it can come back a hair past the limit. A value within `tolerance` outside the
/// range is snapped onto the limit. This is not clamping: anything further out is rejected.
///
/// @return The value (snapped onto the limit if it was just outside), or empty if it is out of
///         range or NaN.
inline std::optional<float> within_limits(float value, float min, float max, float tolerance) {
  // Written as "in range" rather than "out of range" so that NaN is rejected too.
  if (value >= min && value <= max) {
    return value;
  }
  if (value >= min - tolerance && value < min) {
    return min;
  }
  if (value > max && value <= max + tolerance) {
    return max;
  }
  return std::nullopt;
}

/// @brief Solve  a·cos(q) + b·sin(q) = c  for q, and keep the root inside [min_rad, max_rad].
///
/// Both the leg IK and the actuator linkage reduce to this one equation. Rewriting the left side
/// as  R·cos(q − φ)  with  R = √(a² + b²)  and  φ = atan2(b, a)  gives the two roots
///
///     q = φ ± acos(c / R)
///
/// @return The root inside the range. If both roots are inside, the one closer to zero (the CAD
///         pose). Empty if there is no real root (|c| > R) or neither root is in range.
inline std::optional<float> solve_cos_sin_equation(float a, float b, float c, float min_rad,
                                                   float max_rad) {
  const float amplitude = std::hypot(a, b);
  if (amplitude <= 0.0f || std::fabs(c) > amplitude) {
    return std::nullopt;
  }
  const float phase = std::atan2(b, a);
  const float spread = std::acos(c / amplitude);

  // Bring each root into (−π, π] so it can be compared with the range.
  auto wrap = [](float angle) { return std::remainder(angle, 2.0f * std::numbers::pi_v<float>); };
  const float root_plus = wrap(phase + spread);
  const float root_minus = wrap(phase - spread);

  // A root just past a limit is snapped onto it (float rounding), see within_limits().
  constexpr float tolerance_rad = limit_tolerance_deg * std::numbers::pi_v<float> / 180.0f;
  const auto plus = within_limits(root_plus, min_rad, max_rad, tolerance_rad);
  const auto minus = within_limits(root_minus, min_rad, max_rad, tolerance_rad);
  if (plus && minus) {
    return std::fabs(*plus) <= std::fabs(*minus) ? plus : minus;
  }
  return plus ? plus : minus;
}

} // namespace mib
