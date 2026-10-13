#pragma once

#include <cmath>
#include <numbers>
#include <optional>

namespace mib {

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

  auto in_range = [&](float q) { return q >= min_rad && q <= max_rad; };
  const bool plus_ok = in_range(root_plus);
  const bool minus_ok = in_range(root_minus);
  if (plus_ok && minus_ok) {
    return std::fabs(root_plus) <= std::fabs(root_minus) ? root_plus : root_minus;
  }
  if (plus_ok) {
    return root_plus;
  }
  if (minus_ok) {
    return root_minus;
  }
  return std::nullopt;
}

} // namespace mib
