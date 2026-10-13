#pragma once

namespace mib {

/// @brief Converts a joint-side quantity into the actuator's own degrees.
///
/// Every seat actuator is described by one straight-line (affine) map:
///
///     actuator_deg = deg_per_unit * input + offset_deg
///
/// "Actuator degrees" are degrees of the shaft the actuator class measures. The actuator class
/// owns degrees -> encoder counts; this struct owns everything between the joint and that shaft
/// (gear ratio, screw lead, direction, zero offset).
///
/// Build one with linear() or rotary() rather than filling the fields by hand.
struct Transmission {
  float deg_per_unit{1.0f}; ///< Actuator degrees per input unit (mm or deg). Sign = direction.
  float offset_deg{0.0f};   ///< Actuator degrees when the input is zero.

  /// @brief Actuator that moves something in a straight line (input in millimetres).
  /// @param travel_per_rev_mm Millimetres of travel per revolution of the output (screw lead,
  ///        or belt travel per pulley turn).
  /// @param gear_ratio Revolutions of the measured shaft per revolution of the output.
  /// @param inverted True if actuator degrees decrease as the input increases.
  /// @param offset_deg Actuator degrees when the input is zero.
  static constexpr Transmission linear(float travel_per_rev_mm, float gear_ratio,
                                       bool inverted = false, float offset_deg = 0.0f) {
    const float direction = inverted ? -1.0f : 1.0f;
    return {.deg_per_unit = direction * 360.0f * gear_ratio / travel_per_rev_mm,
            .offset_deg = offset_deg};
  }

  /// @brief Actuator that turns a joint (input in degrees).
  /// @param gear_ratio Revolutions of the measured shaft per revolution of the joint.
  /// @param inverted True if actuator degrees decrease as the joint angle increases.
  /// @param offset_deg Actuator degrees when the joint angle is zero.
  static constexpr Transmission rotary(float gear_ratio, bool inverted = false,
                                       float offset_deg = 0.0f) {
    const float direction = inverted ? -1.0f : 1.0f;
    return {.deg_per_unit = direction * gear_ratio, .offset_deg = offset_deg};
  }

  /// @brief Joint-side quantity (mm or deg) -> actuator degrees.
  constexpr float to_actuator_deg(float input) const { return deg_per_unit * input + offset_deg; }

  /// @brief Actuator degrees -> joint-side quantity (mm or deg).
  constexpr float from_actuator_deg(float actuator_deg) const {
    return (actuator_deg - offset_deg) / deg_per_unit;
  }
};

} // namespace mib
