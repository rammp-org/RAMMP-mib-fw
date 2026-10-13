#pragma once

#include <optional>

#include "seat_geometry.hpp"

namespace mib {

/// @brief Requested or measured seat pose.
struct SeatPose {
  float pitch_deg{0.0f};      ///< Front/back tilt. Positive: front of the seat up (tilt back).
  float roll_deg{0.0f};       ///< Lateral tilt. Positive: left side up.
  float elevation_mm{0.0f};   ///< Height of the chassis-frame origin above flat ground.
  float translation_mm{0.0f}; ///< Fore/aft carriage travel from URDF zero, both sides.
};

/// @brief Joint positions: swing-arm angles and carriage travel.
struct JointConfig {
  float main_left_arm_deg{0.0f};
  float main_right_arm_deg{0.0f};
  float rear_caster_arm_deg{0.0f};
  float left_carriage_mm{0.0f};
  float right_carriage_mm{0.0f};
};

/// @brief Actuator positions, in each actuator's own degrees.
struct ActuatorPositions {
  float main_left_deg{0.0f};
  float main_right_deg{0.0f};
  float rear_caster_deg{0.0f};
  float left_carriage_deg{0.0f};
  float right_carriage_deg{0.0f};
};

/// @brief Seat kinematics: seat pose <-> joint positions <-> actuator degrees.
///
///     SeatPose ──inverse()──> JointConfig ──to_actuators()──> ActuatorPositions
///     SeatPose <──forward()── JointConfig <──from_actuators()── ActuatorPositions
///
/// The seat is moved by the wheel legs (main left, main right, rear caster) and the fore/aft
/// carriages. The front caster is not part of this. Flat ground is assumed.
///
/// Pure math: no hardware, no state besides the geometry, safe to call from any task.
/// Nothing is clamped. An empty result means the request is unreachable or outside a limit, and
/// the caller decides what to do.
class SeatKinematics {
public:
  explicit SeatKinematics(const SeatGeometry &geometry)
      : geometry_(geometry) {}

  /// @brief Seat pose -> joint positions.
  /// @return Empty if any leg cannot reach the ground within its arm limits, or the translation
  ///         is outside the carriage limits.
  std::optional<JointConfig> inverse(const SeatPose &pose) const;

  /// @brief Joint positions -> seat pose (for reporting seat state from feedback).
  /// @return Empty if no ground plane touches all three wheels (degenerate geometry).
  std::optional<SeatPose> forward(const JointConfig &joints) const;

  /// @brief Joint positions -> actuator degrees.
  /// @return Empty if any joint is outside its limits.
  std::optional<ActuatorPositions> to_actuators(const JointConfig &joints) const;

  /// @brief Actuator degrees -> joint positions.
  /// @return Empty if a main-leg stroke has no swing-arm angle within limits, or any joint is
  ///         outside its limits.
  std::optional<JointConfig> from_actuators(const ActuatorPositions &actuators) const;

private:
  bool joints_within_limits(const JointConfig &joints) const;

  SeatGeometry geometry_;
};

} // namespace mib
