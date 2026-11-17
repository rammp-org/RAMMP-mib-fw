#pragma once

#include <cstdint>
#include <optional>

#include "seat_geometry.hpp"
#include "seat_kinematics.hpp"

namespace mib {

/// @brief Calibrated travel of one leg actuator, in encoder counts (as Actuator::range()).
struct CountRange {
  int32_t min{0};
  int32_t max{0};
};

/// @brief Encoder counts of the three legs that set the seat's height and tilt.
struct SeatLegCounts {
  int32_t main_left{0};
  int32_t main_right{0};
  int32_t rear_caster{0};
};

/// @brief Seat pose <-> encoder counts of the main left, main right and rear caster legs.
///
/// The glue between SeatKinematics (geometry in millimetres and degrees) and the leg actuators
/// (encoder counts). Each leg's counts run linearly between the two ends of its calibrated
/// range, whose swing-arm angles SeatLegCalibration gives: linear in arm angle for an encoder
/// on the swing arm, linear in actuator stroke for an encoder on a main leg's linear actuator.
/// The swing-arm limits become the calibrated ends, so a pose that needs an arm past either
/// end is refused.
///
/// The carriages are not driven here: the caller passes the translation they are at.
///
/// Pure math, no hardware: safe to call from any task.
class SeatLegMap {
public:
  /// @return Empty if a range is not min < max, a leg's two end angles are equal, or an end
  ///         angle is not finite.
  static std::optional<SeatLegMap> create(const SeatGeometry &geometry,
                                          const SeatLegCalibration &calibration,
                                          CountRange main_left, CountRange main_right,
                                          CountRange rear_caster);

  /// @brief Seat pose -> leg counts.
  /// @return Empty if SeatKinematics::inverse() refuses the pose with the calibrated arm limits.
  std::optional<SeatLegCounts> counts_for(const SeatPose &pose) const;

  /// @brief Leg counts -> swing-arm angles (carriages at `translation_mm`).
  /// @return Empty if a main leg's stroke has no arm angle near its calibrated travel.
  std::optional<JointConfig> joints_at(const SeatLegCounts &counts, float translation_mm) const;

  /// @brief Leg counts -> seat pose (carriages at `translation_mm`), for reporting.
  std::optional<SeatPose> pose_at(const SeatLegCounts &counts, float translation_mm) const;

  /// @brief The kinematics, built with the calibrated arm limits.
  const SeatKinematics &kinematics() const { return kinematics_; }

private:
  /// One leg: its counts are linear in `u`, the arm angle or (through the linkage) the
  /// actuator stroke.
  struct Leg {
    CountRange range;
    float arm_deg_at_min{0.0f};
    float arm_deg_at_max{0.0f};
    std::optional<TriangleLinkage> linkage; ///< Set: counts follow stroke. Empty: arm angle.
    float u_at_min{0.0f};
    float u_at_max{0.0f};

    float u_for_arm(float arm_deg) const;
    int32_t counts_for_arm(float arm_deg) const;
    std::optional<float> arm_for_counts(int32_t counts) const;
  };

  SeatLegMap(const SeatGeometry &geometry, const Leg &main_left, const Leg &main_right,
             const Leg &rear_caster)
      : kinematics_(geometry), main_left_(main_left), main_right_(main_right),
        rear_caster_(rear_caster) {}

  SeatKinematics kinematics_;
  Leg main_left_;
  Leg main_right_;
  Leg rear_caster_;
};

} // namespace mib
