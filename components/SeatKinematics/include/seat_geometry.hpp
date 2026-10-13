#pragma once

#include "linkage.hpp"
#include "transmission.hpp"

namespace mib {

/// @brief A point or offset in millimetres.
///
/// Chassis frame: the rammp base frame (URDF root link). x forward (toward the front caster),
/// y left, z up.
struct PointMm {
  float x_mm{0.0f};
  float y_mm{0.0f};
  float z_mm{0.0f};
};

/// @brief One wheel leg: a wheel on the end of a swing arm.
///
/// Side view (x forward, z up). The arm turns about −y through its pivot, as in the URDF:
///
///     pivot ●─────────── arm ───────────◯ wheel centre
///                                        ‾‾‾‾‾‾‾‾ ground
struct SwingArmLeg {
  PointMm pivot_mm;          ///< Swing-arm pivot in the chassis frame (carriage at zero travel).
  PointMm pivot_to_wheel_mm; ///< Wheel centre relative to the pivot, at arm angle 0.
  float wheel_radius_mm{0.0f};
  float min_arm_deg{0.0f}; ///< Swing-arm travel limit.
  float max_arm_deg{0.0f}; ///< Swing-arm travel limit.
};

/// @brief Everything SeatKinematics needs to know about the chair.
///
/// Values live in MIBconfig.hpp (mib::config::seat_geometry).
struct SeatGeometry {
  // Legs. The main legs ride on the carriages, so their pivots move with carriage travel.
  SwingArmLeg main_left;
  SwingArmLeg main_right;
  SwingArmLeg rear_caster; ///< Its two wheels share one arm; modelled as one wheel at y = 0.

  // Main-leg linear actuators (actuator stroke <-> swing-arm angle).
  TriangleLinkage main_left_linkage;
  TriangleLinkage main_right_linkage;

  // Carriage travel limits, same for both sides.
  float min_carriage_mm{0.0f};
  float max_carriage_mm{0.0f};

  // Joint-side quantity -> actuator degrees, one per actuator.
  Transmission main_left_transmission;      ///< Input: actuator stroke, mm.
  Transmission main_right_transmission;     ///< Input: actuator stroke, mm.
  Transmission rear_caster_transmission;    ///< Input: swing-arm angle, deg (encoder after shock).
  Transmission left_carriage_transmission;  ///< Input: carriage travel, mm.
  Transmission right_carriage_transmission; ///< Input: carriage travel, mm.
};

} // namespace mib
