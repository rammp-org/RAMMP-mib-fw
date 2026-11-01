#pragma once

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
/// Plain data: the stroke <-> arm angle math is in SeatKinematics (linkage.hpp).
struct TriangleLinkage {
  float arm_pin_x_mm{0.0f};        ///< A, on the swing arm, relative to the pivot, at q = 0.
  float arm_pin_z_mm{0.0f};        ///< A, on the swing arm, relative to the pivot, at q = 0.
  float plate_pin_x_mm{0.0f};      ///< B, on the carriage plate, relative to the pivot.
  float plate_pin_z_mm{0.0f};      ///< B, on the carriage plate, relative to the pivot.
  float retracted_length_mm{0.0f}; ///< Pin-to-pin length when the actuator reads zero stroke.
};

/// @brief Everything SeatKinematics needs to know about the chair.
///
/// Plain data, so it lives in MIBConfig next to its values (mib::config::seat_geometry in
/// MIBconfig.hpp) and MIBConfig does not depend on SeatKinematics.
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

  /// Lowest a swing-arm pivot may be above flat ground. SeatKinematics::inverse() rejects a pose
  /// that puts any of the three pivots (the main ones moved by carriage travel) lower than this,
  /// which keeps the chassis and carriage plates off the floor.
  float min_pivot_clearance_mm{0.0f};

  // Joint-side quantity -> actuator degrees, one per actuator.
  Transmission main_left_transmission;      ///< Input: actuator stroke, mm.
  Transmission main_right_transmission;     ///< Input: actuator stroke, mm.
  Transmission rear_caster_transmission;    ///< Input: swing-arm angle, deg (encoder after shock).
  Transmission left_carriage_transmission;  ///< Input: carriage travel, mm.
  Transmission right_carriage_transmission; ///< Input: carriage travel, mm.
};

} // namespace mib
