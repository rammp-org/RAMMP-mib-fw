#include "seat_kinematics.hpp"

#include <cmath>
#include <numbers>

#include "trig_equation.hpp"

namespace mib {

namespace {

constexpr float deg_to_rad = std::numbers::pi_v<float> / 180.0f;
constexpr float rad_to_deg = 180.0f / std::numbers::pi_v<float>;

float dot(const PointMm &a, const PointMm &b) {
  return a.x_mm * b.x_mm + a.y_mm * b.y_mm + a.z_mm * b.z_mm;
}

PointMm minus(const PointMm &a, const PointMm &b) {
  return {a.x_mm - b.x_mm, a.y_mm - b.y_mm, a.z_mm - b.z_mm};
}

/// "World up" expressed in the chassis frame, as a unit vector (stored in a PointMm, unitless).
///
/// The height above ground of any chassis point P is  elevation + up · P,  with
///
///     up = ( sin p,  sin r · cos p,  cos r · cos p )
///
/// Check: a point forward (+x) rises when p > 0 (front up); a point left (+y) rises when r > 0
/// (left up).
PointMm world_up(float pitch_deg, float roll_deg) {
  const float p = pitch_deg * deg_to_rad;
  const float r = roll_deg * deg_to_rad;
  return {std::sin(p), std::sin(r) * std::cos(p), std::cos(r) * std::cos(p)};
}

/// Wheel centre in the chassis frame for a given arm angle.
///
///     wheel = pivot + (carriage, 0, 0) + rotate(pivot_to_wheel, q)
///
/// with rotation about −y: (x, z) -> (x·cos q − z·sin q, x·sin q + z·cos q).
PointMm wheel_centre(const SwingArmLeg &leg, float carriage_mm, float arm_deg) {
  const float q = arm_deg * deg_to_rad;
  const PointMm &w = leg.pivot_to_wheel_mm;
  return {leg.pivot_mm.x_mm + carriage_mm + w.x_mm * std::cos(q) - w.z_mm * std::sin(q),
          leg.pivot_mm.y_mm + w.y_mm,
          leg.pivot_mm.z_mm + w.x_mm * std::sin(q) + w.z_mm * std::cos(q)};
}

/// The part of the wheel centre that does not depend on the arm angle:
/// pivot + (carriage, 0, 0) + (0, pivot_to_wheel.y, 0).
PointMm leg_base_point(const SwingArmLeg &leg, float carriage_mm) {
  return {leg.pivot_mm.x_mm + carriage_mm, leg.pivot_mm.y_mm + leg.pivot_to_wheel_mm.y_mm,
          leg.pivot_mm.z_mm};
}

/// Arm angle that puts this leg's wheel on the ground.
///
/// The wheel touches the ground when its centre is one radius above it:
///
///     elevation + up · wheel(q) = radius
///
/// Writing wheel(q) out (see wheel_centre) and collecting the cos q and sin q terms:
///
///     a = up.x · w.x + up.z · w.z
///     b = up.z · w.x − up.x · w.z
///     c = radius − elevation − up · (pivot + (carriage, 0, 0) + (0, w.y, 0))
///
///     a · cos q + b · sin q = c
std::optional<float> arm_deg_on_ground(const SwingArmLeg &leg, float carriage_mm, const PointMm &up,
                                       float elevation_mm) {
  const PointMm &w = leg.pivot_to_wheel_mm;
  const float a = up.x_mm * w.x_mm + up.z_mm * w.z_mm;
  const float b = up.z_mm * w.x_mm - up.x_mm * w.z_mm;
  const float c = leg.wheel_radius_mm - elevation_mm - dot(up, leg_base_point(leg, carriage_mm));

  auto q =
      solve_cos_sin_equation(a, b, c, leg.min_arm_deg * deg_to_rad, leg.max_arm_deg * deg_to_rad);
  if (!q) {
    return std::nullopt;
  }
  return *q * rad_to_deg;
}

} // namespace

std::optional<JointConfig> SeatKinematics::inverse(const SeatPose &pose) const {
  if (pose.translation_mm < geometry_.min_carriage_mm ||
      pose.translation_mm > geometry_.max_carriage_mm) {
    return std::nullopt;
  }
  const PointMm up = world_up(pose.pitch_deg, pose.roll_deg);
  const float carriage_mm = pose.translation_mm;

  auto main_left = arm_deg_on_ground(geometry_.main_left, carriage_mm, up, pose.elevation_mm);
  auto main_right = arm_deg_on_ground(geometry_.main_right, carriage_mm, up, pose.elevation_mm);
  // The rear caster is mounted on the chassis, not a carriage: it does not move with translation.
  auto rear_caster = arm_deg_on_ground(geometry_.rear_caster, 0.0f, up, pose.elevation_mm);
  if (!main_left || !main_right || !rear_caster) {
    return std::nullopt;
  }

  return JointConfig{
      .main_left_arm_deg = *main_left,
      .main_right_arm_deg = *main_right,
      .rear_caster_arm_deg = *rear_caster,
      .left_carriage_mm = carriage_mm,
      .right_carriage_mm = carriage_mm,
  };
}

std::optional<SeatPose> SeatKinematics::forward(const JointConfig &joints) const {
  // Wheel centres in the chassis frame, and the radius of each wheel.
  const PointMm left =
      wheel_centre(geometry_.main_left, joints.left_carriage_mm, joints.main_left_arm_deg);
  const PointMm right =
      wheel_centre(geometry_.main_right, joints.right_carriage_mm, joints.main_right_arm_deg);
  const PointMm rear = wheel_centre(geometry_.rear_caster, 0.0f, joints.rear_caster_arm_deg);
  const float left_radius = geometry_.main_left.wheel_radius_mm;
  const float right_radius = geometry_.main_right.wheel_radius_mm;
  const float rear_radius = geometry_.rear_caster.wheel_radius_mm;

  // Every wheel rests on the ground:  elevation + up · wheel = radius.
  // Subtracting the right and rear equations from the left one removes the elevation:
  //
  //     up · d1 = e1     with  d1 = left − right,  e1 = left_radius − right_radius
  //     up · d2 = e2     with  d2 = left − rear,   e2 = left_radius − rear_radius
  //
  // Two planes meet in a line; `up` is where that line crosses the unit sphere.
  // The line is  closest + t · normal,  where `closest` lies in the span of d1 and d2 and
  // `normal` = d1 × d2 is perpendicular to both.
  const PointMm d1 = minus(left, right);
  const PointMm d2 = minus(left, rear);
  const float e1 = left_radius - right_radius;
  const float e2 = left_radius - rear_radius;

  // closest = α·d1 + β·d2, from the 2x2 system  [d1·d1 d1·d2; d1·d2 d2·d2] [α; β] = [e1; e2].
  const float g11 = dot(d1, d1);
  const float g12 = dot(d1, d2);
  const float g22 = dot(d2, d2);
  const float determinant = g11 * g22 - g12 * g12; // also |d1 × d2|²
  if (determinant <= 0.0f) {
    return std::nullopt; // wheels in a line: no unique ground plane
  }
  const float alpha = (e1 * g22 - e2 * g12) / determinant;
  const float beta = (e2 * g11 - e1 * g12) / determinant;
  const PointMm closest = {alpha * d1.x_mm + beta * d2.x_mm, alpha * d1.y_mm + beta * d2.y_mm,
                           alpha * d1.z_mm + beta * d2.z_mm};

  const float closest_sq = dot(closest, closest);
  if (closest_sq > 1.0f) {
    return std::nullopt; // line misses the unit sphere
  }
  const PointMm normal = {d1.y_mm * d2.z_mm - d1.z_mm * d2.y_mm,
                          d1.z_mm * d2.x_mm - d1.x_mm * d2.z_mm,
                          d1.x_mm * d2.y_mm - d1.y_mm * d2.x_mm};
  const float t = std::sqrt((1.0f - closest_sq) / determinant);

  // Two candidates; the chair is upright, so take the one pointing most nearly up.
  const PointMm up_a = {closest.x_mm + t * normal.x_mm, closest.y_mm + t * normal.y_mm,
                        closest.z_mm + t * normal.z_mm};
  const PointMm up_b = {closest.x_mm - t * normal.x_mm, closest.y_mm - t * normal.y_mm,
                        closest.z_mm - t * normal.z_mm};
  const PointMm up = up_a.z_mm >= up_b.z_mm ? up_a : up_b;
  if (up.z_mm <= 0.0f) {
    return std::nullopt;
  }

  // Undo world_up():  up = (sin p, sin r·cos p, cos r·cos p).
  return SeatPose{
      .pitch_deg = std::asin(up.x_mm) * rad_to_deg,
      .roll_deg = std::atan2(up.y_mm, up.z_mm) * rad_to_deg,
      .elevation_mm = left_radius - dot(up, left),
      .translation_mm = (joints.left_carriage_mm + joints.right_carriage_mm) / 2.0f,
  };
}

std::optional<ActuatorPositions> SeatKinematics::to_actuators(const JointConfig &joints) const {
  if (!joints_within_limits(joints)) {
    return std::nullopt;
  }
  const float main_left_stroke_mm = geometry_.main_left_linkage.stroke_mm(joints.main_left_arm_deg);
  const float main_right_stroke_mm =
      geometry_.main_right_linkage.stroke_mm(joints.main_right_arm_deg);

  return ActuatorPositions{
      .main_left_deg = geometry_.main_left_transmission.to_actuator_deg(main_left_stroke_mm),
      .main_right_deg = geometry_.main_right_transmission.to_actuator_deg(main_right_stroke_mm),
      .rear_caster_deg =
          geometry_.rear_caster_transmission.to_actuator_deg(joints.rear_caster_arm_deg),
      .left_carriage_deg =
          geometry_.left_carriage_transmission.to_actuator_deg(joints.left_carriage_mm),
      .right_carriage_deg =
          geometry_.right_carriage_transmission.to_actuator_deg(joints.right_carriage_mm),
  };
}

std::optional<JointConfig>
SeatKinematics::from_actuators(const ActuatorPositions &actuators) const {
  const float main_left_stroke_mm =
      geometry_.main_left_transmission.from_actuator_deg(actuators.main_left_deg);
  const float main_right_stroke_mm =
      geometry_.main_right_transmission.from_actuator_deg(actuators.main_right_deg);

  auto main_left_arm_deg = geometry_.main_left_linkage.arm_deg(
      main_left_stroke_mm, geometry_.main_left.min_arm_deg, geometry_.main_left.max_arm_deg);
  auto main_right_arm_deg = geometry_.main_right_linkage.arm_deg(
      main_right_stroke_mm, geometry_.main_right.min_arm_deg, geometry_.main_right.max_arm_deg);
  if (!main_left_arm_deg || !main_right_arm_deg) {
    return std::nullopt;
  }

  const JointConfig joints{
      .main_left_arm_deg = *main_left_arm_deg,
      .main_right_arm_deg = *main_right_arm_deg,
      .rear_caster_arm_deg =
          geometry_.rear_caster_transmission.from_actuator_deg(actuators.rear_caster_deg),
      .left_carriage_mm =
          geometry_.left_carriage_transmission.from_actuator_deg(actuators.left_carriage_deg),
      .right_carriage_mm =
          geometry_.right_carriage_transmission.from_actuator_deg(actuators.right_carriage_deg),
  };
  if (!joints_within_limits(joints)) {
    return std::nullopt;
  }
  return joints;
}

bool SeatKinematics::joints_within_limits(const JointConfig &joints) const {
  auto arm_ok = [](const SwingArmLeg &leg, float arm_deg) {
    return arm_deg >= leg.min_arm_deg && arm_deg <= leg.max_arm_deg;
  };
  auto carriage_ok = [this](float carriage_mm) {
    return carriage_mm >= geometry_.min_carriage_mm && carriage_mm <= geometry_.max_carriage_mm;
  };
  return arm_ok(geometry_.main_left, joints.main_left_arm_deg) &&
         arm_ok(geometry_.main_right, joints.main_right_arm_deg) &&
         arm_ok(geometry_.rear_caster, joints.rear_caster_arm_deg) &&
         carriage_ok(joints.left_carriage_mm) && carriage_ok(joints.right_carriage_mm);
}

} // namespace mib
