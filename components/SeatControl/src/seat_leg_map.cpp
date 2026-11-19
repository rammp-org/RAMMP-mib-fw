#include "seat_leg_map.hpp"

#include <algorithm>
#include <cmath>

#include "linkage.hpp"

namespace mib {

namespace {

/// How far past its calibrated ends joints_at() looks for a main leg's arm angle, so a leg that
/// overshot an end a little still reports a pose.
constexpr float arm_search_margin_deg = 10.0f;

} // namespace

float SeatLegMap::Leg::u_for_arm(float arm_deg) const {
  return linkage ? linkage_stroke_mm(*linkage, arm_deg) : arm_deg;
}

int32_t SeatLegMap::Leg::counts_for_arm(float arm_deg) const {
  const float t = std::clamp((u_for_arm(arm_deg) - u_at_min) / (u_at_max - u_at_min), 0.0f, 1.0f);
  const double counts =
      static_cast<double>(range.min) + t * (static_cast<double>(range.max) - range.min);
  return static_cast<int32_t>(std::lround(counts));
}

std::optional<float> SeatLegMap::Leg::arm_for_counts(int32_t counts) const {
  const float t = static_cast<float>((static_cast<double>(counts) - range.min) /
                                     (static_cast<double>(range.max) - range.min));
  const float u = u_at_min + t * (u_at_max - u_at_min);
  if (!linkage) {
    return u;
  }
  const float low = std::min(arm_deg_at_min, arm_deg_at_max) - arm_search_margin_deg;
  const float high = std::max(arm_deg_at_min, arm_deg_at_max) + arm_search_margin_deg;
  return linkage_arm_deg(*linkage, u, low, high);
}

std::optional<SeatLegMap> SeatLegMap::create(const SeatGeometry &geometry,
                                             const SeatLegCalibration &calibration,
                                             CountRange main_left, CountRange main_right,
                                             CountRange rear_caster) {
  auto make_leg = [](CountRange range, const LegEndAngles &ends,
                     std::optional<TriangleLinkage> linkage) -> std::optional<Leg> {
    if (range.min >= range.max || !std::isfinite(ends.arm_deg_at_min_counts) ||
        !std::isfinite(ends.arm_deg_at_max_counts) ||
        ends.arm_deg_at_min_counts == ends.arm_deg_at_max_counts) {
      return std::nullopt;
    }
    Leg leg{.range = range,
            .arm_deg_at_min = ends.arm_deg_at_min_counts,
            .arm_deg_at_max = ends.arm_deg_at_max_counts,
            .linkage = linkage};
    leg.u_at_min = leg.u_for_arm(leg.arm_deg_at_min);
    leg.u_at_max = leg.u_for_arm(leg.arm_deg_at_max);
    if (!std::isfinite(leg.u_at_min) || !std::isfinite(leg.u_at_max) ||
        leg.u_at_min == leg.u_at_max) {
      return std::nullopt;
    }
    return leg;
  };

  const bool by_stroke = calibration.main_leg_encoder_on_actuator;
  auto left = make_leg(main_left, calibration.main_left,
                       by_stroke ? std::optional(geometry.main_left_linkage) : std::nullopt);
  auto right = make_leg(main_right, calibration.main_right,
                        by_stroke ? std::optional(geometry.main_right_linkage) : std::nullopt);
  auto rear = make_leg(rear_caster, calibration.rear_caster, std::nullopt);
  if (!left || !right || !rear) {
    return std::nullopt;
  }

  // The calibrated ends are the arms' travel.
  SeatGeometry calibrated = geometry;
  auto set_limits = [](SwingArmLeg &arm, const Leg &leg) {
    arm.min_arm_deg = std::min(leg.arm_deg_at_min, leg.arm_deg_at_max);
    arm.max_arm_deg = std::max(leg.arm_deg_at_min, leg.arm_deg_at_max);
  };
  set_limits(calibrated.main_left, *left);
  set_limits(calibrated.main_right, *right);
  set_limits(calibrated.rear_caster, *rear);
  return SeatLegMap(calibrated, *left, *right, *rear);
}

std::optional<SeatLegCounts> SeatLegMap::counts_for(const SeatPose &pose) const {
  const auto joints = kinematics_.inverse(pose);
  if (!joints) {
    return std::nullopt;
  }
  return SeatLegCounts{.main_left = main_left_.counts_for_arm(joints->main_left_arm_deg),
                       .main_right = main_right_.counts_for_arm(joints->main_right_arm_deg),
                       .rear_caster = rear_caster_.counts_for_arm(joints->rear_caster_arm_deg)};
}

std::optional<JointConfig> SeatLegMap::joints_at(const SeatLegCounts &counts,
                                                 float translation_mm) const {
  const auto left = main_left_.arm_for_counts(counts.main_left);
  const auto right = main_right_.arm_for_counts(counts.main_right);
  const auto rear = rear_caster_.arm_for_counts(counts.rear_caster);
  if (!left || !right || !rear) {
    return std::nullopt;
  }
  return JointConfig{.main_left_arm_deg = *left,
                     .main_right_arm_deg = *right,
                     .rear_caster_arm_deg = *rear,
                     .left_carriage_mm = translation_mm,
                     .right_carriage_mm = translation_mm};
}

std::optional<SeatPose> SeatLegMap::pose_at(const SeatLegCounts &counts,
                                            float translation_mm) const {
  const auto joints = joints_at(counts, translation_mm);
  if (!joints) {
    return std::nullopt;
  }
  return kinematics_.forward(*joints);
}

std::optional<SeatUrdfJoints> SeatLegMap::urdf_joints_at(const SeatLegCounts &counts) const {
  // The arms do not depend on the carriages, so any translation does.
  const auto joints = joints_at(counts, 0.0f);
  if (!joints) {
    return std::nullopt;
  }
  constexpr double rad_per_deg = 3.14159265358979323846 / 180.0;
  constexpr double m_per_mm = 0.001;
  const auto &geometry = kinematics_.geometry();
  auto rod_m = [](const TriangleLinkage &linkage, float arm_deg) {
    return (linkage_length_mm(linkage, arm_deg) - linkage_length_mm(linkage, 0.0f)) * m_per_mm;
  };
  return SeatUrdfJoints{
      .main_left_arm_rad = joints->main_left_arm_deg * rad_per_deg,
      .main_right_arm_rad = joints->main_right_arm_deg * rad_per_deg,
      .rear_caster_arm_rad = joints->rear_caster_arm_deg * rad_per_deg,
      .main_left_rod_m = rod_m(geometry.main_left_linkage, joints->main_left_arm_deg),
      .main_right_rod_m = rod_m(geometry.main_right_linkage, joints->main_right_arm_deg)};
}

} // namespace mib
