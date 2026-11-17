// Host-side checks for SeatLegMap. No ESP-IDF needed. Run: ./run.sh
#include <cstdio>
#include <cstdlib>

#include "MIBconfig.hpp"
#include "check_reporter.hpp"
#include "seat_leg_map.hpp"

namespace {

using mib::CountRange;
using mib::SeatLegCalibration;
using mib::SeatLegCounts;
using mib::SeatLegMap;
using mib::SeatPose;
using mib::test::near;

// The URDF arm limits, lower count at the seat-low end (wheel up): the MIBconfig.hpp default.
constexpr SeatLegCalibration urdf_ends{
    .main_left = {.arm_deg_at_min_counts = 45.0f, .arm_deg_at_max_counts = -45.0f},
    .main_right = {.arm_deg_at_min_counts = 45.0f, .arm_deg_at_max_counts = -45.0f},
    .rear_caster = {.arm_deg_at_min_counts = -45.0f, .arm_deg_at_max_counts = 45.0f},
    .main_leg_encoder_on_actuator = true,
};
constexpr CountRange range{1000, 81000};

SeatPose level(float elevation_mm) {
  return {.pitch_deg = 0.0f, .roll_deg = 0.0f, .elevation_mm = elevation_mm,
          .translation_mm = 0.0f};
}

bool within(const SeatLegCounts &c, CountRange r) {
  auto in = [&](int32_t v) { return v >= r.min && v <= r.max; };
  return in(c.main_left) && in(c.main_right) && in(c.rear_caster);
}

} // namespace

int main() {
  mib::test::CheckReporter check([](const char *line) { std::printf("%s\n", line); });
  const auto &geometry = mib::config::seat_geometry;

  auto map = SeatLegMap::create(geometry, urdf_ends, range, range, range);
  check(map.has_value(), "the URDF-limit calibration builds a map");
  if (!map) {
    std::printf("%d failure(s)\n", check.failures());
    return 1;
  }

  {
    // Level seat, 20 to 220 mm: counts stay in range and read back to the same pose.
    bool ok = true;
    for (float elevation = 20.0f; elevation <= 220.0f; elevation += 10.0f) {
      const auto counts = map->counts_for(level(elevation));
      const auto pose = counts ? map->pose_at(*counts, 0.0f) : std::nullopt;
      if (!counts || !within(*counts, range) || !pose || !near(pose->elevation_mm, elevation, 0.1f) ||
          !near(pose->pitch_deg, 0.0f, 0.01f) || !near(pose->roll_deg, 0.0f, 0.01f)) {
        std::printf("  at %.0f mm\n", elevation);
        ok = false;
      }
    }
    check(ok, "level elevation 20..220 mm: counts in range, read back within 0.1 mm");
  }

  {
    // Raising the seat lowers every wheel: toward the seat-high end of each leg's range, which
    // this calibration puts at the higher count.
    const auto low = map->counts_for(level(60.0f));
    const auto high = map->counts_for(level(160.0f));
    check(low && high && high->main_left > low->main_left && high->main_right > low->main_right &&
              high->rear_caster > low->rear_caster,
          "elevation 60 -> 160 mm: every leg's counts rise toward the seat-high end");
    check(low && high && low->main_left == low->main_right && high->main_left == high->main_right,
          "level seat: the two main legs get the same counts");
  }

  {
    // Swapping which end has the lower count mirrors the counts in the range.
    SeatLegCalibration flipped = urdf_ends;
    std::swap(flipped.main_left.arm_deg_at_min_counts, flipped.main_left.arm_deg_at_max_counts);
    const auto flipped_map = SeatLegMap::create(geometry, flipped, range, range, range);
    const auto a = map->counts_for(level(120.0f));
    const auto b = flipped_map ? flipped_map->counts_for(level(120.0f)) : std::nullopt;
    check(a && b && std::abs((a->main_left - range.min) - (range.max - b->main_left)) <= 1 &&
              a->main_right == b->main_right,
          "swapped main-left ends mirror only main left's counts");
  }

  {
    // Counts follow stroke, not arm angle, for an encoder on the linear actuator: half the
    // count span is half the stroke, which is not the middle arm angle (0 deg).
    const auto joints = map->joints_at({.main_left = 41000, .main_right = 41000, .rear_caster = 41000},
                                       0.0f);
    SeatLegCalibration on_arm = urdf_ends;
    on_arm.main_leg_encoder_on_actuator = false;
    const auto arm_map = SeatLegMap::create(geometry, on_arm, range, range, range);
    const auto arm_joints = arm_map ? arm_map->joints_at({41000, 41000, 41000}, 0.0f) : std::nullopt;
    check(joints && arm_joints && near(arm_joints->main_left_arm_deg, 0.0f, 0.01f) &&
              !near(joints->main_left_arm_deg, 0.0f, 0.5f) &&
              near(joints->rear_caster_arm_deg, 0.0f, 0.01f),
          "mid-range counts: arm 0 deg for an encoder on the arm, not for one on the actuator");
  }

  {
    // Narrower calibrated ends narrow the reachable elevations.
    SeatLegCalibration narrow = urdf_ends;
    narrow.main_left = {.arm_deg_at_min_counts = 10.0f, .arm_deg_at_max_counts = -10.0f};
    const auto narrow_map = SeatLegMap::create(geometry, narrow, range, range, range);
    const auto full = map->counts_for(level(200.0f));
    const auto refused = narrow_map ? narrow_map->counts_for(level(200.0f)) : std::nullopt;
    check(full && narrow_map && !refused,
          "an elevation past a leg's calibrated end is refused, though the URDF limits allow it");
  }

  check(!map->counts_for(level(1000.0f)), "elevation 1000 mm is refused");
  check(!SeatLegMap::create(geometry, urdf_ends, {5000, 5000}, range, range),
        "an empty range is refused");
  {
    SeatLegCalibration same = urdf_ends;
    same.rear_caster = {.arm_deg_at_min_counts = 0.0f, .arm_deg_at_max_counts = 0.0f};
    check(!SeatLegMap::create(geometry, same, range, range, range),
          "equal end angles are refused");
  }
  check(SeatLegMap::create(geometry, mib::config::seat_leg_calibration, range, range, range)
            .has_value(),
        "MIBconfig.hpp's seat_leg_calibration builds a map");

  std::printf("%d failure(s)\n", check.failures());
  return check.failures() == 0 ? 0 : 1;
}
