#pragma once

#include <cmath>
#include <cstdio>

#include "seat_kinematics.hpp"

namespace mib::test {

/// @brief Runs every SeatKinematics check, reporting one line per check.
///
/// Shared by the host test (test/host/) and the ESP-IDF example, so both run the same checks.
///
/// @param kinematics The kinematics under test, normally built from mib::config::seat_geometry.
/// @param report Called with each result line, e.g. "PASS neutral pose round trip".
/// @return Number of failed checks (0 = all passed).
inline int run_seat_kinematics_checks(const SeatKinematics &kinematics,
                                      void (*report)(const char *line)) {
  int failures = 0;
  char line[160];

  auto check = [&](bool passed, const char *name) {
    std::snprintf(line, sizeof(line), "%s %s", passed ? "PASS" : "FAIL", name);
    report(line);
    if (!passed) {
      ++failures;
    }
  };
  auto near = [](float a, float b, float tolerance) { return std::fabs(a - b) <= tolerance; };
  constexpr float angle_tolerance_deg = 0.01f;
  constexpr float length_tolerance_mm = 0.01f;

  auto same_pose = [&](const SeatPose &a, const SeatPose &b) {
    return near(a.pitch_deg, b.pitch_deg, angle_tolerance_deg) &&
           near(a.roll_deg, b.roll_deg, angle_tolerance_deg) &&
           near(a.elevation_mm, b.elevation_mm, length_tolerance_mm) &&
           near(a.translation_mm, b.translation_mm, length_tolerance_mm);
  };
  auto same_joints = [&](const JointConfig &a, const JointConfig &b) {
    return near(a.main_left_arm_deg, b.main_left_arm_deg, angle_tolerance_deg) &&
           near(a.main_right_arm_deg, b.main_right_arm_deg, angle_tolerance_deg) &&
           near(a.rear_caster_arm_deg, b.rear_caster_arm_deg, angle_tolerance_deg) &&
           near(a.left_carriage_mm, b.left_carriage_mm, length_tolerance_mm) &&
           near(a.right_carriage_mm, b.right_carriage_mm, length_tolerance_mm);
  };

  const SeatPose neutral{
      .pitch_deg = 0.0f, .roll_deg = 0.0f, .elevation_mm = 100.0f, .translation_mm = 0.0f};

  // --- Round trips -------------------------------------------------------------------------

  {
    auto joints = kinematics.inverse(neutral);
    auto pose = joints ? kinematics.forward(*joints) : std::nullopt;
    check(pose && same_pose(*pose, neutral), "neutral pose: pose -> joints -> pose");
  }

  {
    bool all_reachable = true;
    bool all_round_trip = true;
    for (float pitch = -5.0f; pitch <= 5.0f; pitch += 2.5f) {
      for (float roll = -3.0f; roll <= 3.0f; roll += 1.5f) {
        for (float elevation = 60.0f; elevation <= 160.0f; elevation += 25.0f) {
          for (float translation = -100.0f; translation <= 100.0f; translation += 50.0f) {
            const SeatPose requested{pitch, roll, elevation, translation};
            auto joints = kinematics.inverse(requested);
            if (!joints) {
              all_reachable = false;
              continue;
            }
            auto pose = kinematics.forward(*joints);
            if (!pose || !same_pose(*pose, requested)) {
              all_round_trip = false;
            }
          }
        }
      }
    }
    check(all_reachable, "pose grid (pitch +-5, roll +-3, elev 60-160, trans +-100) reachable");
    check(all_round_trip, "pose grid: pose -> joints -> pose");
  }

  {
    const JointConfig joints{.main_left_arm_deg = 10.0f,
                             .main_right_arm_deg = -5.0f,
                             .rear_caster_arm_deg = 20.0f,
                             .left_carriage_mm = 50.0f,
                             .right_carriage_mm = 50.0f};
    auto actuators = kinematics.to_actuators(joints);
    auto back = actuators ? kinematics.from_actuators(*actuators) : std::nullopt;
    check(back && same_joints(*back, joints), "joints -> actuators -> joints");
  }

  {
    // Feedback from a joint resting on its hard stop must still be reported.
    bool all_round_trip = true;
    for (float limit_deg : {-45.0f, 45.0f}) {
      const JointConfig joints{.main_left_arm_deg = limit_deg,
                               .main_right_arm_deg = limit_deg,
                               .rear_caster_arm_deg = limit_deg,
                               .left_carriage_mm = 400.0f,
                               .right_carriage_mm = -400.0f};
      auto actuators = kinematics.to_actuators(joints);
      auto back = actuators ? kinematics.from_actuators(*actuators) : std::nullopt;
      if (!back || !same_joints(*back, joints)) {
        all_round_trip = false;
      }
    }
    check(all_round_trip, "joints at their limits: joints -> actuators -> joints");
  }

  {
    // The edges of the reachable range (level seat, found by sweeping on the host).
    bool all_round_trip = true;
    for (float elevation_mm : {-64.0f, 231.0f}) {
      const SeatPose requested{.pitch_deg = 0.0f,
                               .roll_deg = 0.0f,
                               .elevation_mm = elevation_mm,
                               .translation_mm = 0.0f};
      auto joints = kinematics.inverse(requested);
      auto pose = joints ? kinematics.forward(*joints) : std::nullopt;
      if (!pose || !same_pose(*pose, requested)) {
        all_round_trip = false;
      }
    }
    check(all_round_trip, "elevation -64 and 231 mm (edges of reach): pose -> joints -> pose");
  }

  {
    // Measured carriages can disagree; translation is reported as their mean.
    auto pose = kinematics.forward({.main_left_arm_deg = 0.0f,
                                    .main_right_arm_deg = 0.0f,
                                    .rear_caster_arm_deg = 0.0f,
                                    .left_carriage_mm = 40.0f,
                                    .right_carriage_mm = 60.0f});
    check(pose && near(pose->translation_mm, 50.0f, length_tolerance_mm),
          "carriages at 40 and 60 mm: translation reported as 50 mm");
  }

  // --- Known behaviour ---------------------------------------------------------------------
  // Arm angle sign (URDF, rotation about -y): a positive angle raises a wheel that sits in
  // front of its pivot (main legs) and lowers a wheel that sits behind it (rear caster).

  const auto level = kinematics.inverse(neutral);

  {
    // Front up: the main wheels (forward) drop and the rear wheel rises, relative to the chassis.
    SeatPose request = neutral;
    request.pitch_deg = 5.0f;
    auto pitched = kinematics.inverse(request);
    check(level && pitched && pitched->main_left_arm_deg < level->main_left_arm_deg &&
              pitched->main_right_arm_deg < level->main_right_arm_deg &&
              pitched->rear_caster_arm_deg < level->rear_caster_arm_deg,
          "pitch +5: main wheels drop, rear wheel rises");
  }

  {
    // Left up: the left main wheel drops and the right main wheel rises.
    SeatPose request = neutral;
    request.roll_deg = 3.0f;
    auto rolled = kinematics.inverse(request);
    check(level && rolled && rolled->main_left_arm_deg < level->main_left_arm_deg &&
              rolled->main_right_arm_deg > level->main_right_arm_deg,
          "roll +3: left main wheel drops, right main wheel rises");
  }

  {
    // Seat up: all three wheels drop relative to the chassis.
    SeatPose request = neutral;
    request.elevation_mm = 150.0f;
    auto raised = kinematics.inverse(request);
    check(level && raised && raised->main_left_arm_deg < level->main_left_arm_deg &&
              raised->main_right_arm_deg < level->main_right_arm_deg &&
              raised->rear_caster_arm_deg > level->rear_caster_arm_deg,
          "elevation +50 mm: all three wheels drop");
  }

  {
    auto joints = kinematics.inverse(
        {.pitch_deg = 0.0f, .roll_deg = 0.0f, .elevation_mm = 100.0f, .translation_mm = 75.0f});
    check(joints && joints->left_carriage_mm == 75.0f && joints->right_carriage_mm == 75.0f,
          "translation 75 mm: both carriages at 75 mm");
  }

  // --- Rejections --------------------------------------------------------------------------

  check(!kinematics.inverse(
            {.pitch_deg = 0.0f, .roll_deg = 0.0f, .elevation_mm = 1000.0f, .translation_mm = 0.0f}),
        "elevation 1000 mm is rejected");
  check(
      !kinematics.inverse(
          {.pitch_deg = 0.0f, .roll_deg = 0.0f, .elevation_mm = 100.0f, .translation_mm = 500.0f}),
      "translation 500 mm is rejected");
  check(!kinematics.inverse(
            {.pitch_deg = 40.0f, .roll_deg = 0.0f, .elevation_mm = 100.0f, .translation_mm = 0.0f}),
        "pitch 40 deg is rejected");
  check(!kinematics.to_actuators({.main_left_arm_deg = 60.0f}),
        "to_actuators rejects a 60 deg main arm");

  check(!kinematics.inverse({.pitch_deg = std::nanf(""),
                             .roll_deg = 0.0f,
                             .elevation_mm = 100.0f,
                             .translation_mm = 0.0f}),
        "NaN pitch is rejected");
  check(!kinematics.inverse({.pitch_deg = 0.0f,
                             .roll_deg = 0.0f,
                             .elevation_mm = 100.0f,
                             .translation_mm = std::nanf("")}),
        "NaN translation is rejected");
  check(!kinematics.from_actuators({.main_left_deg = std::nanf("")}),
        "from_actuators rejects a NaN reading");

  return failures;
}

} // namespace mib::test
