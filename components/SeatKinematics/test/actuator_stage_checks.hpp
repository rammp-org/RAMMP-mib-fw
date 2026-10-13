#pragma once

#include <cmath>
#include <cstdio>

#include "linkage.hpp"
#include "transmission.hpp"

namespace mib::test {

/// @brief Checks for Transmission and TriangleLinkage, reporting one line per check.
///
/// Shared by the host test (test/host/) and the ESP-IDF example.
///
/// @param report Called with each result line, e.g. "PASS transmission: linear 2 mm lead, 10:1".
/// @return Number of failed checks (0 = all passed).
inline int run_actuator_stage_checks(void (*report)(const char *line)) {
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

  // --- Transmission ------------------------------------------------------------------------

  {
    // 2 mm per screw turn, motor turns 10 times per screw turn:
    // 1 mm = 0.5 screw turns = 5 motor turns = 1800 motor degrees.
    const auto screw = Transmission::linear(2.0f, 10.0f);
    check(near(screw.to_actuator_deg(1.0f), 1800.0f, 1e-3f),
          "transmission: linear 2 mm/rev, 10:1 -> 1 mm = 1800 deg");
  }
  {
    const auto inverted = Transmission::linear(2.0f, 10.0f, true, 90.0f);
    check(near(inverted.to_actuator_deg(1.0f), -1710.0f, 1e-3f),
          "transmission: inverted with 90 deg offset -> 1 mm = -1710 deg");
    check(near(inverted.from_actuator_deg(inverted.to_actuator_deg(37.5f)), 37.5f, 1e-3f),
          "transmission: linear round trip");
  }
  {
    const auto joint = Transmission::rotary(3.0f, false, -10.0f);
    check(near(joint.to_actuator_deg(20.0f), 50.0f, 1e-3f),
          "transmission: rotary 3:1, -10 deg offset -> 20 deg = 50 deg");
    check(near(joint.from_actuator_deg(50.0f), 20.0f, 1e-3f), "transmission: rotary round trip");
  }

  // --- TriangleLinkage ---------------------------------------------------------------------

  // Main-leg pins from the URDF (see MIBconfig.hpp).
  const TriangleLinkage linkage{
      .arm_pin_x_mm = -103.72f,
      .arm_pin_z_mm = 126.68f,
      .plate_pin_x_mm = 298.85f,
      .plate_pin_z_mm = 109.69f,
      .retracted_length_mm = 287.58f,
  };

  // Expected lengths computed independently from the URDF in Python.
  check(near(linkage.length_mm(0.0f), 402.93f, 0.05f), "linkage: length at 0 deg = 402.93 mm");
  check(near(linkage.length_mm(-45.0f), 287.58f, 0.05f), "linkage: length at -45 deg = 287.58 mm");
  check(near(linkage.length_mm(45.0f), 471.13f, 0.05f), "linkage: length at +45 deg = 471.13 mm");
  check(near(linkage.stroke_mm(-45.0f), 0.0f, 0.05f), "linkage: stroke at -45 deg = 0 mm");

  {
    bool all_round_trip = true;
    for (float arm_deg = -45.0f; arm_deg <= 45.0f; arm_deg += 5.0f) {
      auto back = linkage.arm_deg(linkage.stroke_mm(arm_deg), -45.0f, 45.0f);
      if (!back || !near(*back, arm_deg, 0.01f)) {
        all_round_trip = false;
      }
    }
    check(all_round_trip, "linkage: arm -> stroke -> arm, -45..45 deg");
  }

  {
    // A stroke a hair past the -45 deg stop (float rounding) is snapped onto the stop.
    auto snapped = linkage.arm_deg(linkage.stroke_mm(-45.0f) - 1e-4f, -45.0f, 45.0f);
    check(snapped && *snapped == -45.0f, "linkage: stroke 1e-4 mm past -45 deg stop snaps to -45");
  }

  check(!linkage.arm_deg(400.0f, -45.0f, 45.0f), "linkage: 400 mm stroke is rejected");

  return failures;
}

} // namespace mib::test
