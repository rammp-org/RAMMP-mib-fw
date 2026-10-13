# Seat Kinematics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a pure, human-readable `SeatKinematics` component that maps a seat pose to joint positions and actuator degrees, and back.

**Architecture:** Three pure stages. `SeatKinematics::inverse/forward` maps seat pose ↔ joint positions with closed-form, per-leg geometry. `TriangleLinkage` maps main-leg arm angle ↔ actuator stroke. `Transmission` (built with `linear()` / `rotary()`) maps the joint-side quantity ↔ actuator degrees. Geometry values come from the Demo-Software URDF and live in `MIBconfig.hpp`.

**Tech Stack:** C++20, ESP-IDF ≥ 6.0 component layout, espp logger (example only). Math has no ESP-IDF dependency and is tested on the host with `c++`.

**Spec:** `docs/superpowers/specs/2026-10-13-seat-kinematics-design.md`

## Global Constraints

- Branch: `feat/20-seat-kinematics`. Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- Output is **actuator degrees**, never encoder counts.
- Front caster is not part of this module.
- Geometric IK only: no IMU, flat ground.
- Human readability is the top priority: named fields with units in every name (`_deg`, `_mm`), one explicit block per leg, a sketch/derivation comment above each formula, no templates, no `std::variant`, no index tables.
- Nothing is clamped; unreachable or out-of-limit requests return an empty `std::optional`.
- Chassis frame = `rammp` base frame (URDF root link): x forward, y left, z up, millimetres.
- Swing arms rotate about −y: `(x, z) -> (x·cos q − z·sin q, x·sin q + z·cos q)`.
- Seat pose signs: `pitch_deg > 0` front up; `roll_deg > 0` left up.
- Style: clang-format (LLVM based, 2-space indent, 100 columns, `const T &x`), enforced by the pre-commit hook. Doxygen `///` comments. Namespace `mib`; config in `mib::config`.
- Uncertain numbers carry `TODO(bench)` comments in `MIBconfig.hpp` — these are intentional and are the only allowed TODOs.

## Review Focus

- **A joint resting on its hard stop** (exactly ±45°, carriage at ±400 mm) must still round-trip through `to_actuators` → `from_actuators`, or seat state stops being reported. Pinned in Task 2 ("joints at their limits").
- **NaN in a command or a reading** must give an empty result, never NaN joint targets. Pinned in Task 2 (three NaN checks).
- **Left and right carriages disagree in feedback**: `forward()` reports translation as their mean. Pinned in Task 2.
- **Poses at the very edge of reach** (level seat at −64 mm and 231 mm) must pick a consistent root and round-trip. Pinned in Task 2.
- **A zero gear ratio or lead in config** would divide by zero in `from_actuator_deg`. Pinned in Task 2 with `static_assert`s in `MIBconfig.hpp` (zero lead already fails constexpr evaluation).

---

## File map

| File | Task | Responsibility |
|---|---|---|
| `components/SeatKinematics/include/transmission.hpp` | 1 | `Transmission`: joint-side quantity ↔ actuator degrees |
| `components/SeatKinematics/include/trig_equation.hpp` | 1 | `solve_cos_sin_equation`: shared `a·cos q + b·sin q = c` solver |
| `components/SeatKinematics/include/linkage.hpp` | 1 | `TriangleLinkage`: main-leg arm angle ↔ actuator stroke |
| `components/SeatKinematics/test/actuator_stage_checks.hpp` | 1 | Checks for Transmission + TriangleLinkage |
| `components/SeatKinematics/test/host/run.sh` | 1 | Build + run checks on the host |
| `components/SeatKinematics/test/host/main.cpp` | 1, 2 | Host test entry point |
| `components/SeatKinematics/include/seat_geometry.hpp` | 2 | `PointMm`, `SwingArmLeg`, `SeatGeometry` |
| `components/SeatKinematics/include/seat_kinematics.hpp` | 2 | `SeatPose`, `JointConfig`, `ActuatorPositions`, `SeatKinematics` |
| `components/SeatKinematics/src/seat_kinematics.cpp` | 2 | IK / FK / actuator mapping |
| `components/SeatKinematics/test/seat_kinematics_checks.hpp` | 2 | Checks for SeatKinematics |
| `components/MIBConfig/include/MIBconfig.hpp` | 2 | `mib::config::seat_geometry` values (URDF-derived) |
| `components/SeatKinematics/CMakeLists.txt` | 3 | ESP-IDF component registration |
| `components/MIBConfig/CMakeLists.txt` | 3 | `REQUIRES SeatKinematics` |
| `CMakeLists.txt` (root) | 3 | Add `SeatKinematics` to `COMPONENTS` |
| `components/SeatKinematics/example/**` | 3 | ESP-IDF example running the same checks on target |

There is no local ESP-IDF install on the development machine; ESP-IDF builds are verified by CI (`.github/workflows/build.yml`) once the branch is pushed. All math is verified on the host.

---

### Task 1: Actuator stage — Transmission, trig solver, TriangleLinkage

**Files:**
- Create: `components/SeatKinematics/include/transmission.hpp`
- Create: `components/SeatKinematics/include/trig_equation.hpp`
- Create: `components/SeatKinematics/include/linkage.hpp`
- Create: `components/SeatKinematics/test/actuator_stage_checks.hpp`
- Create: `components/SeatKinematics/test/host/main.cpp`
- Create: `components/SeatKinematics/test/host/run.sh`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `struct mib::Transmission { float deg_per_unit; float offset_deg; static constexpr Transmission linear(float travel_per_rev_mm, float gear_ratio, bool inverted = false, float offset_deg = 0.0f); static constexpr Transmission rotary(float gear_ratio, bool inverted = false, float offset_deg = 0.0f); constexpr float to_actuator_deg(float input) const; constexpr float from_actuator_deg(float actuator_deg) const; }`
  - `std::optional<float> mib::solve_cos_sin_equation(float a, float b, float c, float min_rad, float max_rad)`
  - `struct mib::TriangleLinkage { float arm_pin_x_mm, arm_pin_z_mm, plate_pin_x_mm, plate_pin_z_mm, retracted_length_mm; float length_mm(float arm_deg) const; float stroke_mm(float arm_deg) const; std::optional<float> arm_deg(float stroke_mm, float min_arm_deg, float max_arm_deg) const; }`
  - `int mib::test::run_actuator_stage_checks(void (*report)(const char *line))`

- [ ] **Step 1: Write the checks (failing test)**

Create `components/SeatKinematics/test/actuator_stage_checks.hpp`:

```cpp
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
      .retracted_length_mm = 287.59f,
  };

  // Expected lengths computed independently from the URDF in Python.
  check(near(linkage.length_mm(0.0f), 402.93f, 0.05f), "linkage: length at 0 deg = 402.93 mm");
  check(near(linkage.length_mm(-45.0f), 287.59f, 0.05f), "linkage: length at -45 deg = 287.59 mm");
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

  check(!linkage.arm_deg(400.0f, -45.0f, 45.0f), "linkage: 400 mm stroke is rejected");

  return failures;
}

} // namespace mib::test
```

Create `components/SeatKinematics/test/host/main.cpp`:

```cpp
// Host-side test for SeatKinematics. No ESP-IDF needed. Run: ./run.sh
#include <cstdio>

#include "actuator_stage_checks.hpp"

int main() {
  auto print_line = [](const char *line) { std::printf("%s\n", line); };

  const int failures = mib::test::run_actuator_stage_checks(print_line);

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
```

Create `components/SeatKinematics/test/host/run.sh` and make it executable (`chmod +x`):

```bash
#!/usr/bin/env bash
# Build and run the SeatKinematics checks on the host (macOS or Linux). No ESP-IDF needed.
set -euo pipefail
shopt -s nullglob
here="$(cd "$(dirname "$0")" && pwd)"
component="$here/../.."
sources=("$here/main.cpp" "$component"/src/*.cpp)
out="$(mktemp -d)"
c++ -std=c++20 -Wall -Wextra -Werror \
  -I "$component/include" -I "$component/test" -I "$component/../MIBConfig/include" \
  "${sources[@]}" -o "$out/seat_kinematics_host_test"
"$out/seat_kinematics_host_test"
```

- [ ] **Step 2: Run to verify it fails**

Run: `components/SeatKinematics/test/host/run.sh`
Expected: compile error, `'linkage.hpp' file not found`.

- [ ] **Step 3: Implement the headers**

Create `components/SeatKinematics/include/transmission.hpp`:

```cpp
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
```

Create `components/SeatKinematics/include/trig_equation.hpp`:

```cpp
#pragma once

#include <cmath>
#include <numbers>
#include <optional>

namespace mib {

/// @brief Solve  a·cos(q) + b·sin(q) = c  for q, and keep the root inside [min_rad, max_rad].
///
/// Both the leg IK and the actuator linkage reduce to this one equation. Rewriting the left side
/// as  R·cos(q − φ)  with  R = √(a² + b²)  and  φ = atan2(b, a)  gives the two roots
///
///     q = φ ± acos(c / R)
///
/// @return The root inside the range. If both roots are inside, the one closer to zero (the CAD
///         pose). Empty if there is no real root (|c| > R) or neither root is in range.
inline std::optional<float> solve_cos_sin_equation(float a, float b, float c, float min_rad,
                                                   float max_rad) {
  const float amplitude = std::hypot(a, b);
  if (amplitude <= 0.0f || std::fabs(c) > amplitude) {
    return std::nullopt;
  }
  const float phase = std::atan2(b, a);
  const float spread = std::acos(c / amplitude);

  // Bring each root into (−π, π] so it can be compared with the range.
  auto wrap = [](float angle) { return std::remainder(angle, 2.0f * std::numbers::pi_v<float>); };
  const float root_plus = wrap(phase + spread);
  const float root_minus = wrap(phase - spread);

  auto in_range = [&](float q) { return q >= min_rad && q <= max_rad; };
  const bool plus_ok = in_range(root_plus);
  const bool minus_ok = in_range(root_minus);
  if (plus_ok && minus_ok) {
    return std::fabs(root_plus) <= std::fabs(root_minus) ? root_plus : root_minus;
  }
  if (plus_ok) {
    return root_plus;
  }
  if (minus_ok) {
    return root_minus;
  }
  return std::nullopt;
}

} // namespace mib
```

Create `components/SeatKinematics/include/linkage.hpp`:

```cpp
#pragma once

#include <cmath>
#include <numbers>
#include <optional>

#include "trig_equation.hpp"

namespace mib {

/// @brief A linear actuator pinned between a swing arm and a fixed plate (main legs).
///
/// Side view, in the swing arm's plane (x forward, z up), origin at the arm's pivot O:
///
///                      B  pin on the carriage plate (does not move)
///                     /
///       actuator, length L
///                   /
///        A  pin on the swing arm (rotates with the arm about O)
///         \
///          O ─────────────────── wheel
///
/// As the arm turns by q, A moves to rotate(A, q) and the actuator length becomes
///
///     L(q) = | B − rotate(A, q) |
///
/// Arm rotation follows the URDF: about −y, so (x, z) -> (x·cos q − z·sin q, x·sin q + z·cos q).
struct TriangleLinkage {
  float arm_pin_x_mm{0.0f};   ///< A, on the swing arm, relative to the pivot, at q = 0.
  float arm_pin_z_mm{0.0f};   ///< A, on the swing arm, relative to the pivot, at q = 0.
  float plate_pin_x_mm{0.0f}; ///< B, on the carriage plate, relative to the pivot.
  float plate_pin_z_mm{0.0f}; ///< B, on the carriage plate, relative to the pivot.
  float retracted_length_mm{0.0f}; ///< Pin-to-pin length when the actuator reads zero stroke.

  /// @brief Pin-to-pin actuator length for a swing-arm angle.
  float length_mm(float arm_deg) const {
    const float q = arm_deg * std::numbers::pi_v<float> / 180.0f;
    const float moved_arm_pin_x = arm_pin_x_mm * std::cos(q) - arm_pin_z_mm * std::sin(q);
    const float moved_arm_pin_z = arm_pin_x_mm * std::sin(q) + arm_pin_z_mm * std::cos(q);
    return std::hypot(plate_pin_x_mm - moved_arm_pin_x, plate_pin_z_mm - moved_arm_pin_z);
  }

  /// @brief Actuator stroke (extension past fully retracted) for a swing-arm angle.
  float stroke_mm(float arm_deg) const { return length_mm(arm_deg) - retracted_length_mm; }

  /// @brief Swing-arm angle for an actuator stroke (the inverse of stroke_mm()).
  ///
  /// Expanding  L² = |B − rotate(A, q)|²  gives  |A|² + |B|² − 2·B·rotate(A, q) = L², i.e.
  ///
  ///     (B·A)·cos q + (Bz·Ax − Bx·Az)·sin q = (|A|² + |B|² − L²) / 2
  ///
  /// which is the a·cos q + b·sin q = c form.
  ///
  /// @param stroke_mm Actuator stroke.
  /// @param min_arm_deg, max_arm_deg Swing-arm travel; picks the root inside it.
  /// @return Swing-arm angle in degrees, or empty if no angle in range gives this stroke.
  std::optional<float> arm_deg(float stroke_mm, float min_arm_deg, float max_arm_deg) const {
    const float length = retracted_length_mm + stroke_mm;
    const float arm_pin_sq = arm_pin_x_mm * arm_pin_x_mm + arm_pin_z_mm * arm_pin_z_mm;
    const float plate_pin_sq = plate_pin_x_mm * plate_pin_x_mm + plate_pin_z_mm * plate_pin_z_mm;

    const float a = plate_pin_x_mm * arm_pin_x_mm + plate_pin_z_mm * arm_pin_z_mm;
    const float b = plate_pin_z_mm * arm_pin_x_mm - plate_pin_x_mm * arm_pin_z_mm;
    const float c = (arm_pin_sq + plate_pin_sq - length * length) / 2.0f;

    constexpr float deg_to_rad = std::numbers::pi_v<float> / 180.0f;
    auto q = solve_cos_sin_equation(a, b, c, min_arm_deg * deg_to_rad, max_arm_deg * deg_to_rad);
    if (!q) {
      return std::nullopt;
    }
    return *q / deg_to_rad;
  }
};

} // namespace mib
```

- [ ] **Step 4: Run to verify it passes**

Run: `components/SeatKinematics/test/host/run.sh`
Expected: 11 `PASS` lines, last line `0 failure(s)`, exit code 0.

- [ ] **Step 5: Format and commit**

```bash
pre-commit run --files components/SeatKinematics/include/*.hpp components/SeatKinematics/test/*.hpp components/SeatKinematics/test/host/main.cpp || true
components/SeatKinematics/test/host/run.sh
git add components/SeatKinematics
git commit -m "feat(seat): add Transmission and TriangleLinkage (#20)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

(`pre-commit` may reformat files and exit non-zero on the first run; rerunning the host test confirms formatting changed nothing semantic.)

---

### Task 2: Seat kinematics — geometry, IK/FK, URDF-derived config

**Files:**
- Create: `components/SeatKinematics/include/seat_geometry.hpp`
- Create: `components/SeatKinematics/include/seat_kinematics.hpp`
- Create: `components/SeatKinematics/src/seat_kinematics.cpp`
- Create: `components/SeatKinematics/test/seat_kinematics_checks.hpp`
- Modify: `components/SeatKinematics/test/host/main.cpp` (replace)
- Modify: `components/MIBConfig/include/MIBconfig.hpp` (add include after `#include <cstdint>`; append block before `} // namespace mib::config`)

**Interfaces:**
- Consumes: `Transmission`, `TriangleLinkage`, `solve_cos_sin_equation` from Task 1.
- Produces:
  - `struct mib::PointMm { float x_mm, y_mm, z_mm; }`
  - `struct mib::SwingArmLeg { PointMm pivot_mm; PointMm pivot_to_wheel_mm; float wheel_radius_mm, min_arm_deg, max_arm_deg; }`
  - `struct mib::SeatGeometry { SwingArmLeg main_left, main_right, rear_caster; TriangleLinkage main_left_linkage, main_right_linkage; float min_carriage_mm, max_carriage_mm; Transmission main_left_transmission, main_right_transmission, rear_caster_transmission, left_carriage_transmission, right_carriage_transmission; }`
  - `struct mib::SeatPose { float pitch_deg, roll_deg, elevation_mm, translation_mm; }`
  - `struct mib::JointConfig { float main_left_arm_deg, main_right_arm_deg, rear_caster_arm_deg, left_carriage_mm, right_carriage_mm; }`
  - `struct mib::ActuatorPositions { float main_left_deg, main_right_deg, rear_caster_deg, left_carriage_deg, right_carriage_deg; }`
  - `class mib::SeatKinematics` with `explicit SeatKinematics(const SeatGeometry &)`, `inverse`, `forward`, `to_actuators`, `from_actuators` (all `const`, all return `std::optional`).
  - `inline constexpr mib::SeatGeometry mib::config::seat_geometry`
  - `int mib::test::run_seat_kinematics_checks(const SeatKinematics &, void (*report)(const char *line))`

- [ ] **Step 1: Write the checks (failing test)**

Create `components/SeatKinematics/test/seat_kinematics_checks.hpp`:

```cpp
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

  check(!kinematics.inverse({.pitch_deg = 0.0f,
                             .roll_deg = 0.0f,
                             .elevation_mm = 1000.0f,
                             .translation_mm = 0.0f}),
        "elevation 1000 mm is rejected");
  check(!kinematics.inverse({.pitch_deg = 0.0f,
                             .roll_deg = 0.0f,
                             .elevation_mm = 100.0f,
                             .translation_mm = 500.0f}),
        "translation 500 mm is rejected");
  check(!kinematics.inverse({.pitch_deg = 40.0f,
                             .roll_deg = 0.0f,
                             .elevation_mm = 100.0f,
                             .translation_mm = 0.0f}),
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
```

Replace `components/SeatKinematics/test/host/main.cpp` with:

```cpp
// Host-side test for SeatKinematics. No ESP-IDF needed. Run: ./run.sh
#include <cstdio>

#include "MIBconfig.hpp"
#include "actuator_stage_checks.hpp"
#include "seat_kinematics_checks.hpp"

int main() {
  auto print_line = [](const char *line) { std::printf("%s\n", line); };

  int failures = mib::test::run_actuator_stage_checks(print_line);

  const mib::SeatKinematics kinematics(mib::config::seat_geometry);
  failures += mib::test::run_seat_kinematics_checks(kinematics, print_line);

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `components/SeatKinematics/test/host/run.sh`
Expected: compile error, `'seat_kinematics.hpp' file not found`.

- [ ] **Step 3: Add the geometry types and the class**

Create `components/SeatKinematics/include/seat_geometry.hpp`:

```cpp
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
  Transmission main_left_transmission;   ///< Input: actuator stroke, mm.
  Transmission main_right_transmission;  ///< Input: actuator stroke, mm.
  Transmission rear_caster_transmission; ///< Input: swing-arm angle, deg (encoder after shock).
  Transmission left_carriage_transmission;  ///< Input: carriage travel, mm.
  Transmission right_carriage_transmission; ///< Input: carriage travel, mm.
};

} // namespace mib
```

Create `components/SeatKinematics/include/seat_kinematics.hpp`:

```cpp
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
  explicit SeatKinematics(const SeatGeometry &geometry) : geometry_(geometry) {}

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
```

Create `components/SeatKinematics/src/seat_kinematics.cpp`:

```cpp
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
std::optional<float> arm_deg_on_ground(const SwingArmLeg &leg, float carriage_mm,
                                       const PointMm &up, float elevation_mm) {
  const PointMm &w = leg.pivot_to_wheel_mm;
  const float a = up.x_mm * w.x_mm + up.z_mm * w.z_mm;
  const float b = up.z_mm * w.x_mm - up.x_mm * w.z_mm;
  const float c = leg.wheel_radius_mm - elevation_mm - dot(up, leg_base_point(leg, carriage_mm));

  auto q = solve_cos_sin_equation(a, b, c, leg.min_arm_deg * deg_to_rad,
                                  leg.max_arm_deg * deg_to_rad);
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
  const PointMm left = wheel_centre(geometry_.main_left, joints.left_carriage_mm,
                                    joints.main_left_arm_deg);
  const PointMm right = wheel_centre(geometry_.main_right, joints.right_carriage_mm,
                                     joints.main_right_arm_deg);
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
  const float main_left_stroke_mm =
      geometry_.main_left_linkage.stroke_mm(joints.main_left_arm_deg);
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
```

- [ ] **Step 4: Add the URDF-derived values to `MIBconfig.hpp`**

In `components/MIBConfig/include/MIBconfig.hpp`, after `#include <cstdint>` add:

```cpp

#include "seat_geometry.hpp"
```

Then, immediately before the final `} // namespace mib::config`, insert:

```cpp
/// @brief Seat kinematics geometry (millimetres, degrees).
///
/// Derived from the Demo-Software URDF
/// (core/rammp_prototype_description/urdf/rammp_prototype_base.urdf), converted to millimetres.
/// Chassis frame = rammp base frame (URDF root link): x forward, y left, z up.
/// Each comment names the URDF joint(s) a number comes from.
inline constexpr SeatGeometry seat_geometry{
    .main_left =
        {
            // dw_main_plate_l origin + motor_swing_arm_l origin
            .pivot_mm = {.x_mm = 166.82f, .y_mm = 182.00f, .z_mm = 113.42f},
            // drive_motor_l origin + drive_wheel_l origin
            .pivot_to_wheel_mm = {.x_mm = 318.77f, .y_mm = 119.54f, .z_mm = -13.80f},
            .wheel_radius_mm = 184.06f, // drive_wheel_l collision cylinder
            .min_arm_deg = -45.0f,      // motor_swing_arm_l limits
            .max_arm_deg = 45.0f,
        },
    .main_right =
        {
            // dw_main_plate_r origin + motor_swing_arm_r origin
            .pivot_mm = {.x_mm = 166.82f, .y_mm = -187.12f, .z_mm = 113.42f},
            // drive_motor_r origin + drive_wheel_r origin
            .pivot_to_wheel_mm = {.x_mm = 318.77f, .y_mm = -119.55f, .z_mm = -13.80f},
            .wheel_radius_mm = 184.06f, // drive_wheel_r collision cylinder
            .min_arm_deg = -45.0f,      // motor_swing_arm_r limits
            .max_arm_deg = 45.0f,
        },
    .rear_caster =
        {
            // rear_caster_arm_mount origin + rear_caster_swing_arm origin
            .pivot_mm = {.x_mm = -31.75f, .y_mm = 0.11f, .z_mm = 8.89f},
            // rear_casteer_wheel_l / rear_caster_wheel_r origins, y averaged to one wheel
            .pivot_to_wheel_mm = {.x_mm = -209.55f, .y_mm = 0.0f, .z_mm = 12.70f},
            .wheel_radius_mm = 101.07f, // rear_caster_wheel_r collision cylinder
            .min_arm_deg = -45.0f,      // rear_caster_swing_arm limits
            .max_arm_deg = 45.0f,
        },

    // Pins relative to the swing-arm pivot, in the arm's (x, z) plane.
    // Arm pin: motor_elevator_l origin. Plate pin: motor_elevator_pivot_l origin.
    // TODO(bench): confirm the plate pin from CAD. motor_elevator_rod_link_l is the other
    // candidate (50.8 mm higher) and changes stroke by up to ~13 mm at the ends of travel.
    // TODO(bench): retracted length. Placeholder = pin-to-pin length at arm angle −45°.
    .main_left_linkage =
        {
            .arm_pin_x_mm = -103.72f,
            .arm_pin_z_mm = 126.68f,
            .plate_pin_x_mm = 298.85f,
            .plate_pin_z_mm = 109.69f,
            .retracted_length_mm = 287.59f,
        },
    .main_right_linkage =
        {
            .arm_pin_x_mm = -103.72f,
            .arm_pin_z_mm = 126.68f,
            .plate_pin_x_mm = 298.85f,
            .plate_pin_z_mm = 109.69f,
            .retracted_length_mm = 287.59f,
        },

    // dw_main_plate_l / dw_main_plate_r limits.
    // TODO(bench): Demo-Software's calibrated carriage range was 0–300 mm.
    .min_carriage_mm = -400.0f,
    .max_carriage_mm = 400.0f,

    // TODO(bench): every transmission below is a placeholder (1 mm per rev, 1:1, no offset)
    // until the actuator lead, gear ratios and directions are measured.
    .main_left_transmission = Transmission::linear(1.0f, 1.0f),
    .main_right_transmission = Transmission::linear(1.0f, 1.0f),
    // Encoder sits after the shock, on the swing arm: 1:1 is real; sign and offset are not.
    .rear_caster_transmission = Transmission::rotary(1.0f),
    .left_carriage_transmission = Transmission::linear(1.0f, 1.0f),
    .right_carriage_transmission = Transmission::linear(1.0f, 1.0f),
};

// A zero gear ratio would make actuator degrees -> joint position divide by zero.
static_assert(seat_geometry.main_left_transmission.deg_per_unit != 0.0f);
static_assert(seat_geometry.main_right_transmission.deg_per_unit != 0.0f);
static_assert(seat_geometry.rear_caster_transmission.deg_per_unit != 0.0f);
static_assert(seat_geometry.left_carriage_transmission.deg_per_unit != 0.0f);
static_assert(seat_geometry.right_carriage_transmission.deg_per_unit != 0.0f);
```

- [ ] **Step 5: Run to verify it passes**

Run: `components/SeatKinematics/test/host/run.sh`
Expected: 29 `PASS` lines, last line `0 failure(s)`, exit code 0.

- [ ] **Step 6: Mutation sanity check (do not commit)**

Temporarily change `return {std::sin(p), ` to `return {-std::sin(p), ` in `world_up()` and rerun `run.sh`.
Expected: `FAIL pitch +5: main wheels drop, rear wheel rises` (and the pose-grid round trip fails). Revert the change and rerun: `0 failure(s)`.

- [ ] **Step 7: Format and commit**

```bash
pre-commit run --files components/SeatKinematics/include/*.hpp components/SeatKinematics/src/*.cpp components/SeatKinematics/test/*.hpp components/SeatKinematics/test/host/main.cpp components/MIBConfig/include/MIBconfig.hpp || true
components/SeatKinematics/test/host/run.sh
git add components/SeatKinematics components/MIBConfig/include/MIBconfig.hpp
git commit -m "feat(seat): add seat pose <-> joint <-> actuator kinematics (#20)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: ESP-IDF integration and example project

**Files:**
- Create: `components/SeatKinematics/CMakeLists.txt`
- Modify: `components/MIBConfig/CMakeLists.txt`
- Modify: `CMakeLists.txt` (root, `COMPONENTS` list)
- Create: `components/SeatKinematics/example/CMakeLists.txt`
- Create: `components/SeatKinematics/example/sdkconfig.defaults`
- Create: `components/SeatKinematics/example/main/CMakeLists.txt`
- Create: `components/SeatKinematics/example/main/idf_component.yml`
- Create: `components/SeatKinematics/example/main/seat_kinematics_example.cpp`

**Interfaces:**
- Consumes: everything from Tasks 1–2.
- Produces: ESP-IDF component `SeatKinematics`; `MIBConfig` now requires it; main firmware compiles it.

- [ ] **Step 1: Register the component**

Create `components/SeatKinematics/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "src/seat_kinematics.cpp"
                       INCLUDE_DIRS "include")
```

Replace `components/MIBConfig/CMakeLists.txt` with:

```cmake
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES SeatKinematics
)
```

In the root `CMakeLists.txt`, change the `COMPONENTS` string to:

```cmake
  "main esptool_py bsp MIBConfig MibSystem DifferentialDrive DriveController SeatKinematics rammp_rtps_messages"
```

- [ ] **Step 2: Add the example project**

Create `components/SeatKinematics/example/CMakeLists.txt`:

```cmake
# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../"
)

set(
  COMPONENTS
  "main esptool_py SeatKinematics MIBConfig"
  CACHE STRING
  "List of components to include"
  )

project(seat_kinematics_example)

set(CMAKE_CXX_STANDARD 20)
```

Create `components/SeatKinematics/example/sdkconfig.defaults`:

```ini
CONFIG_IDF_TARGET="esp32p4"

# The RAMMP MIB boards are pre-v3 silicon.
CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y
CONFIG_ESP32P4_REV_MIN_100=y
```

Create `components/SeatKinematics/example/main/CMakeLists.txt`:

```cmake
idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "." "../../test"
                       REQUIRES SeatKinematics MIBConfig)
```

Create `components/SeatKinematics/example/main/idf_component.yml`:

```yaml
## IDF Component Manager Manifest File
dependencies:
  idf:
    version: '>=6.0'
  espp/logger: '>=1.3'
```

Create `components/SeatKinematics/example/main/seat_kinematics_example.cpp`:

```cpp
#include "MIBconfig.hpp"
#include "actuator_stage_checks.hpp"
#include "logger.hpp"
#include "seat_kinematics.hpp"
#include "seat_kinematics_checks.hpp"

namespace {

espp::Logger &example_logger() {
  static espp::Logger logger(
      {.tag = "SeatKinematics Example", .level = espp::Logger::Verbosity::INFO});
  return logger;
}

void log_line(const char *line) { example_logger().info("{}", line); }

} // namespace

extern "C" void app_main(void) {
  auto &logger = example_logger();

  //! [seat kinematics example]
  const mib::SeatKinematics kinematics(mib::config::seat_geometry);

  // Tilt back 3 degrees, seat origin 120 mm above the ground, carriages 50 mm forward.
  const mib::SeatPose pose{
      .pitch_deg = 3.0f, .roll_deg = 0.0f, .elevation_mm = 120.0f, .translation_mm = 50.0f};

  auto joints = kinematics.inverse(pose);
  if (!joints) {
    logger.warn("Pose is unreachable");
  } else {
    logger.info("Arms: main left {:.2f} deg, main right {:.2f} deg, rear caster {:.2f} deg",
                joints->main_left_arm_deg, joints->main_right_arm_deg,
                joints->rear_caster_arm_deg);
    logger.info("Carriages: left {:.1f} mm, right {:.1f} mm", joints->left_carriage_mm,
                joints->right_carriage_mm);

    auto actuators = kinematics.to_actuators(*joints);
    if (actuators) {
      logger.info("Actuators (deg): main left {:.1f}, main right {:.1f}, rear caster {:.1f}, "
                  "left carriage {:.1f}, right carriage {:.1f}",
                  actuators->main_left_deg, actuators->main_right_deg,
                  actuators->rear_caster_deg, actuators->left_carriage_deg,
                  actuators->right_carriage_deg);
    }
  }
  //! [seat kinematics example]

  int failures = mib::test::run_actuator_stage_checks(log_line);
  failures += mib::test::run_seat_kinematics_checks(kinematics, log_line);
  logger.info("{} failure(s)", failures);
}
```

- [ ] **Step 3: Verify**

Run: `components/SeatKinematics/test/host/run.sh`
Expected: `0 failure(s)` (unchanged by this task).

If ESP-IDF is available (`idf.py --version` succeeds):
- `idf.py -C components/SeatKinematics/example set-target esp32p4 build` → build succeeds.
- `idf.py build` at the repo root → build succeeds (needs `git submodule update --init --recursive` first).
- Flash the example and confirm the log ends with `0 failure(s)`.

If ESP-IDF is not available, say so explicitly in the hand-off; the ESP-IDF build is then verified by CI after the branch is pushed (pushing needs the user's go-ahead).

- [ ] **Step 4: Format and commit**

```bash
pre-commit run --files components/SeatKinematics/example/main/seat_kinematics_example.cpp || true
git add CMakeLists.txt components/MIBConfig/CMakeLists.txt components/SeatKinematics
git commit -m "build(seat): register SeatKinematics and add example project (#20)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
