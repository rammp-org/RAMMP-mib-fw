# Seat Kinematics — Design

**Issue:** #20 · **Branch:** `feat/20-seat-kinematics` · **Date:** 2026-10-13

## Goal

Give the MIB firmware a pure, human-readable mapping between a requested **seat pose** and the
**actuator positions in degrees** that realise it, plus the inverse for reporting seat state from
feedback.

The chair has no dedicated seat actuators. Seat tilt, lateral tilt and elevation come from raising
and lowering the base's wheel legs; seat translation comes from the fore/aft carriages. This is the
same scheme the Demo-Software (Teensy) firmware used in `runSelfLeveling()`, but done here as
absolute, closed-form geometry instead of an IMU-error loop.

## Decisions (agreed)

- **Output is actuator degrees, not encoder counts.** Each actuator class owns its own
  degrees → counts mapping. It does not know about gear ratios or linkages; this module handles
  those for it.
- **Geometry comes from the URDF** in Demo-Software
  (`core/rammp_prototype_description/urdf/rammp_prototype_base.urdf`). Numbers that the URDF cannot
  supply are placeholders marked `TODO(bench)`.
- **Geometric IK only.** No IMU, no closed-loop leveling, flat ground assumed.
- **Front caster is not part of this module.**
- **Rear caster needs no linkage model.** Its encoder sits after the shock, on the swing arm, so the
  actuator is closed-loop on the arm angle directly.
- **Human readability is the top priority.** Named fields with units in every name, one explicit
  block of code per leg, a sketch and derivation in a comment above each formula. No templates, no
  `std::variant`, no index tables.

## Pipeline

```
SeatPose ──[SeatKinematics IK]──> JointConfig ──[Linkage]──> actuator input ──[Transmission]──> actuator degrees
pitch, roll,                      arm angles (deg),          main:  stroke (mm)                  one affine map
elevation, translation            carriage travel (mm)       rear:  arm angle (deg)              per actuator
                                                             carriage: travel (mm)
```

Every stage has an exact inverse, so feedback runs the pipeline backwards.

## Components and files

New component `components/SeatKinematics/`, no ESP-IDF dependencies in the math:

| File | Contents |
|---|---|
| `include/seat_kinematics.hpp` | `SeatPose`, `JointConfig`, `ActuatorPositions`, class `mib::SeatKinematics` |
| `include/linkage.hpp` | `mib::TriangleLinkage` (main-leg actuator ↔ arm angle) |
| `include/transmission.hpp` | `mib::Transmission` (actuator input ↔ actuator degrees) |
| `src/seat_kinematics.cpp` | IK / FK implementation |
| `example/` | ESP-IDF example project, `DifferentialDrive` style, exercising round trips |

Geometry constants live in `components/MIBConfig/include/MIBconfig.hpp` as one
`mib::config::SeatGeometryConfig` grouped per leg. `SeatKinematics` is added to the root
`CMakeLists.txt` `COMPONENTS` list so CI compiles it.

### Data types

```cpp
struct SeatPose {
  float pitch_deg;       // > 0: front of seat up (tilt back)
  float roll_deg;        // > 0: left side up
  float elevation_mm;    // chassis-frame origin height above flat ground
  float translation_mm;  // carriage travel from URDF zero, same both sides
};

struct JointConfig {
  float main_left_arm_deg;
  float main_right_arm_deg;
  float rear_caster_arm_deg;
  float left_carriage_mm;
  float right_carriage_mm;
};

struct ActuatorPositions {   // all in actuator degrees
  float main_left_deg;
  float main_right_deg;
  float rear_caster_deg;
  float left_carriage_deg;
  float right_carriage_deg;
};
```

Named fields, not the `Leg` enum from `feature/2-actuator-control`, so this branch stands alone on
`main`.

### `SeatKinematics` API

```cpp
std::optional<JointConfig>       inverse(const SeatPose &pose) const;
std::optional<SeatPose>          forward(const JointConfig &joints) const;
std::optional<ActuatorPositions> to_actuators(const JointConfig &joints) const;
std::optional<JointConfig>       from_actuators(const ActuatorPositions &actuators) const;
```

An empty `optional` means the request is unreachable or outside a limit. The module never clamps
silently; the caller decides what to do.

## Frames and conventions

- **Chassis frame** = URDF `mebot` link: x forward (toward front caster), y left, z up. Millimetres.
- **Swing arms** rotate about −y as in the URDF. q = 0 is the CAD pose. Rotating a point (x, z) in
  the arm frame by q:
  `x' = x·cos q − z·sin q`, `z' = x·sin q + z·cos q`.
  So positive q raises a wheel that sits forward of its pivot (main legs) and lowers a wheel that
  sits behind it (rear caster).
- Seat pose sign conventions are as in `SeatPose` above.

## Inverse kinematics (per leg, closed form)

1. **World-up seen from the chassis.** Height above ground of a chassis point P is
   `elevation + up · P`, where
   `up = (sin p, sin r·cos p, cos r·cos p)` with p = `pitch_deg`, r = `roll_deg`.
   Check: a point forward (+x) rises when p > 0 (front up); a point left (+y) rises when r > 0
   (left up). This is the bottom row of R = Rx(r)·Ry(−p).
2. **Wheel centre in the chassis frame:**
   `wheel(q) = pivot + rotate(arm_to_wheel, q)`.
   Main-leg pivots also shift by carriage travel along x.
3. **Wheel rests on the ground:**
   `elevation + up · wheel(q) = wheel_radius`.
4. Expanding gives `a·cos q + b·sin q = c`, solved as
   `q = atan2(b, a) ± acos(c / √(a² + b²))`. Pick the root inside the ±45° joint limit. If
   `|c| > √(a² + b²)` or neither root is in range → unreachable.
5. **Carriages:** `left_carriage_mm = right_carriage_mm = translation_mm`, checked against travel
   limits.

Documented simplifications:

- Contact point taken directly below the wheel centre. Error is r·(1 − cos roll): ≈ 0.25 mm at 3°
  roll on the 184 mm main wheels.
- The rear caster's two wheels share one swing arm; they are merged into a single contact at the
  midpoint (y = 0).
- Flat ground only.

## Forward kinematics

Given the three arm angles, compute the three wheel centres in the chassis frame. Each wheel gives
`elevation + up · wheel_i = radius_i`. Subtracting pairs eliminates elevation and leaves two linear
constraints on the unit vector `up`: a line intersected with the unit sphere, solved in closed form
(take the root with `up.z > 0`). Pitch and roll follow from `up`; elevation from any one equation.
Translation is the mean of the two carriage positions.

## Linkage (main legs only)

```
         B  pin on carriage plate (fixed in plate frame)
        /
  actuator, length L
      /
     A  pin on swing arm (rotates with q about pivot O)

  L(q)      = |B − rotate(A, q)|
  q(L)      = closed form from the same triangle (law of cosines)
  stroke_mm = L(q) − L_retracted
```

- A = URDF `motor_elevator_l` origin, in the swing-arm frame: (−103.72, 126.68) mm (x, z).
- B = URDF `motor_elevator_pivot_l` origin, relative to the swing-arm pivot: (298.85, 109.69) mm.
- URDF `motor_elevator_rod_link_l` is the other candidate for B and disagrees by up to ≈ 13 mm of
  stroke at the ends of the range. `TODO(bench)`: confirm the pin from CAD.
- Over ±45°, L spans ≈ 288–471 mm (≈ 184 mm stroke), consistent with the URDF rod joint's ±0.1 m.
- `L_retracted`: `TODO(bench)`. Placeholder = L(−45°).

Rear caster and carriages pass straight through.

## Transmission

```cpp
/// actuator_deg = deg_per_unit * input + offset_deg
struct Transmission {
  float deg_per_unit;   // sign encodes direction
  float offset_deg;

  /// Linear actuator: input in mm. deg_per_unit = 360 * gear_ratio / travel_per_rev_mm.
  static constexpr Transmission linear(float travel_per_rev_mm, float gear_ratio,
                                       bool inverted = false, float offset_deg = 0.0f);
  /// Rotary joint: input in degrees. deg_per_unit = gear_ratio.
  static constexpr Transmission rotary(float gear_ratio, bool inverted = false,
                                       float offset_deg = 0.0f);

  float to_actuator_deg(float input) const;
  float from_actuator_deg(float actuator_deg) const;
};
```

| Actuator | Input | Constructor | Values |
|---|---|---|---|
| Main L/R | stroke mm | `linear` | `TODO(bench)` placeholders |
| Rear caster | arm deg | `rotary(1.0)` | 1:1, encoder after the shock; sign/offset `TODO(bench)` |
| Carriages L/R | travel mm | `linear` | `TODO(bench)`; Demo-Software placeholder was 12000 ticks per 0.30 m |

## Geometry values (from URDF, millimetres)

| Item | Value | URDF source |
|---|---|---|
| Main-left swing pivot (chassis) | (166.82, 182.00, 113.42) | `dw_main_plate_l` + `motor_swing_arm_l` |
| Main-right swing pivot (chassis) | (166.82, −187.12, 113.42) | `dw_main_plate_r` + `motor_swing_arm_r` |
| Main pivot → wheel centre (arm frame) | (318.77, ±119.54, −13.80) | `drive_motor_*` + `drive_wheel_*` |
| Main wheel radius | 184.06 | `drive_wheel_l` collision cylinder |
| Rear caster swing pivot (chassis) | (−31.75, 0.11, 8.89) | `rear_caster_arm_mount` + `rear_caster_swing_arm` |
| Rear pivot → wheel centre (arm frame) | (−209.55, 0, 12.70) | `rear_casteer_wheel_l` / `rear_caster_wheel_r`, y averaged |
| Rear caster wheel radius | 101.07 | `rear_caster_wheel_r` collision cylinder |
| Swing-arm limits | ±45° | all three revolute joints |
| Carriage travel limits | ±400 | `dw_main_plate_*` prismatic limits; `TODO(bench)` (Demo-Software calibrated range was 0–300) |

The URDF is not perfectly left/right symmetric (≈ 5 mm in y); values are used as-is.

## Testing

Example project `components/SeatKinematics/example/` logs PASS/FAIL for:

- **Round trips:** pose → joints → pose, and joints → actuators → joints, within 0.01° / 0.01 mm,
  over a grid of reachable poses.
- **Known behaviour:** pure positive pitch lowers the rear contact relative to the main contacts
  (front up); pure positive roll extends the left main leg relative to the right; elevation change
  alone moves all three arms in the "lift" direction.
- **Rejection:** poses beyond the arm or carriage limits return empty.

Because the math has no ESP-IDF dependency, the same checks are also compiled and run on the host
during development.

## Out of scope

- Wiring into `MibSystem` / `DriveController` / `handle_seat_control_command`. Waits for
  `feature/2-actuator-control` to merge.
- IMU self-leveling, slopes, front caster, trajectory/rate limiting of seat moves.
- Encoder counts.
