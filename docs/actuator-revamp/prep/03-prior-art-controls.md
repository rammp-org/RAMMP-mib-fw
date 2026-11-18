# 03 — Prior art for the actuator controller

Survey date: 2026-11-17. Read-only survey of code we own or depend on, done so the redesigned
linear-actuator controller reuses existing code instead of rebuilding it.

Abbreviations used in paths:

- `ESPP` = `/Users/alxchunlin/code-stuff/espp/components` (HEAD `30c51bd7`, 2026-10-26)
- `MIB` = this worktree (`RAMMP-mib-fw`, branch off `main` @ `2b8f48e`)
- `EIL` = `/tmp/mcp-revamp/eileen/components/Actuator` (the first `mib::Actuator`, from
  `rammp-org/RAMMP-mib-fw` branch `feature/2-actuator-control`)
- `AD` = `/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw-espp143/components/pace-racer-board/atos-drive`
  (the most recent pace-racer tree, fw 0.4.10, espp 1.4.3)
- `PRB` = `.../pace-racer-fw-espp143/components/pace-racer-board`
- `MEB` = `/Users/alxchunlin/code-stuff/atdev/MeBot`
- `WP` = `/tmp/mcp-revamp/work-prior` (scratch clones: `ATOS`, `prototype-fw`, `dynopace`, `mibfw-actuator`)

Line numbers that I spot-checked are the important ones (espp, MIB, EIL, AD supervisor/stall,
MEBot3.0 levelling and PID). The rest came from survey passes over the other trees and should
be re-checked before anyone quotes them in a PR.

---

## 0. The fact that shapes everything else

**The MCP266 closes the position loop itself, and its CANopen interface is the bottleneck.** The
MIB does not command duty or current. It sends position setpoints, and the MCP's own PID tracks
them. Bench results already recorded (`EIL/DESIGN.md:264-350`, `WP/prototype-fw/MIB/README.md`
"Control notes", `ESPP/mcp266/include/mcp266.hpp:38-51`):

- On this firmware a CiA 402 profile-position target is a **step**. `0x6081` (profile velocity)
  is honoured only as a speed cap below the stored default speed. `0x6083/0x6084` (accel/decel)
  are accepted and then ignored. So a smooth move must be **streamed from the MIB as a sequence
  of setpoints**, which is what `EIL` does (`EIL/src/actuator.cpp:535-599`).
- The velocity and duty commands (manufacturer mirrors cmd 32/33/35/36, and `0x606B`) produce
  no motion. `0x6502` reads `0x700`, which advertises cyclic-synchronous velocity (CSV) and
  torque (CST). **CSV over an RPDO (`0x60FF`) has not been tried.** It is the one untested route
  to MIB-side velocity control.
- **Limit switches are not reported over CAN**: statusword bit 11 never sets, and the cmd-90
  flags read 0. An end stop shows up only as a stall: the count stops changing while a move is
  commanded. Wire the pins as *limit* inputs, not *home* inputs, so they block motion without
  latching an error.
- A loop left enabled at its target hunts. `EIL` releases the drive after arrival and re-moves
  if the resting count drifts past `tolerance`.
- Quick stop leaves the drive in a state the normal enable sequence cannot leave. Writing a new
  encoder count while the loop is enabled makes the loop chase the old target.
- `Ds402Drive::set_target_position()` costs at least 5 SDOs per setpoint: write `0x607A`, raise
  controlword bit 4, poll statusword for bit 12, drop bit 4, poll until bit 12 clears. Each poll
  that misses sleeps `poll_period` (25 ms by default) (`ESPP/canopen/include/ds402.hpp:312-371`).
  With 6 legs on 3 nodes over SDO, the achievable setpoint rate per axis is a few tens of Hz at
  best. **Measure it before choosing a loop rate.** PDO mapping (`CanopenClient::send_rpdo` /
  `register_tpdo_callback`, `ESPP/canopen/include/canopen_client.hpp:202-224`) is the
  configuration-level fix, but nobody has tested whether the MCP266 accepts PDO remapping.

So the MIB-side "controller" is mostly a **setpoint generator plus a supervisor**: profile, limits,
sync, stall/fault detection, levelling outer loop. The inner servo is the MCP's PID, tuned
through its manufacturer objects.

---

## 1. Reuse map

Marks: **as-is** (use unchanged), **configure**, **wrap** (thin adapter), **upstream** (small fix in
espp), **none** (no implementation exists).

| Building block | Best existing implementation | Key config / API | Known gaps | Mark |
|---|---|---|---|---|
| CANopen transport + DS402 | `espp::CanopenClient`, `espp::Ds402Drive` (`ESPP/canopen/include/canopen_client.hpp`, `ds402.hpp`) | Client `Config{node_id, send, sdo_timeout=100ms, on_heartbeat}`. Drive `Config{state_timeout, poll_period=25ms, object_offset}` | SDO-only setpoint path is slow (§0). PDO helpers exist but no mapping helper | as-is, then configure PDOs |
| MCP266 driver | `espp::Mcp266` (`ESPP/mcp266/include/mcp266.hpp`) | `Config{state_timeout=1s, poll_period=25ms}`. `configure_position_loop(axis,min,max,fallback_p)` :154. `set_position_pid(p,i,d,max_i,deadzone,min,max)` :248. `set_software_position_limits` :314. `move_to_position` :337. `read_encoder/read_speed/read_statusword/is_target_reached` :396-489. `read_main_battery_voltage/read_temperature` :500-514. `set_encoder/reset_encoders` :412-440 | **No motor-current read.** Packet-serial cmd 49 (`ReadMotorCurrents`, 10 mA units, `ESPP/motor_controller/include/basicmicro_commands.hpp:71`) is mirrored at `0x2031` the same way battery and temperature are. Adding it is a ~15-line upstream change. No stored default speed/accel setter (cmds 68-71); `EIL` writes those itself | as-is + **upstream** (`read_currents`) |
| Actuator axis in counts (enable/release, move, stream, home, persist) | `mib::Actuator` (`EIL/include/actuator.hpp`, `EIL/src/actuator.cpp`) | `Profile{velocity, accel, decel}` :71-74. `Homing{stall_counts=2, stall_time=500ms, timeout=60s, segment=50000}` :78-90. Per-leg table in `MIBconfig.hpp` on that branch (nodes 1/2/3; range 0-4095 abs-encoder legs, 0-20000 carriages; vel/acc 500; tolerance 200) | Hand-rolled trapezoid (`actuator.cpp:535-599`). Polled over SDO from `poll()`. No current, no IMU, no multi-axis sync, no supervisor state machine. Not wired into `MibSystem` (only `feat/home-position` starts that) | **wrap / refactor**: keep its CANopen sequencing and bench workarounds |
| Motion profile (position target, v/a limits, optionally jerk) | `espp::detail::trajectory_planner::update_accel()` / `step_axis()` (`ESPP/trajectory_planner/include/detail/trajectory_planner_math.hpp:51-140`), host-tested | `update_accel(a, dist, step, a_max, dt)` is a discrete **time-optimal double-integrator tracker that lands on the target without overshoot**. Map it one order down: `a`→velocity, `dist`→position error, `step`→accel·dt, `a_max`→v_max, and you get a trapezoidal **position** profile. `step_axis` adds jerk (S-curve) for velocity | It lives in `detail::`, which is not public API. The public `espp::TrajectoryPlanner` (`trajectory_planner.hpp:64-295`) is velocity-target, 2-axis (v, w) and owns its own timer threads, so it cannot take a position target. Retargeting mid-move is handled (it is a tracker, not a precomputed plan) | **wrap** (≈40 lines); upstream a public 1-D `PositionProfile` later if wanted |
| Alternative profile / smoother | `espp::HanTrackingDifferentiator` (`ESPP/adrc/include/adrc.hpp:102-110`) | `Config{tracking_bandwidth r (≈accel bound), filter_factor=5}`. Outputs smoothed position and its rate | No velocity cap, so it cannot be a travel-speed limiter alone | configure (secondary) |
| Position/velocity PID with anti-windup | **Inner loop: the MCP266's own position PID**, set via `Mcp266::set_position_pid`. MIB-side: `espp::Pid` (`ESPP/pid/include/pid.hpp:22-167`) | `Pid::Config{kp, ki, kd, integrator_min/max, output_min/max}`. Anti-windup undoes the integration step when output saturates (:116-119). dt is measured from `esp_timer` | `espp::Pid` has no feedforward term and no derivative-on-measurement. A feedforward added outside the PID is invisible to its saturation check. MCP PID: fixed-point ×1024, field-order quirk handled by espp | **configure** (MCP gains); `espp::Pid` **as-is** for any MIB outer loop (e.g. levelling) |
| Filtering / differentiation | `espp::SimpleLowpassFilter{time_constant}`, `LowpassFilter{normalized_cutoff, q}`, `ButterworthFilter<N>{normalized_cutoff}`, `KalmanFilter<N>`, `HanTrackingDifferentiator` (rate estimate) (`ESPP/filters/include/*.hpp`) | — | MCP already reports velocity (`0x606C`), so differentiation of counts is only needed for the IMU/levelling side | as-is |
| Gravity / attitude from IMU | **BNO055 on-chip fusion** via `espp::Bno055` (`ESPP/bno055/include/bno055.hpp`). `Config{mode=IMU (0x08) default, NDOF, ...}`. Data: `euler`, `gravity_mps2`, `linear_accel_mps2`, `calibration` (:57-95). Already read at 100 Hz on main (`MIB/components/MibSystem/src/mib_system.cpp:908-935`, `938-980`) | Fallback if raw is preferred: `espp::ComplementaryFilter(alpha=0.98)` and `espp::MadgwickFilter(beta=0.1)` (`ESPP/filters/include/complementary_filter.hpp:22-72`, `madgwick_filter.hpp:15-241`) | Main only **logs** the IMU at 2 Hz. Nothing consumes pitch/roll. Mounting-offset calibration does not exist (MEBot3.0 hard-coded −5.0°/+1.5°, see §3) | as-is (read) / **none** (consumer) |
| Seat-pose → leg-height kinematics | `SeatKinematics` on `feat/20-seat-kinematics`, open PR #28 (`seat_geometry.hpp`, `transmission.hpp`, host tests). Pose (pitch, roll, elevation, translation) → ML, MR, RC, carriages. Analogue: `linkage_geometry.hpp` on prototype-fw `feat/pear` (PR #4) | — | Waiting on the actuator branch. Check whether its geometry matches the current chair (MEBot3.0 numbers in §3) | as-is once merged |
| Multi-axis synchronisation | none. MEBot3.0 runs each axis independently and sequences on AND conditions (§3). `EIL` is per-axis | — | Time-scaling all axes so they finish together is new. It is small on top of the profile wrapper: scale each axis's v/a by its distance ratio | **none** |
| Homing | `Actuator::home()` (`EIL/src/actuator.cpp:937`): drive toward the limit and call it home once the count stops changing (`stall_counts`/`stall_time`). Calibration mode and NVS `save_position/restore_position` exist | `Homing{...}` above | Not verified against a real switch (`EIL/DESIGN.md:301-304`). No current-based stall (no current read) | as-is → verify on chair |
| Soft limits | MCP266 MinPos/MaxPos clamp (`configure_position_loop`) **and** CiA 402 `0x607D` (`set_software_position_limits`). `Actuator::set_range` above both | — | MCP reverts to EEPROM on power-up: re-apply every boot. `EIL` already does | configure |
| Stall / overcurrent detection | MCP266 internal current limit (set in Motion Studio; it faults the drive). Pattern: `AD/main/atos_drive.cpp:1015-1046`: "wants motion **and** integrator ≥ ½ cap **and** net motion ≤ 2 steps over 0.5 s → stall fault". I²t budget with fold-back `AD/main/drive_supervisor.hpp:372-389`. `EIL` homing uses count-stall only | — | On the MIB, current is invisible until `Mcp266::read_currents` exists. Then: count-stall **and** current-high → stall | **wrap** (pattern) + upstream |
| Fault/supervisor state machine | `atos-drive` `supervisor::Supervisor` + `Hooks` (`AD/main/drive_supervisor.hpp`): DISARMED/ARMED/SAFE_STOPPING/HOLDING/FAULT, first fault wins, clear by new `clear_fault_req_id`, host-tested against a fake plant (`AD/tests/host_supervisor/`). espp `state_machine` exists (HFSM, `ESPP/state_machine/include/state_base.hpp`) but nobody in our repos uses it | Supervisor `Measured` struct :57-79, `Hooks` :81-108 | It is board-local code, not a library. Copy the shape; the states map well to an actuator | **wrap** (port the pattern) |
| Comm watchdog | MIB: `motor_jog_timeout 300 ms`, `twist_setpoint_timeout 300 ms`, `motor_state_timeout 500 ms` (`MIB/components/MIBConfig/include/MIBconfig.hpp:51,87,90`). CANopen heartbeat callback (`canopen_client.hpp:60-71`). `AD`: 200 ms → safe stop → hold → latched `FAULT(WATCHDOG)` (`drive_supervisor.hpp:215-225`) | — | Nothing watches the MCPs today. Use the MCP heartbeat (or failed SDOs, as `EIL::check_online` does) | configure |
| Persistence | `EIL` `ActuatorStore` (raw NVS; position saved at most every 20 counts / 5 s). `AD/main/axis_config.hpp:39-150` (one NVS namespace, versioned `tune_v` key, writes deferred to a 10 Hz task, never from the control tick). `espp::PersistentData` (`ESPP/persistent_data/include/persistent_data.hpp:78-188`) is file-based and needs a filesystem partition; MIB has only `nvs` (`MIB/partitions.csv:4`) | — | — | as-is (`ActuatorStore`) + copy the `AD` versioning rule |
| Telemetry / logging | MIB `ServiceLink` → `espp::Telemetry` named float channels at 20 ms (`MIB/components/ServiceLink/include/service_link.hpp:38,67,101`; channels `MIB/components/MibSystem/src/mib_panel.cpp:102-110`; `emit_telemetry` :474). RTPS `MibStatus.current_seat_state` (`mib_system.cpp:819`) and `Motors` debug list (:1138). `AD` capture ring `cap/capdump` for burst capture (`AD/main/atos_drive.cpp:466-475, 2509-2537`) | — | Telemetry already has `seat.*` channels (targets only). `publish_seat_state()` is a stub (`mib_system.cpp:851`). No burst capture on MIB | as-is + add channels |
| Tuning UI | `tools/mib_panel` (live uPlot plots, CSV export, a seat-target page, and drive-motor auto-tune; `MIB/tools/mib_panel/README.md:3-78`). `espp::Mcp266Service` + `mcp266_console.html` (dispatcher module 6: configure the position loop, command moves, stream STATUS; `ESPP/mcp266/include/mcp266_service.hpp:44-183`, `ESPP/mcp266/web/mcp266_console.html`). `espp` `telemetry/web/telemetry.html` | `Mcp266Service::Config{send, module=6, mcp_mutex, default/min/max_stream_period_ms=200/50/10000}` | `Mcp266Service` registers straight onto `ServiceLink::register_module`. It is the cheapest way to get an MCP266 tuning page through the existing panel transport. Share `mcp_mutex` with the controller | **as-is** (register it) |

---

## 2. How ATOS and pace-racer structure motor control

### ATOS (`WP/ATOS`, ATDev-Inc/ATOS HEAD `fe37e55`; PRD at `/Users/alxchunlin/code-stuff/pace/ATOS PRD V1.md`)

ATOS contains **no drive code yet**: `main/main.cpp` is 39 lines, and the only dependency is
`espp/task`. Its `experiments/` folder is about runtime module loading, not control. What it does
contribute is a contract:

- Layer 1 is "the embedded, deterministic motor-control node … owns the real-time drive loop"
  (`ATOS/README.md:14`).
- "Safe-state default (drive disabled unless actively held safe), watchdogs, fault containment and
  unload-on-fault, and hard limits the core enforces and a module can never override"
  (`README.md:47`). Modules sit "behind a validated, clamped setpoint interface" (`README.md:121`).
- The PRD adds drive-over-auxiliary priority with load shedding, and deterministic-domain
  actuators kept apart from Linux (PRD lines 169, 200-201, 257-258).

The lead's "atos-drive" is the implementation of that contract in the pace-racer tree (`AD`).

### pace-racer / `atos-drive` (`AD`)

| Layer | Rate / placement | Where |
|---|---|---|
| ADC ISR + integer trip check | 20 kHz, IRAM | `AD/main/foc_sampler.hpp:88-138`, `trip_gate.hpp` |
| FOC task: current PI, speed PI, position P(D), governor, stall/hall guards | 20 kHz, priority 20, CPU1 | `AD/main/atos_drive.cpp:873-1049` |
| **Supervisor**: states, watchdog, limits, heat budget, tuning | **100 Hz**, `espp::Task` with absolute deadline, priority 6, CPU0 | `atos_drive.cpp:1593-1612`, `drive_supervisor.hpp:161` |
| `MotorState` publish (RTPS) | 50 Hz | `motor_params.hpp:230` |
| CSV stream, temps, deferred NVS writes | 10 Hz | `atos_drive.cpp:1643-1680` |

Patterns worth copying, in priority order:

1. **Supervisor plus Hooks, host-tested against a fake plant** (`AD/main/drive_supervisor.hpp:1-20,
   57-108`; tests in `AD/tests/host_supervisor/`). Receive callbacks only store the message under
   a mutex, and `tick()` makes every decision. For the actuator, the hooks become: `enable`,
   `release`, `stream_setpoint`, `stop`, `measure` (counts, velocity, statusword, current,
   battery). The MIB repo has no host tests today. This would be the first, and the pattern is
   already proven.
2. **Watchdog escalation** (`drive_supervisor.hpp:215-225`): no command for 200 ms → safe stop →
   hold → latched `FAULT(WATCHDOG)`. A DISARMED board is never energised by the watchdog. For a
   leg the analogue of "hold" is "stop streaming, leave the MCP holding position". That keeps
   the chair from dropping.
3. **Limit semantics** (`drive_supervisor.hpp:391-421`): compiled ceilings per profile. A
   command field of 0 means "use the ceiling", a command can only lower a limit, and the
   effective limits are echoed back in state.
4. **First fault wins; the low level already acted.** The MCP's own current limit and fault stop
   react first. The supervisor only names and latches the fault, and clearing it needs a *new*
   request id, not a level.
5. **Stall rule** (`atos_drive.cpp:1015-1046`, constants :559-560): wants motion, pushing hard,
   and ≤2 counts of net motion in 0.5 s. Exempt the modes where pushing on a stop is the intent.
   For homing this inverts into the success condition.
6. **Rest hold / quantised-feedback deadband** (`AD/main/rest_hold.hpp`; `atos_drive.cpp:909-913,
   939-957`): inside ±1 count, bleed the integrator instead of hunting. This is the same
   symptom `EIL` hit ("an arrived loop hunts") and solved by releasing the drive.
7. **Live tuning message plus NVS rules** (`AD/main/axis_config.hpp:39-150`;
   `atos_drive.cpp:1578-1585, 1655-1662`): apply only on a new `req_id`, clamp to bounds, persist
   from a slow task, and version the stored tune. The MIB already relays `MotorTuning` for the
   drive motors (`mib_system.cpp:1122-1134`).
8. **Capture ring + host fitting script** (§5).

What it does *not* reuse from espp: `espp::Pid`, `state_machine`, `trajectory_planner`,
`persistent_data` and `cli` are all hand-rolled or unused there (its `idf_component.yml` lists them,
but the code does not call them). Do not read its choices as evidence against those components.
They were made for 20 kHz ISR-adjacent code.

---

## 3. Legacy MEBot actuator control

**Caveat first:** only **MEBot 3.0** (Teensy 3.6, C++) drives its height axes with DC motors, and
those are **EHAs** (electro-hydraulic actuators: a DC motor driving a pump), plus two DC carriages
for drive-wheel fore/aft. Cobra (2016), the 2017 and 2018 Raspberry Pi code and MEBot 2.5 all use
**pneumatic cylinders on PWM valves**. Their sequencing, geometry and levelling maths carry over;
their low-level control does not. No generation has current sensing, stall detection or homing,
and none uses RoboClaw or MCP controllers. **Confirm with the hardware lead whether today's
chair still has the MEBot3.0 EHAs on these axes or new linear actuators.** It changes the
plant (EHA strokes are compliant and drift under load).

### MEBot 3.0 (`MEB/MEBot3.0/Base/Base.ino` + `lib/*`; last commit Dec 2025 "curb climbing updated")

- **Drive:** PWM + DIR per motor, 20 kHz, 8-bit 0-255 (`lib/Component/Components.cpp:9, 28-37`).
- **Sensors:** 12 quadrature encoders, low-pass `K=0.1` (`lib/EncoderContainer/*`). Load cells per
  axis through INA118 amplifiers, LPF 0.03. **Carriage limit switches only** (`Constants.h:28-31`).
  BNO055 IMU.
- **Position conversions (cm)** (`lib/Wheel/Wheel.cpp:12-20`, `Base.ino:324-401`, `lib/Carriage/Carriage.cpp:31`):

  | Axis | Formula | Range |
  |---|---|---|
  | ML/MR leg height | `2.5 + 33·sin(enc·33/pos_ticks/DG)`, pos_ticks −390 (ML), 380 (MR) | 2.5-21 |
  | ML/MR EHA stroke | `|enc|/39.4` (ML), `/35.47` (MR) | 0-10.15 |
  | RC | `enc·(−18.4/680)` | 0-18.4 |
  | FC | `1.0 + enc·0.031` (stroke `/83.6` "NOT WORKING PROPERLY") | 2.5-22.5 |
  | Carriage | `34.3·enc/ticks`, ticks 12000 (ML), −12531 (MR) | 13.5 in travel |

  Geometry: length 68 cm, drive-wheel track 62 cm, RC width 22 cm (`Base.ino:114-121`).
- **Control law** (`lib/MotorController/MotorController.cpp:367-539`, verified at :367-436;
  `Wheel.cpp:26-84`; `Carriage.cpp:34-50`): magnitude-only PI(D) with a fixed offset:
  `PWM = Kacc · |20 + Kp|e| + Ki·∫|e|dt + Kd·|Δe|/dt|`, direction from the sign of e. The integral
  accumulates |e|, so it only ever grows until the deadband resets it.
  - Gains: RC and FC Kp 25 Ki 2 (were 20/1.8 and 25/1.5); ML and MR Kp 25 Ki 1; carriage Kp 15
    Ki 1; Kd 0.8 default (`Component.h:20`).
  - Deadband ±0.2 cm (zeroes the integral and `Kacc`).
  - **Soft start**: `Kacc` += 0.01 per loop up to 1.0 (0.003 in proportional mode), reset to 0
    whenever the setpoint changes. That is a ~1 s ramp-in of gain, not a motion profile.
  - Stroke soft limits: ML/MR stop at ≤0.3 or ≥9.8 cm stroke, FC at ≤0.5 or ≥9.9 (RC's limits
    are commented out). Setpoint clamps: ML/MR 3-21, RC 1-21, FC 2-15 cm.
  - Defects: FC `Kacc` is never capped; the FC upper clamp tests the wrong variable (:51); FC's
    in-deadband branch doesn't zero PWM.
- **Loop rate:** `delay(5)` per loop (`Base.ino:313-314`), effectively ~80 Hz (the levelling
  init assumes "80*5 = 5 s", :967-969). The GUI logs at 100 Hz; logged `time` steps at 0.02 s.
- **Self-levelling** (`Base.ino:922-1061`, verified at :922-995):
  - BNO055 Euler angles with **hard-coded mounting offsets** pitch = `euler.z − 5.0`, roll =
    `euler.y + 1.5`, LPF K = 0.08 (`lib/IMU_Class/IMU_Class.cpp:32-37`).
  - Frame angle from the leg heights: `pneuroll = atan2(MR−ML, 62)`, `pneupitch =
    atan2((ML+MR)/2 − RC, 68)` (:428-429).
  - Error = frame angle − IMU angle, with a 0.001 rad deadband. A rotation matrix (nominal height
    9.5 cm, "was 10") times the base corner matrix gives the ML, RC and MR setpoints. **It is a
    kinematic one-shot correction with gain 1, not a tuned feedback loop.**
  - For the first 400 loops (~5 s) it applies continuously. After that it **holds** the previous
    setpoints unless pitch error ≥ 6° or |roll error| ≥ 2° (hysteresis gate). FC is parked at
    1.0 cm and the carriages at 0.1 during levelling.
- **Curb climb** (`Base.ino:817-868, 1195-1333`). The user drives; the firmware sequences the
  actuators (cm):
  1. FC → 2.0 (hold 200 loops), carriages → 0.1, RC → 8.0, ML/MR → 20.0. Advance when ML and MR > 19.
  2. Hold ML/MR at 20.5, FC 2.0. Advance when either load cell changes by > 20 counts (wheel on curb, :1203).
  3. Carriages → 30, RC → 21. Advance when both carriages > 29.5.
  4. ML/MR → 3.0, FC → 4.0. Advance when both < 4.0.
  5. Carriages → 13, FC → 6. Advance when both < 13.5.
  6. Carriages → 31, FC → 1. Advance when both > 30.
  7. RC → 3.0, wheels → 3.5. Advance when RC < 4.0.
  8. Once MR has travelled 15 cm, carriages → 0.5. Advance when both < 1.0.
  9. FC → 0 for 200 loops, reset.
- **Synchronisation:** none. Each axis has its own loop. Steps advance on AND conditions across axes.
- **Tools:** `MEB/MEBot3.0/GUI/` (PyQt, serial commands, 100 Hz CSV logger with columns pitch,
  roll, FC/RC/MR/ML position, load cells, wheel position, `CA_flag`). Sample logs are in
  `MEB/MEBot3.0/Dev_Tools/data_logger/` (Aug 2025). No plotting, system-ID or tuning scripts;
  gains were hand-tuned (the "was X" comments).

### Earlier generations (pneumatic; useful for geometry and sequencing only)

- **MEBot 2.5** (`MEB/MEBot2.5_Black&Gold/MEBot2.5_two_teensy/teensy_base_2.5.ino`): pots mapped to
  inches (FL/FR −2.0-9.125, ML/MR 1.125-8.5, RL/RR 1.125-9.5, :508-517). `pneucontrol` has a
  ±0.3 in deadband with rod-speed gating (:1138-1210). Self-levelling with a 1.5° (0.0262 rad)
  deadband and 4.5 in height offset (:806-879). RealSense-assisted curb ascent and descent are
  in `sivaMEBot/sivaMEBot_current.ino:1324-1640`.
- **2018 Pi/Python** (`MEB/MEBot codes 2018/MEBotBG_2018/mebot2018_latest.py`): ±0.5 in deadband,
  1.5° levelling threshold (:38, 700-703), hard-coded 8 in curb (:731), CSV logging (:1137-1146).
- **2017** (`MEB/MEBot codes 2017/MEBot Projects_BlackGold/blackgoldGUIpneumatics2.py`): `PD()`
  with Kp 10 and a 0.15 deadband (:109-121).
- **Cobra 2016** (`MEB/Cobra/CobraJuly2016/demo.c`): 35 Hz loop (`params.h:14`). Complementary
  filter 0.98/0.02 of gyro + accel fused with serial inclinometers (`demo.c:170-238`). Pressure
  drop ≥ 10 detects the curb edge on descent (:1129). Drive encoder 15220 ticks/rev; carriage
  12131 ticks per 32.5 cm (`params.h:100-105`).

**What it tells us:** levelling worked as a slow, gated, kinematic correction (5 s of continuous
correction, then only above 6° pitch / 2° roll error). It did not need a fast attitude loop. Every
generation used a position deadband of 0.15-0.5 (cm or in) and a soft-start rather than a
planned profile. Mounting offsets were hand-entered constants, and nothing ever detected end
stops or stalls.

---

## 4. How the actuator controller should plug into mib-fw (main @ `2b8f48e`)

What exists today:

- **Tasks** (`MIB/components/MibSystem/src/mib_system.cpp:140-216`; periods in
  `MIB/components/MIBConfig/include/MIBconfig.hpp:12-26`): state machine 20 ms; motor commands
  50 ms (priority 2); odometry 50 ms; publication 500 ms; IMU 10 ms with a fixed-period
  `wait_until` schedule that skips missed periods (`mib_system.cpp:915-935`); panel telemetry
  20 ms.
- **Drive path:** `DriveController` owns an `espp::TrajectoryPlanner` (profiles LOW/NORMAL/HIGH,
  `MIB/components/DriveController/src/drive_controller.cpp:66-99`), and
  `publish_motor_commands()` turns `wheel_speeds()` into RTPS `MotorCommand` at 50 Hz, ARMED only
  in DRIVE_ENABLED/AUTONOMOUS/SHARED/PUSH_ASSIST and only while the e-stop latch is clear
  (`mib_system.cpp:1042-1136`).
- **Seat path today:** RTPS `SeatCommand{axis, target}` (rammp-rtps
  `joystick_message.hpp:250-253`; axes FRONT_BACK_TILT, LATERAL_TILT, ELEVATION, TRANSLATION)
  → `handle_seat_control_command()` accepts it **only in IDLE** (`mib_system.cpp:619-645`) →
  `DriveController::set_seat_target()` is a log-only placeholder (`drive_controller.cpp:106-108`).
  The panel uses the same handler (`mib_panel.cpp:344`). `seat_state_` is echoed in `MibStatus`
  (:819), and `publish_seat_state()` is a stub (:851).
- **E-stop:** `latch_.is_active()` is checked every 20 ms state step (`mib_system.cpp:1256`) and as
  a backstop in the motor-command task (:1050). `enter_estop()` (:1369-1390) calls
  `drive_controller_.stop()` and `arbiter_.estop()`.
- **Arbitration:** only `TwistSetpoint` carries a token (`arbiter_.run_if_admitted`, :285);
  `on_arbiter_halt()` (:521) stops the drive. `SeatCommand` has no token.
- **No CAN on main:** no `twai`/`canopen`/`mcp266` dependency in any `idf_component.yml`. The TWAI
  setup to copy is `WP/prototype-fw/MIB/main/main.cpp:212-267` (TX 17, RX 16, 1 Mbit/s,
  `CanopenClient` with 500 ms SDO timeout, then `Mcp266`). `EIL`'s branch has the MIB version of it.

Recommended fit:

1. **A new component `ActuatorController`** (sibling of `DriveController`, not inside it). It
   owns the TWAI bus, one `CanopenClient` + `Mcp266` per node (1/2/3), one mutex per node (shared
   with `Mcp266Service`), and one axis object per leg built from `EIL`'s `Actuator`.
2. **One control task with an absolute-deadline schedule**, copying the IMU task's
   `wait_until` pattern (`mib_system.cpp:919-928`) or the `AD` supervisor task. **Start at 50 Hz**
   (matches `motor_command_task_interval`, the legacy MEBot logging rate and the planner's
   default). Only go to 100 Hz once the measured SDO setpoint cost (§0) allows six axes within
   the period. If SDO cannot keep up, map RPDOs before raising the rate. Per tick: read feedback,
   advance each axis's profile, stream setpoints, run the supervisor (stall, watchdog, limits).
3. **Command entry:** route `handle_seat_control_command()` to the new component instead of
   `set_seat_target()`, through `SeatKinematics` (PR #28) for pose → leg targets. Keep the IDLE
   gate. Decide separately whether seat moves need a state of their own (e.g. a SEAT_MOVE or
   CALIBRATE state; `enter_calibrate()` at :1392 is the placeholder for homing).
4. **E-stop:** add `actuators_.stop()` next to `drive_controller_.stop()` in `enter_estop()`
   (:1378), and gate streaming on `latch_.is_active()` inside the tick, as
   `publish_motor_commands()` does. "Stop" for a leg means **stop streaming and hold at the
   current count** (a CiA 402 move to the current position, which `EIL` found is the only stop
   that also halts the `0x2041` path). Never use quick stop, which parks the drive.
5. **Arbitration:** if the Jetson will command seat moves, give `SeatCommand` (or a new
   tokened message) the `run_if_admitted` treatment and add a stop in `on_arbiter_halt()`. This
   needs a rammp-rtps change and is out of scope for the controller itself.
6. **Watchdog:** MCP heartbeats via `CanopenClient::Config::on_heartbeat`, plus `EIL`'s
   failed-SDO online check. A lost node → stop the other axes on the same move group.
7. **Telemetry:** extend `kMibChannels`/`kAxisChannels` (`mib_panel.cpp:102-110`) with per-leg
   `target, setpoint, pos, vel, current, state`, plus `imu.pitch, imu.roll`. Fill `MibStatus`
   seat state from measured positions. Register `espp::Mcp266Service` on `ServiceLink` for the
   MCP tuning page.

---

## 5. Existing sysid and tuning tooling

| Tool | Where | What it does | Reuse for the actuator campaign |
|---|---|---|---|
| `tune.py` | `PRB/current-control/tune.py` (`fit_rl` :77) | Open-loop voltage step → fit R, L → kp = L·ωc, ki = R·ωc → iterate closed-loop steps until overshoot ≤ 2 %. CSV/PNG to `tune_logs/` | Fit-then-verify loop structure; swap the plant model for a first-order velocity plant or integrator + lag |
| `fig1_tune.py`, `fig1_rl.py`, `bench_lib.py` | `PRB/halls-sensorless/bench/` | J from a torque step; speed pole placement (K = Kt/J, kp = 2ζωn/K, ki = ωn²/K at several ωn); position P(D) progression; scores overshoot, t90, steady state into JSON. `bench_lib.Board` wraps `cap`/`capdump` | **Closest match.** Same "estimate plant → place poles at several bandwidths → step-test → score" flow |
| `bench_report.py`, `compile_report.py` | `PRB/data-reports/` | JSON → self-contained HTML figure report | Report generation |
| `characterize.html` | `/Users/alxchunlin/code-stuff/test-bench/web/characterize.html` (steps :489-498, least-squares fit :390) | WebUSB end-to-end motor characterisation and pole placement | UI pattern |
| Capture ring | `AD/main/atos_drive.cpp:466-475, 2509-2537` | `cap <every> <n> <mode>` fills a 3000×3 float ring; `capdump` prints rows | Port as a MIB-side ring at the control-tick rate (setpoint, count, current), dumped over `ServiceLink` |
| mib_panel auto-tune + CSV export | `MIB/tools/mib_panel/README.md:59-78`, `panel.js` | Drive-motor speed-loop pole placement on the chair; plots any telemetry channel; CSV export | **Already in this repo.** Add an actuator page with step tests |
| dynopace step / staircase | `WP/dynopace/main/main.cpp:346-480`, `web/dashboard.html:158-437`, README "Tuning" :167 | `k` step test streams CSV between `#step begin … gains` and `#step end`; `w` open-loop staircase; `u` smooth ramp; `g`/`l` live gains and slew | The CSV framing and the "slew, not gains, limited rise time" lesson |
| `mcp266_console.html` + `Mcp266Service` | `ESPP/mcp266/web/`, `ESPP/mcp266/include/mcp266_service.hpp` | Configure the position loop, command moves, stream STATUS (up to 20 Hz) | MCP PID tuning over the existing panel transport |
| `telemetry.html` | `ESPP/telemetry/web/` | Serial plotter for `espp::Telemetry` | Generic live plotting |
| `mib_debugger` | `rammp-org/RAMMP-mib-fw` branch `feature/21-mib-debugger`: `tools/mib_debugger/actuator_ctl.py`, `mcp266_objects.py`, `canopen_sdo.py`, `fake_mib.py` | Python port of `mib::Actuator` over a raw CAN bridge | Host-side identification with no firmware changes: step the MCP from Python and log counts at SDO rate |
| MEBot3.0 GUI logger | `MEB/MEBot3.0/GUI/`, `Dev_Tools/data_logger/` | 100 Hz CSV logs (pitch, roll, heights, load cells) | Reference data: actual levelling and curb-climb traces from 2025 |
| Motion Studio autotune | vendor tool | MCP velocity then position autotune (bench QPPS 2750, `EIL/DESIGN.md:296-300`) | Baseline gains to beat |
| kinova-gen3-sysid | `rammp-org/kinova-gen3-sysid` (`collection.py`, `fit_compensation.py`, `impedance_sweep.py`) | Arm sysid harness | Fitting code style only |

There is no chirp/frequency-response tool anywhere. Everything is step-based.

---

## 6. Gaps: what does not exist and would be new code

1. **A public 1-D position-target profile.** The maths exists (`update_accel` in espp `detail::`).
   What is missing is a ~40-line wrapper that steps (position, velocity) toward a target with v/a
   limits. Replace `EIL`'s hand-rolled trapezoid with it. Optionally upstream it as
   `espp::PositionProfile`.
2. **Motor current over CANopen** in `espp::Mcp266` (`read_currents` via mirrored cmd 49 at
   `0x2031`). Small upstream PR, same shape as `read_temperature`.
3. **A fast setpoint path.** Either RPDO mapping for target position + controlword and TPDO for
   position/status, or CSV mode on `0x60FF`. Neither has been tried on the MCP266. This is a
   bench experiment before it is code.
4. **Multi-axis synchronisation**: scaling all legs in a move so they arrive together. Small and
   new. No prior generation did it.
5. **IMU consumer**: mounting-offset calibration (MEBot hard-coded −5.0°/+1.5°), pitch/roll
   filtering, and the levelling outer loop (MEBot3.0's gated kinematic correction is the
   reference design; `espp::Pid` if it becomes a true feedback loop).
6. **Actuator supervisor state machine** with stall (count + current), watchdog, fault latch and
   clear-by-request-id. The pattern is `AD`'s `Supervisor` + `Hooks`; the code is new for
   CANopen legs. It is also the first host-testable component in mib-fw.
7. **MibSystem integration**: CAN bring-up on main, routing seat commands, e-stop/arbiter stop
   hooks, telemetry channels, seat state publishing.
8. **Actuator step-test capture and fitting**: a MIB capture ring plus a `fig1_tune.py`-style
   script for the MCP position loop. The pieces exist in pace-racer and dynopace; the actuator
   version does not.

Things that look missing but are not: the MCP driver, homing-by-stall, NVS position memory, soft
limits, BNO055 reading, seat kinematics (PR #28), panel telemetry and plotting, and MCP tuning UI
(`Mcp266Service`).
