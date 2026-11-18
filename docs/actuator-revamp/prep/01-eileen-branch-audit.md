# Audit: `feature/2-actuator-control` (Actuator component, tip 665a303)

Scope: the `Actuator` component, its example console, and the BSP/MIBconfig
changes that wire it in, on Eileen's branch `feature/2-actuator-control`
(33 commits touching the component, 2026-09-27 to 2026-11-17). Read against
espp `mcp266` / `canopen` (local checkout v1.3.10+) and espp PR #910 (open).

Paths below are relative to the read-only extract at `/tmp/mcp-revamp/eileen/`
unless they start with `espp:` (= `/Users/alxchunlin/code-stuff/espp/components/`).
Commit hashes are on `origin/feature/2-actuator-control`.

The standard applied throughout: *would this conclusion survive a log file?*
The branch contains a lot of careful engineering (state-machine handling,
offline marking, reset detection, a useful bench console) and a lot of real
bench time. What it does not contain is a single recorded measurement: no
logs, CSVs, traces or plots exist anywhere on the branch. Every behavioural
claim below rests on a console readback, something seen or heard on the bench,
or an inference. That is the gap the revamp should close, not a judgement on
the effort.

---

## 0. Summary of the most consequential findings

1. **The motion design was reversed four times on one question, by eye.**
   "Does the MCP266 apply a ramp to a CiA 402 profile-position move?" went:
   no ramp, MIB trajectory deferred (6195075) -> the ramp is Motion Studio's
   stored Speed/Accel/Decel, "confirmed on the bench", no MIB trajectory needed
   (19aa81a) -> profile velocity is honoured below the stored speed (d1e0ec5)
   -> stored values are really commands 70/68, not Motion Studio's page
   (46cc6e1) -> "final finding": targets are steps, so the MIB streams its own
   trapezoid (46cc6e1). Each step rests on watching or listening to one motor.
   Her own espp PR #910 says "a stored accel of 50 counts/s^2 visibly slowed a
   move's ramp", while DESIGN.md:328 says "even with the stored accel at 1000 a
   CiA 402 move was a jolt". Both statements are still current. A 200 Hz
   encoder log of one move would settle it.
2. **The firmware path cannot move a joint.** `do_move()` sends only the
   current position as the first target (actuator.cpp:497-505) and relies on
   `poll()` to stream the rest. Nothing in the firmware calls `poll()`, or
   even `init_actuators()` (no caller outside `components/Actuator` and
   `components/bsp`). Only the example console, which starts its own 50 ms
   poll task (actuator_example.cpp:574-581), moves anything.
3. **The setpoint streaming is far slower and more irregular than the design
   assumes.** Each streamed setpoint is a full CiA 402 new-set-point handshake
   (espp: `canopen/include/ds402.hpp:312-371`): write 0x607A, write
   controlword, poll statusword bit 12 with 25 ms sleeps, write controlword,
   poll until bit 12 clears. That is at least 5 SDOs and often 25-50 ms of
   sleeping per setpoint, on top of the 50 ms poll period: an effective
   ~10-20 Hz staircase with no velocity feed-forward, into a P-only position
   loop (P = 0.1, commit 665a303) whose speed ceiling is set equal to the
   cruise speed (actuator.cpp:138-141). Six axes on one task would divide that
   further. espp's `CanopenClient` already has RPDO/TPDO/SYNC helpers
   (espp: `canopen/include/canopen_client.hpp:24-28,199-210`); none are used.
4. **Steady-state error is hidden by a 200-count band instead of fixed.** The
   commit that set tolerance 200 says the loop "stalls 90 to 400 counts short at
   P 0.1" and that 200 is "a fraction of a percent of travel" (665a303). It is
   4.9 % of the absolute-encoder legs' configured range (0-4095), 1 % of the
   carriages' (0-20000) and 2.5 % of the bench example's (10000-18000). The
   arrival test (`still && |here - target| <= tolerance`, actuator.cpp:671)
   logs "arrived" while short, the rest watch measures drift from the rest
   point, not the target (actuator.cpp:636), so the real band is up to 2x
   tolerance, and `set_encoder()` accepts a readback 200 counts off
   (actuator.cpp:877). The position-PID record has I, MaxI and Deadzone fields
   (espp: `mcp266/include/mcp266.hpp:197-230`) that were never tried.
5. **Faults are cleared without being looked at, and homing treats them as
   the home switch.** `Mcp266::enable()` silently fault-resets on every move
   (espp: `mcp266.hpp:629-633`); `clear_faults()` also sends the device-wide
   E-stop reset (actuator.cpp:333); `home()` declares the switch found when
   the count stops changing (actuator.cpp:1011-1026) without checking drive
   state or current, then calls `clear_faults()` and writes the home count
   (actuator.cpp:1045-1046). The battery-sag fault she documented herself
   (DESIGN.md:278-284) stops the motor, so it would be read as "home found"
   at the wrong place, then cleared. Nothing reads motor current, duty,
   voltage or temperature during motion.

Also notable: DESIGN.md contradicts itself on three bench facts (limit-switch
bit 11, object 0x2041, object 0x2044; see C7, C8, C13), the drive is released
unpowered at rest on axes that carry the chair's weight with no evidence that
they self-lock (C29), and the tip depends on an unmerged espp fork branch
(`ewang360/espp` `rammp/mcp266-defaults`, components/Actuator/idf_component.yml),
not espp 1.3.7 as the README says.

---

## 1. Architecture as built

### 1.1 Layers

| Layer | Where | What it does |
|---|---|---|
| Application / kinematics | not built (DESIGN.md:20-27, 249-262) | planned: units <-> counts per joint family |
| `mib::bsp::MIB` | components/bsp/src/bsp.cpp:99-277, include/bsp.hpp:106-150 | owns TWAI, one `CanopenClient` + `espp::Mcp266` + mutex per node id, six `Actuator`s, one `ActuatorStore`; `init_actuators`, `home_actuator(s)`, `stop_all_actuators`, `read_all_positions`, `save_actuator_positions`, `set_actuator_range` |
| `mib::Actuator` | components/Actuator/src/actuator.cpp | clamp, reset detection, MIB-side trapezoid streaming (`poll`), arrival/release/rest watch, calibration mode, homing, position memory |
| `mib::ActuatorStore` | components/Actuator/src/actuator_store.cpp | NVS i32 keys `<name>_min`, `<name>_max`, `<name>_pos` in namespace `actuator` (actuator_store.hpp:29) |
| `espp::Mcp266` / `Ds402Drive` / `CanopenClient` / `Twai` | espp `mcp266`, `canopen`, `twai` (fork branch for `set_default_speed/accel`) | CiA 402 state machine, profile objects, new-set-point handshake, manufacturer mirrors at 0x2000 + command number |
| MCP266 | hardware | position PID (P-only, 0.1 on the bench), velocity loop (QPPS 2750 bench motor), clamp, limit-pin functions |

Nothing above the BSP exists. `MibSystem` on the branch never calls
`init_actuators()` or `poll()`; the commit that created the BSP wiring says
so ("Nothing calls init_actuators yet", 6e4ecd5) and that is still true at tip.

### 1.2 Where the loop closes

Two nested loops, split across two devices:

- **Inner: on the MCP266.** CiA 402 profile-position mode (mode 1) on each
  channel; the MCP closes position (and, beneath it, velocity) on its own
  encoder input. Gains live in the controller (tuned in Motion Studio;
  README.md:117-119), never written by the MIB except espp seeding P = 15491
  (~15.1) if the stored P reads 0 (espp: `mcp266.hpp:165-170`, called from
  actuator.cpp:126).
- **Outer: on the MIB, open-loop in time.** `step_trajectory()`
  (actuator.cpp:535-599) integrates a trapezoid (accel, cruise, decel, with
  braking on retarget) from wall-clock `dt` (capped at 100 ms,
  actuator.cpp:32,545-547) and writes the rounded setpoint as a new
  profile-position target each `poll()`. It does **not** read position while
  streaming; there is no feedback into the trajectory, no following-error
  check, no velocity feed-forward.
- **Supervisory, on the MIB, after the stream ends.** `poll()`
  (actuator.cpp:601-722) reads position (0x6064) and target-reached
  (0x6041 bit 10) every 100 ms; declares arrival; releases the drive; then
  watches the count every 200 ms and re-issues a full move if it drifts
  more than `tolerance` from the rest point (at most once per second).

### 1.3 What goes on the bus

M2 objects are M1's +0x800 for 0x6xxx and command n+1 for manufacturer
mirrors (espp: `mcp266/include/detail/mcp266_core.hpp:40-130`). Units are raw
encoder counts, counts/s, counts/s^2 throughout; no unit conversion anywhere.
All traffic is expedited SDO, 100 ms SDO timeout (bsp.cpp:162).

**Boot, `initialize()`** (actuator.cpp:185-225):

| Step | Objects |
|---|---|
| NMT start (BSP, once per node) | NMT 0x01 (bsp.cpp:216-221 via `Mcp266::start`) |
| `reset_faults` | read 0x6041 per axis; if fault, CW 0x0000 then 0x0080, poll 0x6041 |
| `configure_position_loop` | read 0x203F:1-7 (cmd 63, PID readback P,I,D,MaxI,Deadzone,MinPos,MaxPos); write 0x203D:1-7 (cmd 61, setter order D,P,I,...); read 0x203F:6,7 to verify |
| `set_software_position_limits` | write 0x607D:1, 0x607D:2 |
| `set_default_speed` / `set_default_accel` (fork only) | write 0x2046:0 (cmd 70, u16 = profile.velocity) and 0x2044:1 (cmd 68, accel); per PR #910 |
| incremental axes: `restore_position` | `disable_loop` (read 0x6041; CW 0x0000; poll), write 0x2016:0 (cmd 22, set encoder), read 0x6064 |

**Every move, `do_move()`** (actuator.cpp:414-533): roughly 15-20 SDOs plus
at least one 25 ms sleep before anything moves.

| Step | Objects | Line |
|---|---|---|
| `verify_limits` | read 0x203F:6, :7 | 164-183 |
| `release_quick_stop` | read 0x6041; maybe CW 0x000F or 0x0000 + poll | 346-371 |
| `limit_active` | read 0x6041 (bit 11) | 313-320, 454-481 |
| `read_position` | read 0x6064 | 485 |
| `move_to_position(first = current)` | write 0x6060 = 1; sleep 25 ms; read 0x6041; maybe fault reset; enable sequence CW 0x06/0x07/0x0F with 0x6041 polls; write 0x6083, 0x6084, 0x6081; then the handshake below | 504-505; espp `mcp266.hpp:337-353, 617-635` |

**Every `poll()` while streaming** (actuator.cpp:575-584): the handshake only:
write 0x607A; CW 0x003F (enable + new set-point + change-immediately); poll
0x6041 until bit 12 with 25 ms sleeps (Mcp266 default `poll_period`,
espp: `mcp266.hpp:63`); CW 0x002F; poll 0x6041 until bit 12 clears. Up to
2 x 1 s timeout per setpoint.

**After the stream:** read 0x6064 + 0x6041 every 100 ms; on release, CW 0x0000
(disable voltage) and poll for Switch On Disabled. **At rest:** read 0x6064
every 200 ms indefinitely.

**`stop()`**: CW 0x0002 (quick stop), no wait (actuator.cpp:752-773).

### 1.4 Feedback read, and rate

| Signal | Object | When / rate |
|---|---|---|
| Position | 0x6064 | on demand; 100 ms during arrival; 200 ms at rest; 100 ms during homing |
| Statusword | 0x6041 | inside handshakes; 100 ms during arrival |
| Clamp | 0x203F:6,7 | before every move |
| Velocity 0x606C, duty (cmd 48), currents (cmd 49), battery (cmd 24), temperature (cmd 82), error code 0x603F, status flags (cmd 90) | — | **never** in the component. `flags` (0x205A) and raw `sdor` exist only as console commands (actuator_example.cpp:411-458, 375-394) |

Position is never sampled fast enough to see a ramp: the fastest automatic
read is 10 Hz.

### 1.5 Homing (`home()`, actuator.cpp:937-1066)

Incremental-encoder axes only (the two carriages). Lift clamp to +/-1e9
(`kUnlimited`, actuator.cpp:14); command a profile-position move 50 000 counts
toward the switch at the homing profile (500 counts/s; **not streamed**, so
whatever the controller does with a step target applies); poll position
every 100 ms; after a 2 s grace, a count that changes by less than 2 for
500 ms is "stopped"; if target-reached is set, it was the end of a segment, so
issue another; otherwise it is the switch. Then quick stop, `clear_faults()`
(CiA 402 fault reset **and** device-wide E-stop reset), `set_encoder(home_count)`
with a 200-count readback tolerance, restore the clamp, save the position.
Blocks the caller up to 60 s. A cancel hook aborts with a quick stop.

### 1.6 Limits

Three layers (DESIGN.md:129-142): `Actuator::clamp()` before sending; the
MCP's position-PID MinPos/MaxPos clamp (cmd 61/63); CiA 402 0x607D. Whether
the last two are one mechanism is explicitly untested (DESIGN.md:120-127).
Limit switches (carriages only) are handled by the MCP's S-pin functions.
Calibration mode lifts the clamp to +/-1e9 and allows only jog moves
(actuator.cpp:428-434).

### 1.7 Faults and offline handling

- `note_result()` (actuator.cpp:56-93): an SDO timeout probes the statusword;
  a silent controller is marked offline and skipped for 2 s. Good design.
- Reset detection: before every move, clamp read back != installed means the
  controller rebooted; `initialize()` is re-run, which for an incremental axis
  restores the last saved position (actuator.cpp:164-183, 212-216).
- Faults are cleared, not reported: `initialize()` and espp's `enable()`
  (every move) fault-reset unconditionally; nothing reads 0x603F or 0x1001,
  or logs why the drive faulted.

### 1.8 Persistence

`ActuatorStore` (actuator_store.cpp): ranges (absolute-encoder legs, written
by `set_range`) and last position (incremental legs, `save_position()`
throttled to >= 20 counts moved and >= 5 s, actuator.cpp:911-935). The BSP
loads a saved range over the compiled default (bsp.cpp:203-207). No EEPROM
writes to the MCP (DESIGN.md:154-161).

### 1.9 The MIB-side trapezoid (`step_trajectory`, actuator.cpp:535-599)

Velocity-state integrator: if heading away from the target, brake at decel;
if within stopping distance `v^2 / 2d`, decelerate; else accelerate to cruise.
Ends by snapping the setpoint to the target when it passes or when
|v| < 1 and |left| < 1. Retargeting mid-move continues from the in-flight
setpoint and velocity (do_move, actuator.cpp:488-496). The maths is
reasonable. The problems are the transport and the loop beneath it (section 3).

---

## 2. Claims ledger

Grades: **measured** = numbers recorded with a method that could be re-run;
**observed once** = seen/heard/read back on the bench, no record kept;
**inferred** = reasoned from code, spec or another observation;
**unsupported** = asserted with no evidence on the branch;
**contradicted** = conflicts with another statement on the branch.
No claim on the branch is graded *measured*.

Signals named in experiments: position 0x6064, velocity 0x606C, position
demand 0x6062/0x60FC (if implemented), statusword 0x6041, error code 0x603F,
duty (cmd 48, 0x2030), motor currents (cmd 49, 0x2031), main battery (cmd 24,
0x2018), status flags (cmd 90, 0x205A), IMU pitch/roll/gyro from the BNO055
(already read at 100 Hz by `MibSystem`, components/MibSystem/src/mib_system.cpp:737-765,
MIBconfig.hpp:80). "Fast log" means one object read in a tight SDO loop
(a few hundred Hz is plausible at 1 Mbit/s; measure it) or a TPDO, with
MIB timestamps, written to a file.

| # | Claim (where) | Her conclusion | Evidence on branch | Grade | Alternative explanations | Cheapest settling experiment |
|---|---|---|---|---|---|---|
| C1 | Profile accel/decel 0x6083/0x6084 accepted, read back, not applied (DESIGN.md:305-307, 328-331; README.md:43-46; 46cc6e1) | Profile-position targets are steps; MIB must make the trapezoid | Readback of the objects; a move "was a jolt" with stored accel 1000 | observed once (by eye), **contradicted** by PR #910 ("accel of 50 visibly slowed a move's ramp") | Units/scaling of 0x6083 differ from counts/s^2 (1000 may be effectively instant); the stored default accel (cmd 68) governs and 0x6083 does not, which is a different statement; the "jolt" is the P-only loop's response to a large error, not an absent ramp; the speed ceiling was the 32768 "no limit" sentinel at the time | One 3000-count move per setting: 0x6083 in {50, 200, 1000, 5000} x stored accel in {50, 1000}. Fast log 0x6064 + 0x606C (+ duty). Fit the velocity slope; compare with the commanded accel |
| C2 | Profile velocity 0x6081 honoured when below stored speed (DESIGN.md:314-318; a15fc50 "appears to") | `Profile.velocity` is a per-move cap under the stored ceiling | 100 counts/s "visibly slow and cogging", 500 "smooth" | observed once (by eye) | The observed speed is the position loop's natural closing speed, not a cap; cap is honoured only in some state | Long move, stored speed fixed at 1500, 0x6081 in {200, 500, 1000, 2000}; fast log 0x606C plateau |
| C3 | Motion Studio Position-page Speed/Accel/Decel are what CANopen moves use, "confirmed on the bench" (19aa81a; DESIGN.md:311-314) | Trapezoid is a controller setting; no MIB trajectory needed | "Confirmed on the bench" | **contradicted** by her own later finding (DESIGN.md:321-324) but still in the text | — | Superseded; delete from DESIGN.md |
| C4 | Stored speed/accel are packet-serial cmds 70/71 and 68/69; decel field inert, one ramp per axis (DESIGN.md:321-327; PR #910) | Install them every boot | Readback via cmd 72/81; decel write accepted, readback unchanged | observed once (readback) | Decel at a different sub-index or width; cmd 81 field order differs from assumed | Write decel at each sub/width, read back via 81; then a move with a decel distinct from accel, fast log 0x606C on the decel side |
| C5 | Write-settings (cmd 94) accepted but did not persist speed/accel; power cycle shows 32768 (DESIGN.md:322-324; PR #910) | Write them every boot | One power cycle, readback | observed once | The CAN mirror of cmd 94 may need a specific payload/key or width (an SDO "accepted" only means the object exists); some settings groups may not be part of 94 | Change a setting that Motion Studio's save does persist (e.g. position deadzone), issue 94 over CAN with each payload variant, power cycle, read back |
| C6 | Packet-serial buffered position cmd 65 mirror (0x2041) "accepts every write and moves nothing" (DESIGN.md:308-310, 6195075) vs "does move the motor ... with a real trapezoid ... outside the CiA 402 state machine: quick stop, disable voltage and a duty of 0 did not stop it" (DESIGN.md:331-335, 46cc6e1) | Do not use 0x2041; stream on the MIB | Two bench sessions, no record of the exact SDO sequence either time | **contradicted** (both statements are in DESIGN.md today) | First attempt used a wrong sub-index/buffer flag; second correct. "Cannot be stopped" may depend on state; duty-0 used the duty mirror that espp says is inert anyway | Write the exact sequence down; replay with a fast log of 0x6064/0x606C/duty. Test every stop path (CW 0x0000, CW 0x0002, NMT stop 0x02, E-stop pin, cmd 0 / cmd 32). **Safety-relevant**: if a CAN command can produce unstoppable motion, it must be characterised before anything else runs on the chair |
| C7 | Default-accel mirror 0x2044 is read-only (DESIGN.md:310) | — | One write attempt | **contradicted** by PR #910 (0x2044 sub 1 writes and reads back) | Earlier attempt at sub 0 | Superseded; delete |
| C8 | Supported modes 0x6502 reads 0x700: cyclic-synchronous velocity and torque only (DESIGN.md:311-312) | Only pp works anyway | One readback | observed once | 0x700 = bits 8, 9, 10 (csv, cst, cstca). Bit 0 (pp) is not set, yet pp works, so either the object is unreliable or CSV is real and untested. espp has no CSV mode enum (espp: `canopen/include/detail/canopen_core.hpp:545-550`) | Write 0x6060 = 9 (CSV), enable, write 0x60FF small; log 0x6061, 0x606C, duty. If CSV works with RPDO + SYNC, it is the natural streaming interface |
| C9 | Velocity/duty commands accepted but inert; only profile position moves the motor (README.md:111-113, from espp's README) | Jogging must be a closed-loop position move | espp authors' testing | inferred (others' observation) | espp's `drive_speed`/`drive_duty` first enable CiA 402 ProfileVelocity (espp: `mcp266.hpp:363-384`). C6 suggests manufacturer commands run outside the CiA 402 machine, so the enable may be what blocks them | With the axis in Switch On Disabled and with it in Operation Enabled, write a small duty to 0x2020 and a small speed to 0x2023; log duty readback + 0x6064 |
| C10 | Quick stop leaves the drive in Quick Stop Active, which the standard enable sequence cannot leave (DESIGN.md:337-340; 15b1a1f) | Release it before each move: transition 16, else 12 | Bench observation; consistent with CiA 402 (Shutdown is not a valid transition out of QSA; espp's `enable_operation` sends Shutdown first, espp: `ds402.hpp:201-207`) | observed once + spec-consistent (**credible**) | — (the fix belongs in espp's `enable_operation`) | Read 0x605A (quick-stop option code) and 0x6085 (quick-stop decel). Quick stop mid-move with a fast log of 0x6064/0x606C to measure stopping distance and whether QSA holds position |
| C11 | Rewriting the encoder with the loop active makes the loop chase the old target ("set to 0 at 20000: the motor drove back to 20000") (DESIGN.md:340-342; 23a4046; actuator.cpp:853-855) | Disable the loop before `set_encoder` | One observation, with numbers | observed once (**credible**: exactly what a position loop does) | — | Note: this observation also shows the cmd 22 mirror works, yet README.md:114-116, DESIGN.md:177 and 262, and actuator.hpp:229-231 still call `set_encoder` unverified. Confirm with a readback log and close it |
| C12 | MCP266 reports no limit switch over CAN: statusword bit 11 never set, cmd 90 flags always 0 (DESIGN.md:289-292, 84afc31 at 13:26) vs "statusword bit 11, which this controller does set once the pin is configured" (49506e8, same day 15:55) | Code uses bit 11 for `limit_active()` (actuator.cpp:313-320) to gate moves (454-481) | Two contradictory observations; DESIGN.md never updated | **contradicted** | Bit 11 may depend on pin function, drive state, or only assert while commanded into the switch; cmd 90 read with the wrong width (the console tries u32 then u16, actuator_example.cpp:433-439); the bit map is from RoboClaw docs, "unverified" (414-417) | For pin function in {limit, home, disabled} x drive {idle, enabled, commanded into switch}: press the switch, log 0x6041 and raw 0x205A (both widths). 10 minutes, no motion needed for most cells |
| C13 | Pin as "home" latches an M1 home error; as "limit" blocks one direction without latching (DESIGN.md:292-295) | Use the limit function | One observation | observed once | — | Part of C12 |
| C14 | After a limit-switch stop, the position loop is "wound up against an unreachable target; the next move ... runs away (seen on the bench)" (actuator.cpp:450-453; 49506e8) | Drop the drive to Switch On Disabled first; refuse moves toward an active switch | One observation | observed once | With P-only there is no integrator to wind up; more likely the drive kept the old target and resumed it once released, or the new target's handshake was missed (ack bit still high) | Drive into switch, then command away; fast log 0x607A, 0x6062/0x60FC, 0x6064, duty |
| C15 | A target a billion counts away makes the controller's trajectory maths overflow and the motion stutter (actuator.hpp:87-90; fee5f4d) | Home in 50 000-count segments | One observation (by eye) | observed once | Velocity-cap or clamp interplay; overflow plausible but unproven | Low priority; segments are harmless. Log 0x606C during a +1e9 target if anyone cares |
| C16 | Too low a speed (100 counts/s) makes the velocity loop cog, QPPS 2750 (actuator.hpp:69-70, 82-83; DESIGN.md:316-317; a15fc50) | Keep speeds well above the cogging floor | By eye | observed once | Speed estimate quantisation at low counts/s; static friction; P-only position loop | Constant-velocity sweep 50-1000 counts/s; fast log 0x6064/0x606C; compute velocity ripple vs speed |
| C17 | An arrived loop hunts; the gearmotor whirs (DESIGN.md:343-345; README.md:54-56) | Release the drive after arrival | Audible | observed once | Position-PID Deadzone = 0 (a configurable field, espp: `mcp266.hpp:204-205, 248-276`); gain too high for the backlash; encoder quantisation. All are configuration, which should come before a release scheme | Hold at target 10 s with deadzone in {0, 5, 20} and the current gains; fast log 0x6064 + duty + current |
| C18 | The loop "stalls 90 to 400 counts short at P 0.1" (665a303) | Raise tolerance to 200 | Numbers quoted, no log | observed once | Expected for a P-only loop against friction/gravity: no integral term. A tuning fact, not a controller quirk | Step responses with P vs PI (MaxI bounded); fast log error vs time; report the steady-state error per setting |
| C19 | Tolerance 200 is "comfortably outside that and still a fraction of a percent of travel" (665a303) | Tolerance 200 for every leg | Arithmetic | **unsupported** (false by arithmetic: 4.9 % of 0-4095, 1 % of 0-20000, 2.5 % of the bench 10000-18000) | — | None needed; tolerance must come from a measured requirement per axis |
| C20 | Rear caster: "whose shock absorber lets the count sag under load" (DESIGN.md:347-349) | Rest watch moves the joint back | No record of which hardware, load, or amount | unsupported | The actuator back-drives once released (no self-locking); encoder sits after a compliant element, so the count moves with load, not actuator position | On the chair: hold vs release, chair loaded; log 0x6064 at 50 Hz and IMU pitch/roll for 60 s each |
| C21 | Releasing the drive at rest is fine: "the joint rests unpowered" (README.md:59-61; actuator.hpp:164-166) | Default for every axis | None for a loaded axis; bench motor was unloaded | unsupported | Lead-screw actuators may or may not self-lock; a gas-spring/shock linkage stores energy | Same experiment as C20, per axis, with the occupant mass. Pass/fail by count drift and IMU tilt drift per minute |
| C22 | MCP266 reverts to EEPROM at power-up; factory clamp [0, 0] forces every target to 0 (DESIGN.md:144-152; README.md:108-110) | Re-install every boot; detect resets by reading the clamp back | From espp (espp: `mcp266.hpp:38-46`) | inferred (others' observation, **credible**) | — | Power-cycle one MCP while the MIB runs (her own "unverified" item, DESIGN.md:301-303); confirm detection fires |
| C23 | espp's PID field-order remap is correct: gains survive `initialize()` (DESIGN.md:296-298) | — | Readback | observed once (**credible**) | — | Keep; recheck once on each robot controller |
| C24 | Battery minimum 29 V on a 30 V supply tripped main-battery-low at motor start, the drive faulted and the error self-cleared (DESIGN.md:278-284) | Set min ~21 V, max >= 30 V for 24 V | One observation | observed once (plausible) | — | Log cmd 24 voltage + 0x6041 + 0x603F at 100 Hz during a move start on the bench supply and on the chair battery |
| C25 | Bench supply needs 4700-10000 uF across the controller for regeneration (DESIGN.md:285-288) | — | Supply crackled/tripped | observed once (plausible, standard practice) | — | Keep as bench practice |
| C26 | Motion Studio may "update" a controller down to its bundled firmware, wiping settings (DESIGN.md:268-272) | Back up settings first | One incident | observed once | — | Keep as procedure; record firmware versions per controller |
| C27 | Clamp (cmd 61) and 0x607D: one mechanism or two? (DESIGN.md:120-127) | Write both | Not tested | open | — | Wide clamp + narrow 0x607D, target beyond; then the reverse; log 0x6064 and the abort/ack |
| C28 | Restoring the saved position at boot is valid "on the assumption that it did not move while off" (DESIGN.md:181-186) | Restore instead of homing | Not tested (DESIGN.md:301-303) | unsupported | Released-at-rest axes can drift while powered and while off; the save is throttled 5 s / 20 counts | Power cycle after a move, then home and compare the restored vs homed count; repeat after loading the axis while off |
| C29 | A stall is the limit switch: "detected only by their effect, the count ceasing to change while a move is commanded" (DESIGN.md:289-293; actuator.cpp:1011-1026) | Homing by stall detection | Never run against a real switch (DESIGN.md:301-303) | inferred | Fault (e.g. battery sag, C24), current limit at a jam, hard stop with a failed switch, drive left Operation Enabled | Home with the switch disconnected (hard stop at current limit) and with a fault injected; log 0x6041, 0x603F, current, 0x6064 |
| C30 | A set-point acknowledge timeout on the long homing approach means the move is running (actuator.cpp:973-976; d4e302f) | Treat timeout as success | None recorded | inferred | Handshake genuinely failed; ack bit stuck high from the previous set-point | Log 0x6041 bits 10/12 and 0x6064 through a homing approach |
| C31 | Statusword/position as TPDOs untested (DESIGN.md:349-351) | Poll over SDO | — | open | — | Read 0x1800-0x1803 / 0x1A00-0x1A03; try mapping 0x6064 + 0x6041 to TPDO1 with SYNC or event timer |
| C32 | Node ids set in Motion Studio: 1 casters, 2 carriages, 3 main legs (665a303) | MIBconfig rows | Stated configuration | observed (configuration fact) | — | `scan` console command on the chair; read 0x1008 per node |

---

## 3. Controls critique

### 3.1 Two position loops, neither in charge

The MIB integrates a trajectory open-loop and feeds it, as a sequence of
step targets, into the MCP's own profile-position loop. That loop is P-only
on the bench (P = 0.1, 665a303), and `initialize()` sets the MCP's speed
ceiling equal to the cruise speed (actuator.cpp:138-141). Consequences:

- During cruise the drive can never catch up: any lag (from the P-only loop
  or from late setpoints) cannot be recovered because the cap equals the
  reference speed. Following error grows through the move and is only
  closed at the end, at the cap, which looks like the "jolt" the stream was
  meant to remove.
- Each setpoint is a position step of `v * dt` (at 1000 counts/s and
  ~70 ms, ~70 counts). With "change set immediately" set (ds402.hpp:317-320)
  the MCP replans every step. Whether it applies its stored accel to each
  mini-move (which would give a sawtooth velocity) is unknown. No signal is
  logged that could show it.
- No velocity or torque feed-forward, no following-error monitoring on the
  MIB, no use of the drive's own following-error window (0x6065/0x6066).

A controls engineer would pick one owner of the trajectory. Either (a) prove
the MCP's internal ramp with a fast log (C1/C4) and send one target per move,
or (b) stream at a fixed rate with SYNC + RPDO into a mode meant for it (CSV,
if C8 shows it works) with the position loop and feed-forward on the MIB. The
current hybrid gets the costs of both.

### 3.2 Setpoint rate vs CAN latency

- Per setpoint: >= 5 SDOs plus 0-2 x 25 ms statusword-poll sleeps
  (ds402.hpp:329-370 with `poll_period` 25 ms, espp: `mcp266.hpp:63`; the BSP
  and example pass only `log_level`, bsp.cpp:166-167). Plus the 50 ms task
  delay (actuator_example.cpp:578). Effective rate roughly 10-20 Hz, jittery,
  unmeasured.
- `dt` is measured wall-clock and capped at 100 ms (actuator.cpp:32,
  545-547): when the bus is slow, the trajectory silently runs slower than
  commanded rather than reporting it.
- Six axes in one task (the planned owner) would serialise: ~6 x 30-60 ms
  per cycle. Two axes on one MCP also serialise on the shared mutex, and a
  handshake can hold it up to 2 s.
- Nothing on the branch measures any of this: no loop-period histogram, no
  SDO round-trip time.

### 3.3 Releasing the drive at rest on a gravity-loaded axis

`poll()` disables voltage half a second after arrival (actuator.cpp:689-712).
On an axis that carries the chair and occupant, that is a choice between
self-locking (unknown, C21) and back-driving. The rest watch then re-moves the
joint once it is more than 200 counts from the rest point, at most once a
second (actuator.cpp:631-650): on a back-drivable axis that is a slow sawtooth
limit cycle, each correction a full ~15-20-SDO `do_move`. If the encoder sees
the joint through a compliant element (the rear-caster shock, C20), the watch
pushes the actuator to cancel load-dependent compression, so the actuator's
real position walks with load and overshoots when the load comes off. The
alternatives (hold with a deadzone and an integrator, or a brake) were not
tried. The decision to hold or release should be per axis and made from a
drift measurement under load (C20/C21).

### 3.4 Tolerance bands hiding steady-state error

`tolerance` is used for four different things: arrival (actuator.cpp:671), the
rest-watch deadband (636), the `set_encoder` readback check (877, so a home
count 199 counts wrong passes), and the selftest pass criterion
(actuator_example.cpp:531-533). It was raised from 10 to 200 globally because
the loop stalls short (665a303); the root cause (no integral action, C18) stays.
Because the watch measures drift from the rest point rather than the target,
a joint can sit up to ~2 x tolerance from its target without a correction.
The stall branch releases the drive 3 s after the last setpoint wherever it is
(actuator.cpp:29, 669, 713-720) and only logs a warning: the caller is never
told the move failed (`poll()` returns true).

### 3.5 No current, stall or fault protection

- No read of motor current, duty, voltage or temperature anywhere in the
  component. A jammed actuator, a mechanical stop or a dragging joint is
  invisible except as "count stopped".
- espp's `enable()` fault-resets on every move (espp: `mcp266.hpp:629-633`);
  `initialize()` resets faults; `clear_faults()` also sends the device-wide
  E-stop reset (actuator.cpp:333), and `home()` calls it (1045). If the MCP's
  E-stop input is wired into the chair's e-stop chain, homing clears it
  without anyone deciding to. Nothing reads 0x603F, so the cause of a fault is
  lost.
- An SDO failure mid-stream abandons the move with the drive left in
  Operation Enabled holding the last intermediate setpoint, and clears
  `pending_release_`, so it is never released (actuator.cpp:586-593).

### 3.6 Homing against stops

- Stall = switch (C29). A fault, a jam or a hard stop with a failed switch
  produces the same signature, and the code then clears the fault and writes
  the home count. The battery-sag fault in DESIGN.md:278-284 is exactly this
  case.
- The approach is a single profile-position target, not the streamed
  trapezoid, so it runs at whatever the controller does with a step (by her
  own finding, no ramp) straight into the switch.
- After a missed switch, the motor pushes against the stop for at least the
  500 ms stall time (plus up to 2 s grace) with no current monitoring.
- Configuration disagrees: MIBconfig homes carriages toward min (direction -1,
  home count 0; MIBconfig.hpp:233-234) while the example, set up on the actual
  rack, homes toward the front switch at +1 with home count 12 600
  (actuator_example.cpp:75-80, 90-95; 139ce2c).

### 3.7 No coordination between axes

Each `Actuator` is independent: no synchronised start, no common arrival time,
no limit on pitch/roll change, no reaction in one axis to another's failure.
On a chair whose legs together set the seat attitude, an axis that stalls
while its partner keeps going changes pitch/roll. `stop_all_actuators()`
exists (bsp.cpp:253-262), but nothing calls it. The BNO055 is already read at
100 Hz on this branch (mib_system.cpp:737-765) and is the obvious supervisory
signal; the actuator code does not use it.

### 3.8 Blocking calls and thread safety

- Every call blocks for several SDOs; `command_state` up to 1 s; each
  handshake up to 2 s; `home()` up to 60 s. Fine in a dedicated task, but the
  BSP API offers no non-blocking form and no task.
- Lock order is consistent (controller mutex, then state mutex); I found no
  deadlock. There are races, though:
  - `step_trajectory()` computes the setpoint under the state lock and sends
    it later under the controller lock (actuator.cpp:538-584). A `do_move()`
    from another task can send its first target in between, and the stale
    setpoint then overwrites it.
  - `do_move()` releases the controller mutex between `verify_limits`,
    `release_quick_stop`, the limit check, the position read and the move
    (440-526), so `poll()` can release or correct in the gaps.
  - The rest watch calls `do_move()` from the poll task (649) while the owner
    may be issuing its own move.
- `move_absolute()` returns true once the first setpoint (the current
  position) is accepted, before any motion. Callers that wait on target
  reached return at once: the example's `wait` and `selftest`
  (actuator_example.cpp:224-251, 510-543) almost certainly read "reached"
  for the first setpoint and measure before the move (inferred; the
  selftest's PASS/FAIL under the streaming design is unverified).

### 3.9 Persistence and reset recovery

After a controller brown-out, `verify_limits()` re-runs `initialize()`, which
for a carriage restores the last *saved* position (actuator.cpp:174-182,
212-216). That save may be up to 5 s / 20+ counts stale, and the axis may
have moved since. The joint is then declared homed with a warning. A reset
mid-run should un-home the axis and require `home()`.

### 3.10 What is solid

Credit where it is due. Offline marking with a cheap probe that tells a
silent node from a refused transition (actuator.cpp:56-93); clamp read-back
as a reset detector; quick-stop release that follows the CiA 402 transitions
(C10); disabling the loop before `set_encoder` (C11); calibration mode that
refuses anything but jogs while the clamp is lifted; the shared
per-controller mutex and its documented reason (actuator.hpp:44-50); and a
bench console with `scan`, raw `sdor`/`sdow` and `flags`, which is the right
instrument for this controller. The layering decision (BSP speaks counts,
kinematics above) is sound.

---

## 4. Keep / rework / discard

### Keep as-is (after a one-line confirmation on the chair)

- Wiring and identity: CAN on TWAI TX GPIO17 / RX GPIO16 at 1 Mbit/s; node
  ids 1 (casters), 2 (carriages), 3 (main legs); M2 = M1 + 0x800 / command+1.
- NVS layout: namespace `actuator`, keys `<name>_min`, `<name>_max`,
  `<name>_pos`, names <= 11 characters.
- `ActuatorStore` as written.
- Offline marking (`check_online` / `note_result`).
- Clamp read-back reset detection (but change what happens after it, 3.9).
- `release_quick_stop()` logic; better still, upstream it into espp's
  `Ds402Drive::enable_operation`.
- `disable_loop()` before `set_encoder()`.
- Calibration vs homing split, calibration-mode jog-only rule.
- The bench checklist in DESIGN.md:264-300 (Motion Studio firmware, settings
  after reset, battery limits, regen capacitor, pin functions, autotune order).
- The example console's `scan`, `sdor`, `sdow`, `flags`, `limits` commands.

### Rework

- **Trajectory ownership** (3.1): decide from a fast log of C1/C4/C6/C8, then
  either one target per move or fixed-rate SYNC/RPDO streaming.
- **Feedback**: TPDO (or a dedicated fast SDO poller) for position, velocity,
  statusword, current and duty at a fixed rate, timestamped and loggable.
  This is the measurement backbone the revamp needs anyway.
- **Arrival and holding**: tune PI + deadzone first (prior art: the MCP's
  own PID record); per-axis tolerance from a requirement; hold vs release
  per axis from a measured drift test.
- **Fault handling**: never reset silently; read and log 0x603F, cmd 90 and
  0x6041 on any fault; do not send the E-stop reset from library code.
- **Homing**: check drive state and current during the approach; distinguish
  switch vs fault vs hard stop; exact `set_encoder` readback; one source of
  truth for direction and home count.
- **API semantics**: a move returns a handle or status that reports
  progress, completion, stall and failure; no "arrived" on a stall.
- **Integration**: a single actuator task in `MibSystem` that owns all
  axes, with coordinated start/stop and an IMU tilt supervisor.
- **espp dependency**: get PR #910 merged or drop the dependency; the tip
  cannot build against the registry.

### Discard

- The release-at-rest + rest-watch scheme as a default (keep the idea as an
  option for axes proven to self-lock).
- The global tolerance of 200.
- Treating a set-point acknowledge timeout as "running" (actuator.cpp:973-976).
- Restoring a saved position after a mid-run controller reset.
- The contradictory and superseded text in DESIGN.md "Bench findings"
  (C3, C6, C7, C12); replace it with this ledger's outcomes once tested.

---

## 5. Hardware facts recorded on the branch

Trust: **high** = consistent across the branch and plausibly read off the
hardware; **medium** = stated once, or for a bench motor only; **low** =
placeholder, contradicted, or by eye.

| Fact | Value | Source | Trust |
|---|---|---|---|
| CAN pins | TX GPIO17, RX GPIO16 | MIBconfig.hpp:177-178; actuator_example.cpp:67-68 | high (worked on the bench) |
| CAN bitrate | 1 Mbit/s | MIBconfig.hpp:179; DESIGN.md:51, 275 | high |
| SDO timeout | 100 ms | bsp.cpp:162; example 141 | high (a software setting) |
| Node ids | 1 casters, 2 carriages, 3 main legs | MIBconfig.hpp:221-234; 665a303 | medium (set in Motion Studio; earlier placeholders 10/11/12, bench used 1 then 3) |
| Channels | FC M1, RC M2 (node 1); ML M1, MR M2 (node 3); LCarr M1, RCarr M2 (node 2) | MIBconfig.hpp:229-234 | **low** ("TODO(bench): channels ... are placeholders", MIBconfig.hpp:221-223) |
| Axes | six "legs": FC, RC, ML, MR, LCarr, RCarr | MIBconfig.hpp:187-195 | medium; does not match the four-axis description of the base (see Q1) |
| Encoder types | FC/RC/ML/MR absolute; carriages incremental AB (optical on the rail) | MIBconfig.hpp:224-227; DESIGN.md:167 | medium (only an AB bench motor was tested; MIBconfig.hpp:183 also says every leg is absolute) |
| Ranges | 0-4095 (absolute legs), 0-20000 (carriages) | MIBconfig.hpp:229-234 | **low** (placeholders) |
| Carriage travel | 12 600 counts switch to switch | actuator_example.cpp:80; d4e302f | medium (one rack; method not recorded) |
| Carriage switch wiring | front = M1FLimit, back = M1RLimit, count increases toward the front | actuator_example.cpp:75-79 | medium |
| Carriage homing | +1 to front, home count 12 600 (example) vs -1 to min, count 0 (MIBconfig) | actuator_example.cpp:79, 92; MIBconfig.hpp:233-234 | **low** (contradictory) |
| Profiles | 500 / 500 / 500 counts/s, /s^2 every leg | MIBconfig.hpp:229-234 | low (placeholder) |
| Bench profile | 1000 / 1000 / 3000 | actuator_example.cpp:86 | low (bench motor) |
| Jog step | 50 (MIBconfig), 500 (example) | MIBconfig.hpp; example 87 | low |
| Tolerance | 200 every leg | MIBconfig.hpp:229-234; 665a303 | low (see C19) |
| Bench motor | Dumore gearmotor, AB encoder, node 3 M1, range 10000-18000 (earlier node 1, 37000-45000) | DESIGN.md:264; actuator_example.cpp:71-72, 85; 1655e05 | medium (bench only) |
| Velocity loop | QPPS 2750 (bench motor, autotune) | DESIGN.md:296 | medium (bench only) |
| Position loop | P 0.1, no I/D noted; stalls 90-400 counts short | 665a303 | medium (bench only; not logged) |
| espp fallback P | 15491 raw (~15.1), only if stored P is 0 | espp: `mcp266.hpp:545` | high (code) |
| Stored speed sentinel | 32768 after power cycle ("no limit") | PR #910 | medium (one readback) |
| 0x6086 motion profile type | 0 | DESIGN.md:308 | medium (one readback) |
| 0x6502 supported modes | 0x700 | DESIGN.md:311 | medium (one readback, inconsistent with pp working) |
| Battery limits (24 V system) | min ~21 V, max >= 30 V | DESIGN.md:281-284 | medium |
| Regen capacitor (bench) | 4700-10000 uF across main terminals | DESIGN.md:285-288 | medium |
| Lifted clamp | +/-1 000 000 000 | actuator.cpp:14; 575ac53 | high (a software choice) |
| Homing parameters | 500 counts/s; stall < 2 counts in 500 ms; 2 s grace; 100 ms poll; 50 000-count segments; 60 s timeout | actuator.hpp:78-93; actuator.cpp:16-17 | high as code, unvalidated (never run against a real switch) |
| `poll()` timing | release poll 100 ms; settle 500 ms; watch 200 ms; correction hold-off 1 s; still = 3 counts / 500 ms; stall release 3 s; trajectory dt cap 100 ms | actuator.cpp:21-32 | high as code, no basis recorded |
| espp Mcp266 timing | CiA 402 transition timeout 1 s; statusword poll 25 ms; mode settle 25 ms | espp: `mcp266.hpp:62-63, 550` | high (code) |
| Position save throttle | >= 20 counts and >= 5 s | actuator.hpp:107-108 | high as code |
| IMU | BNO055, I2C 0x28, SDA GPIO7 / SCL GPIO8, read every 10 ms | MIBconfig.hpp:80-86 | high (separate, merged IMU work) |
| NVS partition | 0x9000, 0x6000 bytes | partitions.csv | high |

---

## 6. Open questions for the bench session

Ordered by how much the answer changes the redesign.

1. **What exactly are the axes?** The base is described as four
   linear-actuator axes (drive wheels, front casters, rear casters); the
   branch configures six legs, including two carriages with incremental
   encoders. Which MCP, channel, encoder type, travel and switches belong to
   each real axis? (Read 0x1008 per node with `scan`; jog each channel;
   record the encoder type in Motion Studio.)
2. **Does the MCP266 apply any ramp to a profile-position move, and from
   which objects?** C1, C2, C4: one fast-logged move per setting. This
   decides whether the MIB generates trajectories at all.
3. **Can a CAN command produce motion that CiA 402 cannot stop?** C6. Settle
   before anything else runs on the chair, and record every stop path's
   stopping distance (C10).
4. **Does CSV mode (0x6060 = 9) work, and can the MCP stream TPDOs / accept
   RPDOs with SYNC?** C8, C31. If yes, fixed-rate streaming with an MIB-side
   loop is cheap; if no, streaming is limited to SDO handshakes and should be
   avoided.
5. **Do the leg actuators self-lock under the chair's weight?** C20, C21:
   hold vs release, loaded, 60 s, encoder + IMU pitch/roll. Decides hold
   vs release per axis and whether the rest watch exists.
6. **What steady-state error and holding behaviour do PI + deadzone give?**
   C17, C18: replaces the 200-count tolerance with a measured number per axis.
7. **Are limit switches visible over CAN, in which pin mode?** C12, C13.
   Decides whether homing can see the switch or has to infer it.
8. **What does homing see when the switch is missed or the drive faults?**
   C29, C30: log state, error code, current. Decides the homing detector.
9. **What current does each axis draw moving, holding and stalled, and what
   is the MCP current limit set to?** Needed for stall protection; nothing on
   the branch records it.
10. **Is the clamp the same mechanism as 0x607D?** C27.
11. **Does write-settings (cmd 94) persist anything over CAN, and with what
    payload?** C5. Decides whether the MIB must rewrite settings every boot.
12. **Does reset detection fire on a real MCP brown-out, and what state does
    the axis come back in?** C22, 3.9.
13. **What is the real SDO round-trip time and the achievable fast-log rate
    on this bus?** Needed to size every logging and streaming decision.

---

*Prepared from a read-only review: no hardware was touched, nothing was built.
Where this document says "inferred", it is my reading of the code and has not
been run.*
