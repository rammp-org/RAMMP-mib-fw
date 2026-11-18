# The leg actuators: the MCP266 as measured, and how the Actuator drives it

What the controller does, what this component does with it, where the limits live, and
what is still open. The evidence is the chair session of 2026-11-17,
[`docs/actuator-revamp/bench-findings.md`](../../docs/actuator-revamp/bench-findings.md)
(logs under `docs/actuator-revamp/runs/`), and the bench firmware that took it,
`tools/actuator_sysid/firmware`. This document cites them rather than restating them;
where an older note (the October bench, the handoff) disagrees, the findings win.

## Layers

The **BSP is the electronics interface** (CAN, the MCP266s, their encoders, the
calibration of those encoders) and nothing in it knows what a joint means. Kinematics,
units and moves that span joints belong in an application layer above the BSP (not built
yet), which holds references to the BSP's actuators and speaks to them in encoder counts.

```
  application layer (planned)      units <-> counts, coordinated moves
            |  move_absolute(counts) / move_relative(delta) / stop()
            v
  mib::Actuator (owned by the BSP)  settings at boot, trapezoid, streaming, arrival,
            |                       release, faults; poll() every 10 ms
            |  SDOs: target + controlword handshake, encoder, statusword
            v
  espp::Mcp266 / Ds402Drive / CanopenClient / Twai     (CAN, 1 Mbit/s)
            v
  MCP266 channel: position stage -> velocity PI -> PWM -> EHA
```

## The controller, as measured

Two MCP266 2x60A, software 1.4.9: node 3 drives the main legs (M1 right, M2 left), node 2
the carriages. No node 1: this chair has no powered casters.

**A cascade, not a position PID on PWM.** The position stage turns error into a speed
command, the velocity stage turns speed error into PWM, and there is no feedforward
(findings: "Decisive tests" and "Velocity loop structure"):

```
speed command = position P x (target - count)          capped at the profile velocity 0x6081
PWM           = (velocity P / QPPS) x (speed command - speed) + velocity integral
```

Only the ratios matter. What the numbers mean:

| Setting | Units | Meaning | Stock (right leg) | Set by the MIB |
|---|---|---|---|---|
| position P | x1024 | counts/s of speed command per count of error; a target moving at v is followed v / P behind | 103 (0.10) | 4096 (4.0): 250 ms lag at constant speed |
| position I, D, MaxI, deadband | | | 0, 868, 0, 10 | all 0 |
| velocity P | 16.16 | with QPPS: PWM fraction per counts/s of speed error | 55400 | 65536 (1.0) |
| velocity QPPS | counts/s | the speed the loop takes as 100 % | 176 | 7143 |
| velocity I | 16.16 | carries the load (~5 % PWM going up) | 1701 | 1701 |
| position window 0x6067 | counts | no speed command at all inside it | **100** | 10 |
| profile velocity 0x6081 | counts/s | caps the speed command | | 20000 (never caps) |

Stock settings jolt and stall: velocity P / QPPS is 34x too hot, so a small speed command
is a large PWM burst, the leg overshoots its commanded speed and the velocity loop brakes
it, and the 100-count window then stops it ~90 short ("stalls short", the 200-count
tolerance and the stall release on the old branch). Window and gains were both needed: the
window alone left stock gains 145 counts short.

**No ramp.** The controller runs no trajectory: 0x6083/0x6084 have no visible effect, and
even with the right gains and 0x6081 = 800 a single 1,000-count target ran at ~2,060
counts/s (findings: "Evening session"). So the MIB generates the trapezoid and streams it.

**How a target is taken.** A position target takes effect when the drive reaches
OPERATION ENABLED, from whatever 0x607A holds, so 0x607A is written before enabling. After
that a bare 0x607A write is ignored: every update needs the CiA 402 new-set-point handshake.
espp's handshake polls the statusword (~50 ms per update); the lean one used here writes
target + controlword 0x003F on one 10 ms tick and 0x002F on the next, with no polling (each
SDO < 2 ms). Each bit level must be held a whole tick: back-to-back 0x3F/0x2F worked for
hundreds of updates and then for none of one move.

**Lead.** With position P 4.0 the leg follows a moving target 1/P = 250 ms behind (150 to
200 counts at 800 counts/s). Writing the target ahead of the profile by speed x 250 ms held
it within ~10 counts of the profile; end error unchanged.

**What it achieves (right leg).** 1,000-count moves at 800 counts/s, 1,600 counts/s²,
across 10,530..17,420: end within ~20 counts (up moves end 10 to 20 past, down moves 2 to
17 short), up to ~18 % PWM / 14 A going up, chassis rate under 3.5 deg/s, no ringing
(findings: "Full-range check"). Tilt changes about -2.0 deg per 1,000 counts.

**Holding.** Enabled at rest against the load the loop holds ~4 to 5 % PWM and ~4.5 A for
nothing: the EHAs cannot be back-driven. Disable voltage takes PWM to 0 within 100 ms and
the leg stays put. So a move ends with a release.

**Encoders.** Noise at rest ~50 to 70 counts peak to peak (±30); every decision uses a
median of 9 reads. 0x606C (speed) is unusable; speed comes from encoder differences. The
left leg's encoder (M2) jumps 1,500 to 2,000 counts and decays over ~100 ms about every 10 s
with the chair still: a hardware fault, and the reason the left leg is not commissioned.

**Settings live in RAM.** Everything the MIB writes returns to the controller's flash
values at its next power cycle; nothing here writes the flash. The position PID record must
be read from subindex 1 (subindices 6/7, the clamp, read 0 on their own; espp PR #911), and
written whole in the setter's D, P, I order. The velocity record is set at 0x201C/0x201D
(subindices D, P, I, QPPS) and read at 0x2037/0x2038 (P, I, D, QPPS). The stored default
speed/accel objects 0x2044..0x2047 error on read on 1.4.9 and are not what limits speed;
they are not used.

## CAN and the watchdog

**`tx_retry_count = -1`.** On the ESP32-P4's SJA1000-style controller ESP-IDF sets single
shot for every `tx_retry_count` but -1 (espressif/esp-idf#19195), so 0..15 all mean one
attempt, and a frame that loses arbitration is reported unsent: ~1.6 a second on the chair's
bus, each one a failed SDO. With -1: 2 unsent frames in a 120 s run. The residual rate of
failed SDOs (about one a minute) is why a move tolerates single failures.

**The heartbeat.** The BSP sends a CANopen heartbeat as node 127 every 100 ms from
`init_actuators()` on, and each commissioned controller's heartbeat consumer is set to
0x007F01F4 (node 127, 500 ms) by `initialize()`. If the heartbeat stops (MIB crash or reset,
cable cut), an enabled drive faults after ~450 ms and stops where it is. Writing 0 to the
consumer does not stop it watching, so the heartbeat never stops on purpose.

**What a watchdog trip looks like** (findings: "Evening session"): the statusword shows
FAULT REACTION ACTIVE for ~11.5 s, the drive holding ~2 to 6 % PWM / 1.5 to 3 A, ignoring
disable voltage; then FAULT, PWM 0. A move in progress ends FAILED ("drive faulted"). To
recover: wait for FAULT (the example's `status` shows it, PWM 0), `clear_faults()`
(console: `clear`), then move normally. A drive already in FAULT REACTION ACTIVE may refuse
the clear until it reaches FAULT.

## The component

### initialize()

For a commissioned leg: NMT start of its node, then the settings from its config, all
written to RAM and all read back: the position window, the whole position PID record with
the travel as clamp, the whole velocity PID record, 0x607D, 0x6081 and the heartbeat
consumer. Any field that reads back differently is logged and leaves the leg not ready.
Faults are reported, not cleared. A leg that is **not commissioned** gets no frame at all.

### A move

```
move_absolute(target)        validate: commissioned, ready, not faulted, settings intact
     |                        (a controller that reset is re-initialised), clamp the target
     v
ACQUIRE   9 ticks            encoder every tick; start = median (refused if spread > 200)
     v
enable                       mode 1, 0x607A = start, enable sequence (refuses a fault),
     |                        0x6081
     v
STREAM    every tick         encoder; statusword every 5th tick; alternately
     |                        SET (0x607A = clamp(setpoint), CW 0x3F) / CLEAR (CW 0x2F)
     |                        setpoint = profile + speed x lead, never past the target
     v
settle                       profile ended, final target latched, median within
     |                        tolerance (40) for settle_time (200 ms)
     v
release                      disable voltage (retried); 100 ms later PWM and current are
                             read into the result: ARRIVED
```

A move asked for while one is in progress replans from the profile's position and speed
(`MotionProfile`, host-tested): no jump in position or speed. A **failed** move is stopped
and released and its result says how far short and why: not arrived by the profile time
plus `arrival_margin` (2 s), the drive faulted, the encoder unsettled at the start, the
enable refused, or the feedback lost (5 failed reads in a row, or none for 150 ms). Single
failed SDOs are counted and tolerated.

`move_result()` carries the outcome, start, target, final median count, peak speed (from
the median over 200 ms), profile and elapsed time, the time from profile end to release,
the PWM and current read after the release, and the SDO failures tolerated.

### Ticks: who calls poll()

The owner calls `poll()` (the BSP's `poll_actuators()`) every `Actuator::kTickPeriod`,
10 ms, from one task, on a fixed-period schedule. That is how the repo schedules work:
components are passive (DriveController computes, it owns no task) and MibSystem owns the
periodic tasks, the IMU task with exactly this deadline schedule; and it is how the old
`poll()` was meant to be called. The tick drives the handshake, so its period is the bit
hold time; `poll()` will not toggle the bit faster than 8 ms whatever the caller does, and
the profile runs on the clock, so a late tick delays an update but not the profile. A tick
is one to four SDOs (< 2 ms each) while moving and nothing when idle. MibSystem does not
call `init_actuators()` yet; the example's poll task is the reference (priority above the
console, `xTaskDelayUntil`).

### Faults and stop

Nothing clears a fault as a side effect: not `initialize()`, not a move (espp's
`move_to_position()`, which fault-resets in its enable, is not used), not homing.
`clear_faults()` resets this channel only (espp's `Mcp266::reset_faults()` would reset
both) and no longer sends the device-wide E-stop reset. `stop()` is disable voltage,
immediately, retried up to 8 times; `read_status()` then shows PWM and current. (A quick
stop parked the drive in QUICK STOP ACTIVE, which the enable sequence cannot leave; a
hold-then-disable let the leg run on and reversed it.)

### Concurrency

Two actuators on one controller share its `CanopenClient` and a mutex: a tick, a move
start, the settings are sequences that must not interleave. Lock order is the controller
mutex, then the actuator's state mutex. `stop()` marks the move ended under the state mutex
before it waits for the controller mutex, and a tick re-checks that before every write, so
nothing is sent for a stopped move.

### Deleted, and why

| Old behaviour | Why it went |
|---|---|
| 200-count tolerance | Hid the window stall; the tuned loop ends within ~20. Tolerance is 40, per leg. |
| Stall release: accept a leg 3 s after the stream wherever it stopped, with a warning | A move that does not arrive is now FAILED, released, and reported with how far short. |
| Rest watch: re-move a released leg that drifted past tolerance | EHAs are not back-drivable, and the left encoder's 1,800-count glitches would have driven the leg. No drift monitor replaces it. |
| Stored default speed/accel install (0x2044..0x2047) | Error on read on 1.4.9; not what limits speed. |
| Target-reached flag as arrival | Set while 200 counts away with stock settings; arrival is the median count. |
| Fault reset in initialize(), every move, homing; E-stop reset in clear_faults() | Faults were cleared unread, and homing took a fault for its switch. |
| Quick stop | See above; disable voltage. |
| espp's handshake per setpoint (~50 ms) | The lean handshake, 50 Hz. |

### Commissioned legs, and node 2

`commissioned = false` (MIBconfig.hpp) means the MIB never writes to that channel: no
settings, no heartbeat consumer, no NMT start, no enable, no move, no stop, no fault reset,
no calibration, no homing. Moves are refused with a message. The left main leg and both
carriages are not commissioned. Frames that can still reach node 2 after this change:

- explicit reads by name: `read_position`, `read_position_median`, `read_status`,
  `read_statusword`, `get_drive_state`, `limit_active`, `read_controller_settings`,
  `read_controller_limits` on a carriage's actuator (the example: `leg LCarr` then
  `status`, `settings`, `limits`, `flags`, `sdor 2 ...`);
- the example's `scan` (an SDO read of 0x1000 on every node id);
- the MIB heartbeat, a broadcast (COB-ID 0x77F) every node receives; node 2's consumer
  is never set, so it does not act on it.

Nothing writes to node 2: `init_actuators()` creates its client without sending, and the
example's `sdow` refuses nodes with no commissioned leg. To commission a carriage: tune it,
set `commissioned = true` and its tuning in MIBconfig.hpp, reflash.

## Where the limits live

1. `Actuator::clamp()` clips every target, and every streamed setpoint, to the range before
   it goes on the bus, and logs a clipped target.
2. The controller's position clamp (MinPos/MaxPos of the position PID record) forces any
   target into it. Factory value [0, 0]; a controller that reset comes back on its flash
   clamp. Before every fresh move the Actuator reads the record back and re-initialises a
   controller whose settings are gone.
3. CiA 402 0x607D, written equal to the clamp. Statusword bit 11 ("internal limit active")
   was seen set when the left encoder's glitch crossed the clamp.

The clamp limits targets, not measurements: an encoder glitch inside an enabled loop is an
error step the loop would act on. The left leg stays uncommissioned until that is fixed.

## Calibration and homing

| | Calibration | Homing |
|---|---|---|
| Encoder | absolute (main legs) | incremental AB (carriages) |
| When | once per installation | every boot, unless the saved position is restored; never automatic |
| Finds | the counts at the two mechanical ends | a known count at the limit switch |
| API | `set_calibration_mode`, `set_range` | `home`, `restore_position`, `save_position` |
| Persists | the range, in NVS | the last position, in NVS |

**Calibration**: `cal on` lifts the clamp (jogs only), jog to each end and read the count,
`range <min> <max>` installs and saves it, `cal off`. A range in NVS overrides the compiled
default at boot. Export the calibration as text and copy it into MIBconfig.hpp so a
replaced board boots right (planned: `export`/`import` console commands).

**Homing** (`home()`): lift the clamp, stream toward the switch at the homing profile in
segments, wait until the median count stops changing, stop. A drive that faulted is not the
switch: homing fails and leaves the fault for `clear_faults()`. Otherwise install
`home_count` with `set_encoder()` (drive disabled first; a live loop chases its old target),
restore the clamp, save the position. Never run against a real switch yet.

## Bench checklist (MCP266)

From the October bench, still current:

- Back up a controller's settings before letting Motion Studio touch its firmware: the
  public build may "update" it down to its bundled firmware and wipe them.
- After a firmware reset check: CAN mode CANopen, 1 Mbit/s, node id; encoder pin enabled
  and its type; main battery min/max (min ~21 V, max >= 30 V on 24 V; on the chair the bus
  is ~50 V, set accordingly); per-motor current limit; limit-pin functions.
- Battery limits too tight make a move fault at once (the sag at motor start).
- A bench supply cannot sink regeneration: 4,700 to 10,000 uF across the controller.
- Never use object 0x2041 (the buffered position command mirror): on the October bench it
  moved a motor outside the CiA 402 state machine, and disable voltage did not stop it.
- A drive left in SWITCH ON DISABLED by firmware blocks Motion Studio's PWM slider until a
  controller power cycle.

## Open

- The left main leg: fix its encoder, tune it (the right leg's settings are a guess), check
  its sign (+counts lowers the left side, the opposite of the right), then commission it.
- The carriages: confirm which channel is which, tune, test homing against a real switch,
  then commission.
- MibSystem: an actuator task calling `poll_actuators()`; coordinated moves of both main
  legs with the IMU as a supervisor (tilt changes ~2 deg per 1,000 counts of right leg).
- TPDOs for encoder and statusword (mapping entries all read 0; untested), to replace the
  per-tick SDO read.
- Return the espp dependency to a released version once esp-cpp/espp#911 is released.
