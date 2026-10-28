# Branch notes: feature/21-mib-debugger

This branch carries the bench debugger in `tools/mib_debugger` and the firmware
support it needs: the runtime parameter store (`mib_params.hpp`) and the raw
CAN bridge (`mib_can_bridge.hpp`) that lets the debugger program the MCP266
motor controllers over CANopen. **It is not intended to merge into `main`.** Everything the
debugger turned up that `main` should act on is tracked here, so nothing is lost
when the branch is eventually dropped. Keep this file current: when the debugger
reveals something new, add it under "Findings for main".

## Findings for main

Ordered by importance. Each entry says where the problem is on `main`, how it
was found, and how to confirm a fix with the debugger.

### 1. Joystick right turns the chassis left (sign error)

**Status: open on main.** This is the important one.

- **Where:** `components/MibSystem/src/mib_system.cpp`, `handle_joystick_message`,
  on `main` at line 170:
  ```cpp
  drive_controller_.set_target(sample.y, sample.x);
  ```
- **Why it is wrong:** `XYTwist.x` is defined in `joystick_message.hpp` as
  "+1 = right". The MIB passes it straight through as the angular velocity.
  `DifferentialDrive::compute` uses `left = linear - angular * half` and
  `right = linear + angular * half`, the standard convention where positive
  angular is counter-clockwise, a left turn. So a rightward stick produces a left
  turn.
- **Fix on main (one character):**
  ```cpp
  drive_controller_.set_target(sample.y, -sample.x);
  ```
  If the team would rather define positive angular as clockwise, flip the sign in
  `DifferentialDrive::compute` instead and leave the handler alone. Pick one and
  document it in `MIBconfig.hpp`.
- **How it was found:** the simulator reproduces the firmware math exactly; with
  a rightward stick the debugger's chassis-frame readout reports "turning left
  (CCW)".
- **How to verify a fix:** on this branch, set the `JOYSTICK_X_SIGN` parameter to
  -1 on the Parameters page and push the stick right; the debugger reports
  "turning right (CW)". On a fixed `main`, the same result must appear with no
  parameter change (and the parameter will not exist).
- **On this branch** the handler multiplies `sample.x` by the runtime parameter
  `JOYSTICK_X_SIGN`, default +1, purely so the bug can be demonstrated live.
  That parameter is a bench aid, not the fix.

### 2. Build breaks under ESP-IDF v6.1 (LVGL memory setting)

**Status: fixed on this branch in commit `cf16ccb`; safe to cherry-pick to main.**

- **Where:** `sdkconfig.defaults` sets `CONFIG_LV_MEM_SIZE_KILOBYTES=0`.
- **Why it is wrong:** LVGL 9.5, which the component manager resolves under
  ESP-IDF v6.1, derives its pool size from that symbol and fails the build with
  `LV_MEM_SIZE >= 2kB is required`. The adjacent `CONFIG_LV_MEM_SIZE=65536` is
  ignored by LVGL 9.x.
- **Fix:** `CONFIG_LV_MEM_SIZE_KILOBYTES=64`, matching the intended 65536 bytes.
- **Note:** CI pins ESP-IDF v6.0.1 and may resolve a different LVGL, which is
  why CI does not show this. Anyone building locally on v6.1 hits it.

### 3. CALIBRATE is a dead state

**Status: informational.**

- `SystemState::CALIBRATE` exists in `MibSystem` with a case in
  `run_state_step`, but nothing ever assigns it. No command, RTPS message or
  internal path enters it. Either add an entry path or remove the state so the
  state machine reflects what the firmware can do.

### 4. Joystick values are treated as metres per second

**Status: informational, worth a deliberate decision.**

- `handle_joystick_message` passes the normalised stick value (−1..+1) directly
  to `set_target` as linear m/s and angular rad/s. The trajectory planner then
  clamps to the active profile's maximum. The effect is that full stick reaches
  the profile limit only because every profile's limit is ≥ 1 in magnitude for
  linear; for the LOW profile's 0.35 m/s cap the stick saturates at 35 % travel.
  If the intent is "full stick = profile maximum", scale by the profile limits
  before clamping.

### 5. App partition headroom is 13 %

**Status: informational.**

- The binary is about 900 KB in a 1 MB app partition (`--flash-size 2MB`
  default). Adding features will overflow it. Decide the real flash size for the
  PCB and update the partition table before that happens.

### 6. Main's e-stop latch boots the MIB into ERROR; the debugger cannot clear it yet

**Status: open on this branch (2026-10-27).**

- `main` since PR 36 runs the estop-core latch, which starts latched, forces
  the state to ERROR while active, and DISARMs the wheels. The clear GPIO is
  `GPIO_NUM_NC`, so only a clear attempt on the `rammp/estop` topic releases
  it. The debugger has no e-stop panel, so against current `main` firmware
  Enable will never arm the wheels. It needs the `estop_message.hpp` layout
  from a newer `rammp-rtps` than this branch pins.

### 7. Speed-loop tuning of the PACE RACER boards needs no MIB change

**Status: informational, for the tuning work.** Also closes finding 1 on
main: PRs 24 and 35 fixed the turn direction there, so the branch's
`JOYSTICK_X_SIGN` parameter now demonstrates a fixed bug.

- The boards take their gains on `rammp/mib/motor_tuning/<axis>` and report
  them in `rammp/<axis>/motor_state` (motor API v3, `motor_message.hpp` at
  rammp-rtps `6e36aec`). Both are best-effort DDS topics on the chair's
  Ethernet, so the debugger's PACE tuning page writes and reads them directly.
  `feature/tune-pace-parameters` does the same from a serial console on the
  MIB; the two are alternatives, not dependencies.
- What does need the MIB: clearing a latched board fault, because the clear
  counter rides in the MIB's own MotorCommand stream. The tune branch's `clr`
  does it; the debugger cannot without a relay topic on the MIB.
- The MIB sends `accel_limit = 0`, so the board steps its reference and the
  only ramp is the MIB planner's. The boards' API doc names a stepped
  reference as the first cause of shaking; main may want a real accel_limit
  in every MotorCommand.

### 8. Actuator moves need no MIB change either; the debugger ports mib::Actuator

**Status: informational, for the seat work (2026-10-28).**

- The debugger's Actuators page (`tools/mib_debugger/actuator_ctl.py`) is a
  line-for-line port of `components/Actuator/src/actuator.cpp` on
  `feature/2-actuator-control`, over the raw CAN bridge: the Mcp266 enable
  sequence (mode written blind, fault reset, shutdown / switch on / enable),
  the profile-position set-point handshake, the clamp read-modify-write with
  the D, P, I setter order, 0x607D, the limit-switch refusal, and the
  set-encoder command with the loop taken down first.
- What it proves on the bench carries straight to the Actuator branch, and
  the two unverified objects it leans on are the same ones that branch flags:
  the mirrored set-encoder commands 22/23 and WRITENVM (command 94) over CAN.
  If either aborts on the real MCP266, both the page and the firmware need
  the Motion Studio route instead.
- Bench observations the Actuator branch's DESIGN.md already records and the
  page repeats in its notes: the MCP266 honours the per-move profile velocity
  but takes its ramp from the stored Accel/Decel, and too low a velocity makes
  the velocity loop cog.

### 9. The embedded RTPS profile allows four best-effort writers; the MIB needs six

**Status: fixed on this branch in `sdkconfig.defaults` (2026-10-28); main will hit it with its next publisher.**

- espp's rtps engine is built with a static limits profile. The embedded
  profile has `NUM_STATELESS_WRITERS=5`, one of which SPDP takes, so a
  participant gets four best-effort writers. Main already uses three (status,
  left and right motor command); this branch adds the parameter table, CAN rx
  and CAN status, and the last two fail at boot with "BEST_EFFORT writer
  capacity reached", after which the CAN bridge endpoints are missing.
- **Fix:** `CONFIG_RTPS_LIMIT_NUM_STATELESS_WRITERS=8` in `sdkconfig.defaults`
  raises only that pool (capacity-only, no wire change, a few hundred bytes).
  An existing `sdkconfig` does not pick up a changed default: edit it, run
  `idf.py menuconfig`, or delete it so it is regenerated.
- Main is one publisher away from the same wall (a seat-state or e-stop
  status topic would be the fifth). Readers have more headroom: three
  best-effort and two reliable against caps of five each.

### 10. espp's RTPS stack cannot complete endpoint discovery with CycloneDDS

**Status: root-caused and fixed on a host build of espp (2026-10-28). The
debugger no longer depends on it: since the same day it runs espp's own
RTPS stack from the `espp` wheel, so both ends share one implementation and
no board needs reflashing for the tool. The patch and test harness stay in
`tools/mib_debugger/interop/` for anyone pointing a CycloneDDS participant
or a ROS 2 node on the CycloneDDS RMW at the chair.** With an unpatched board
such a peer stays without samples for ever and prints `malformed packet
received from vendor 13.37`.

espp's rtps component (`components/rtps`, a de-vendored embeddedRTPS) was
only ever tested against FastDDS and ROS 2. CycloneDDS, which the debugger
uses, exercises four things FastDDS does not, and each one on its own is
enough to stop every sample. Reproduced on a Mac with espp's own
`pc/tests/rtps_interop_pub` and `rtps_interop_sub` host programs against a
CycloneDDS peer, with the ESP boards out of the loop entirely:

1. **Heartbeats addressed to the participant are dropped.** CycloneDDS sends
   HEARTBEAT (and GAP) with `readerId = ENTITYID_UNKNOWN`, which RTPS allows.
   `MessageReceiver::processHeartbeatSubmessage` looked the reader up by that
   id, found none, and discarded it. The board therefore never answered a
   heartbeat, and since CycloneDDS delivers discovery data only on request,
   the board never learned the laptop's readers. Fix: resolve the reader
   through the writer it is matched to, as the DATA path already does.
2. **The pre-emptive ACKNACK has sequence-number base 0.** CycloneDDS
   validates the base (must be at least 1) and discards it as malformed; it
   patches a base of 0 only for eProsima and RTI vendor ids. That is the
   terminal noise. Fix: base 1.
3. **ACKNACKs carry no INFO_DST and echo the heartbeat's reader id.**
   CycloneDDS resolves the writer by (destination prefix, writerId) and keys
   its reader table by the ACKNACK's reader id; without INFO_DST the prefix
   is null and with reader id 0 the entry is "not a connection", so even a
   well-formed ACKNACK did nothing. Fix: precede every ACKNACK with INFO_DST
   naming the writer's participant, and put the reader's own entity id in it.
4. **A received endpoint announcement without a locator keeps a placeholder
   address.** `TopicData`'s default constructor fills `unicastLocator` with
   `192.168.0.42:7411`, an embeddedRTPS leftover. CycloneDDS omits
   `PID_UNICAST_LOCATOR` from SEDP and relies on the participant's default
   unicast locator; the placeholder passed as valid, so once matched, every
   sample for a CycloneDDS reader went to 192.168.0.42. Fix: reset the
   locators before parsing and fall back to the remote participant's default
   unicast locator from SPDP (`SEDPAgent`).

With the four changes, the host harness passes espp writer to CycloneDDS
reader and CycloneDDS writer to espp reader, reliable and best-effort, with
no malformed-packet lines. Nothing changes on the wire for FastDDS peers:
INFO_DST before ACKNACK and base 1 are what FastDDS itself sends, and the
other two paths only run for input FastDDS never produces.

- **Where:** espp `components/rtps/src/messages/MessageReceiver.cpp`,
  `src/entities/StatefulReader.cpp`, `src/discovery/SEDPAgent.cpp`,
  `src/discovery/TopicData.cpp`. Same code in every espp/rtps release from
  1.3.5 to 1.3.10, so the chair never worked with CycloneDDS; what changed
  recently is only that the chair now has several boards and a laptop that
  looks at the terminal.
- **Getting it onto the boards:** the MIB and both PACE RACER boards pull
  `espp/rtps` from the component registry, so the fix has to come either
  from an espp release (PR to esp-cpp/espp from the fork) or from an
  `override_path` in each firmware's `main/idf_component.yml` pointing at a
  patched checkout. The PACE boards must be reflashed too: a board that
  never learns the laptop's readers sends it nothing.
- **How to verify:** `./doctor.sh --trace` must show MibStatus samples and
  "the MIB acknowledged this machine's endpoint discovery" OK, and the
  terminal must stay free of `malformed packet` lines.

## Firmware changes on this branch (not for main)

These exist only to support the debugger's Parameters page. They add about 320
lines across eight files. If a future decision is to bring runtime parameters to
`main`, use this as the starting point, but it was written for bench use and is
not persisted, not authenticated, and not reviewed for a production wheelchair.

| File | Change |
|---|---|
| `components/MIBConfig/include/mib_params.hpp` | New. Parameter table (X-macro), `mib::Params` mutex-guarded store seeded from `MIBconfig.hpp`, and the `ParamSet` / `ParamState` messages and topic names. |
| `components/DifferentialDrive/{include,src}` | Added `set_config()` so geometry and inversion can change at runtime. |
| `components/DriveController/{include,src}` | `profile_config()` reads limits from `mib::Params` instead of the constexpr table; new `apply_params()`; a mutex around the differential drive and active profile. |
| `components/MibSystem/{include,src}` | Reliable subscriber for `rammp/joystick/param_set`, best-effort publisher for `rammp/mib/params` at the publication rate, `handle_param_set` / `publish_param_state`, and the `JOYSTICK_X_SIGN` multiply in the joystick handler. |
| `sdkconfig.defaults` | Finding 2 above. This one *should* go to main. |

The message definitions for parameters live in this repository rather than in
the shared `external/rammp-rtps` submodule, deliberately, so the submodule is
untouched by this branch.

## Cherry-picking to main

Only the LVGL fix is meant for main:

```
git checkout main
git cherry-pick cf16ccb
```

The joystick sign fix (finding 1) is a hand edit on main, not a cherry-pick,
because this branch demonstrates the bug through a parameter rather than fixing
it.

### 5. The MCP266s need a CAN transport on main; what the bench found

**Status: informational, for whoever builds the MCP266 driver on main.**

- `main` has no CAN support yet (the README lists "Positional control with
  MCP266 motor controllers" as open). The transceiver is on TWAI TX GPIO 17 and
  RX GPIO 16; the MCP266s are assumed at 1 Mbit/s, node id 10 (espp's defaults,
  not yet confirmed on the real controllers).
- The espp components to build on are `espp/twai`, `espp/canopen` and
  `espp/mcp266`. This branch only pulls in `espp/twai`, for the raw bridge in
  `mib_can_bridge.hpp`; that bridge is a bench aid and must not be merged
  alongside a real driver, since two owners of one TWAI node cannot coexist.
- espp's testing of the MCP266 over CANopen found that **only CiA 402 profile
  position mode moves the motor**; the manufacturer speed and duty commands are
  accepted but inert. If main needs velocity control of these axes, plan for
  position moves with a profile velocity, or for packet serial over UART
  (`espp/basicmicro`) instead of CAN.
- The MCP266 reverts to its EEPROM settings at every power-up, and its factory
  position-PID clamp is `[0, 0]`, which forces every position target to zero.
  `espp::Mcp266::configure_position_loop()` handles both; a driver on main must
  call it once per boot.
- The object mapping the debugger's CAN page uses (index `0x2000` + packet-serial
  command number, subindices 1..N for multi-field commands, setter and readback
  field orders differing) is documented in `tools/mib_debugger/mcp266_objects.py`
  with a verified / unverified flag per row. Rows the bench confirms should be
  promoted to verified there, and any that abort should be corrected, so main's
  driver starts from tested addresses.
- **How to verify with the debugger:** open the MCP266 / CAN page against the
  board, read Device; a device name and firmware string prove the transceiver,
  bit rate and node id. Then read each settings group and note which abort.
