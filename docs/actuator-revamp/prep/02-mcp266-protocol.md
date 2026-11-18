# MCP266 over CAN: what we can command, what we can read, and how we know

Prep note 02 for the actuator revamp. It separates three kinds of knowledge:
what the vendor documents, what espp implements and tests, and what someone
saw on a bench (or only believes). Nothing here was re-measured on hardware
for this note. Every claim carries a grade and a source.

Written 2026-11-17 against espp `v1.3.10-1-g30c51bd7`, open espp PR #910,
`origin/feature/2-actuator-control` @ `665a303` (the teammate's Actuator
branch), `origin/feature/21-mib-debugger` @ `e5040da`, and the vendor
documents listed below.

## Evidence grades

| Grade | Meaning |
|---|---|
| **V** | Vendor-documented. Cited by document and page. |
| **T** | Implemented in espp and covered by a host test. The host tests check object addresses, frame bytes and field remaps; they never check device behaviour. |
| **B** | Someone saw it on a bench. The source is named. **B\*** means a single uncorroborated note, or a note the same author contradicts elsewhere. |
| **?** | Assumed. Nobody has documented or observed it. |

## Sources and short names

| Short name | What it is |
|---|---|
| **[M p.N]** | *MCP Series User Manual*, Rev 1.2, "Firmware V1.1 and newer, Hardware V2 and newer", https://downloads.basicmicro.com/docs/mcp_user_manual.pdf (PDF created 2018-06-07, still the linked manual). Printed page numbers. Local copy: `/tmp/mcp-revamp/work-protocol/vendor/mcp_user_manual.{pdf,txt}`. |
| **[DS p.N]** | *MCP26x Data Sheet* v3.0, https://downloads.basicmicro.com/docs/mcp26x_datasheet.pdf |
| **[RC p.N]** | *RoboClaw User Manual* Rev 6.0 (firmware 4.3.0+), https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf. This covers the sibling product. It is used only where the MCP manual omits a command, and every such use is flagged. |
| **[BM-lib]** | Basicmicro's official libraries: github.com/basicmicro/basicmicro_python @ `da31ec7` and basicmicro_arduino @ `4083b93`. |
| **[espp]** | `/Users/alxchunlin/code-stuff/espp/components/{mcp266,canopen,basicmicro,motor_controller}`. |
| **[PR758]** | esp-cpp/espp#758. Its hardware-validation comment: ESP32-P4, node 10, 1 Mbit/s, M1 only. |
| **[PR842]** | esp-cpp/espp#842 (set encoder, PID readback). Says "not yet hardware-verified" for set-encoder. |
| **[PR910]** | esp-cpp/espp#910, **open**, by the teammate: default speed and accel. Its body records raw-SDO bench results. |
| **[ED-D]** | Teammate's `components/Actuator/DESIGN.md`, section "Bench findings" (Oct–Nov 2026, one MCP266, Dumore gearmotor with an AB encoder). |
| **[ED-c]** | Teammate commits on `origin/feature/2-actuator-control`, e.g. `46cc6e1`, `49506e8`. |
| **[DBG]** | `tools/mib_debugger/mcp266_objects.py` on `origin/feature/21-mib-debugger`. This is a guessed object table and marks most rows "Unverified". |
| **[bus_test]** | `/Users/alxchunlin/code-stuff/can-open-testing/espp/components/mcp266/bus_test/` plus `can_bus_test.py`. It instruments SDO latency, but **no run log is saved anywhere**. |

## 0. Identity and bus facts

- **The MCP266 is a Basicmicro product. (V)** The datasheet title is "MCP266 Dual Channel Programmable Brushed DC Motor Controller". Ratings: 10–60 V main, 6–12 V logic, 60 A continuous per channel [DS p.2, p.12].
- **CANopen and DS402 support are claimed, but only in one sentence. (V)** "The MCP can interface to a CAN bus using CANOpen protocol. The MCP supports the DS402 profile for motion control devices. ... CAN is available when the MCP is plugged into the USB" [DS p.7]. In practice the bench works with USB unplugged; the bus_test and teammate setups run without it (B). Treat that sentence as stale.
- **The manual's whole CANopen chapter is three settings. (V)** [M p.45 §1.9.4]:
  - "Priority Timeout – Disabled, CANOpen functions will execute at all priority levels."
  - Rate: 1 M, 800 k, 500 k, 250 k (default), 125 k, 100 k, 50 k, 20 k or 10 k bit/s.
  - Node ID: 1–127.
- **The vendor publishes no object dictionary, EDS, PDO map, list of supported CiA 402 objects, or description of the 0x2000 mirror. (V, by absence)** We searched the manual, datasheet, downloads page, GitHub org and the web; guessed EDS URLs return 404.
  - A forum thread (https://forum.core-electronics.com.au/t/mcp266-canopen/22960, Jul–Aug 2025) relays that Basicmicro supplied a *pre-release* Motion Studio with "more detailed CANopen setup". That build is still downloadable (https://downloads.basicmicro.com/software/BMStudioTest/BasicmicroMotionStudioSetup.msi, Last-Modified 2026-10-13); we did not download it.
- **Bus in use: 1 Mbit/s, 11-bit ids. (B)** Our config is TX GPIO 17, RX GPIO 16, nodes 1 (casters), 2 (carriages), 3 (main legs) [ED-c `665a303`, teammate `MIBconfig.hpp`]. PR758 used node 10.
  - Note the axis count. The teammate's config has **six** actuated legs on three MCP266s: FC, RC, ML, MR, LCarr, RCarr. The revamp brief says four axes. Settle the real count before the timing budget in §5 is used.
- **Two host paths reach the same controller:**
  - CANopen on the CAN pins.
  - Packet serial over USB or RS-232 [M §2.2 p.63ff].
  - The USB packet-serial port keeps working while CANopen is active. The teammate stopped a hunting CANopen-driven loop with Motion Studio's "Stop Motors" over USB [ED-D] (B).
  - Basicmicro's libraries also expose an **MCP-only CAN gateway over packet serial**: 181/182 put/get a raw CAN packet, 190/191 write/read a CANopen dictionary entry (node 0 = the MCP's own dictionary) [BM-lib `Basicmicro.h` L198–209, `commands.py`] (V-library, not in the manual).
- **This repo does not talk to any MCP today.** `main` and this branch have no CAN code. `DriveController` only computes wheel speeds ("a motor-command layer must consume wheel_speeds()"), and the drive wheels are commanded over RTPS to the `rammp::kAxes` boards at 20 Hz (`MibSystem`). The only MCP-over-CAN code is the teammate's Actuator branch and the bench tools.

## 1. Command table

Common details for every row:
- All SDOs are expedited (1, 2 or 4 bytes), on COB-ID 0x600+node for the request and 0x580+node for the reply.
- "M2 at +0x800" means the CiA 402 object for motor 2 is the M1 index plus 0x800 (0x6040 → 0x6840) [espp `mcp266_core.hpp`].
- "Mirror" means manufacturer object 0x2000 + packet-serial command number. A single-value command sits at sub 0. A multi-field command puts its fields at subs 1..N in that command's own packet field order (inferred; confirmed for 61–64 and 68).
- The espp API is `espp::Mcp266` unless another class is named.

### 1a. Network and CiA 402 state machine

| What | Object / frame | Payload, units | espp API | Grade and evidence |
|---|---|---|---|---|
| NMT start node | COB 0x000, `[0x01, node]` | none | `Mcp266::start()`, `CanopenClient::nmt_start()` | **T** (golden frame, `canopen_host_test.cpp` `test_nmt`). **B** [PR758] |
| NMT stop / pre-op / reset | COB 0x000, `[0x02/0x80/0x81/0x82, node]` | none | `CanopenClient::nmt_*` | **T** frame only. Device behaviour **?** |
| Controlword | 0x6040 (M2 0x6840), u16 | Shutdown 0x06, Switch on 0x07, Enable op 0x0F, Disable voltage 0x00, Quick stop 0x02, Fault reset 0x00→0x80 edge; bit 4 new set-point, bit 5 change immediately, bit 6 relative | `Ds402Drive::enable_operation/disable/quick_stop/fault_reset`, via `Mcp266::drive(axis)` | **T** (golden 0x6040 write frame; statusword decode vectors). **B** enable path [PR758]. Quick stop parks the drive in Quick-Stop-Active, which the standard enable walk cannot leave; transition 16 back to op-enabled is tried [ED-D, B\*] |
| Mode of operation | 0x6060 (0x6860), i8 | 1 = profile position (pp), 3 = profile velocity (pv); 9 csv, 10 cst are untried | written blind inside `Mcp266::enable()` | **B**: the MCP does **not** echo the mode in 0x6061, so `Ds402Drive::set_mode()` would time out; espp writes and then waits 25 ms (`kModeSettle`) [espp `mcp266.hpp` L547]. |
| Supported drive modes | 0x6502, u32 read | reads **0x0000_0700** = bits 8, 9, 10 (csv, cst, and bit 10, which CiA 402 reserves) | none | **B** [ED-D]; espp README says "advertises only the cyclic-sync modes". **pp (bit 0) is not advertised yet pp works**, so this object is unreliable both ways. |
| Fault reset | controlword 0x00 then 0x80 | | `Mcp266::reset_faults()` | **B** [PR758]; battery-low fault clears itself when the rail recovers [ED-D] |
| E-stop reset | mirror 200 → **0x20C8**:0, written as u8 1 (u32 fallback) | no payload in packet serial | `Mcp266::reset_estop()` | **V**: cmd 200 [M p.83]. The manual adds that 200 "will do nothing if the E-Stop is not first unlocked" (cmd 201 with 0xAA; locked by default). **T** address. **B** accepted [PR758]. Whether it *clears* a real latched E-stop over CAN: **?** |

### 1b. Position

| What | Object | Payload, units | espp API | Grade and evidence |
|---|---|---|---|---|
| Absolute profile-position move | 0x607A (0x687A) i32 target, then controlword 0x1F/0x3F (bit 4 rising, bit 5 immediate), wait for statusword bit 12, release bit 4, wait for bit 12 to clear | encoder counts | `move_to_position(axis, target, vel, acc, dec)`; `Ds402Drive::set_target_position(pos, ec, immediate, relative)` | **T** address (`0x607A+0x800 == 0x687A`). **B** [PR758]: 10000 → −10000 → 5000 → 0 reached; statusword 0x0637 at hold; endless ±10000 ping-pong. Teammate: a pp target is executed **as a step** (no ramp) capped by the stored default speed [ED-D "Final finding", B\*] |
| Relative profile-position move | same, controlword bit 6 set | counts | `set_target_position(..., relative=true)` | **?** Implemented, never run on hardware. Recommend not using it (§6, C9). |
| Streamed set-points (teammate's trapezoid) | repeated 0x607A + handshake from `poll()` | counts | teammate `Actuator::poll()` | **B\*** [ED-c `46cc6e1`]. Each set-point costs ≥ 4 SDOs: target, cw↑, ≥1 status poll, cw↓, ≥1 status poll. |
| Profile velocity | 0x6081 (0x6881) u32 | counts/s | `move_to_position` writes it | **B\*** [ED-D] "honoured when below the stored Speed" (100 counts/s visibly slower than 500). Only one observation. |
| Profile accel / decel | 0x6083 / 0x6084 u32 | counts/s² | `move_to_position` writes them | **B\*** [ED-D] accepted and read back, but **no effect** |
| Motion profile type | 0x6086 | reads 0 (linear) | none | **B\*** [ED-D] |
| CiA 402 software position limits | 0x607D:1/:2 (0x687D) i32 | counts | `set_software_position_limits()` | **?** Written by espp and by the teammate, never verified. Whether it aliases the PID MinPos/MaxPos clamp is unknown [ED-D says so; espp PR758 self-review flags M2 0x687D as untested]. |
| Position PID record, setter | mirror 61/62 → **0x203D / 0x203E**, subs 1..7, i32 | **D, P, I** (×1024), MaxI, Deadzone (counts), MinPos, MaxPos | `set_position_pid()`, `configure_position_loop()` | **V**: setter order D, P, I and "The defaults values are all zero" [M p.100 cmd 61/62]. The factory [0, 0] clamp is therefore *documented*, not folklore. "Position constants are used only with the Position commands, 65, 66 and 67 ..." [M p.100]. **T** remap (`position_pid_readback_to_setter` static_asserts). **B** [PR758], gains survive the remap [ED-D]. |
| Position PID record, readback | mirror 63/64 → **0x203F / 0x2040**, subs 1..7 | **P, I, D**, MaxI, Deadzone, MinPos, MaxPos | `read_position_pid()`, `read_position_limits()` (subs 6, 7) | **V** cmd 63/64. **T** address. **B** [PR758] |
| Buffered position with accel/speed/decel (native trapezoid) | mirror 65/66 → **0x2041 / 0x2042**, 5 subs: Accel, Speed, Decel, Position (4 B each), Buffer (1 B) | QPPS/s, QPPS, counts; buffer 0 = queue, 1 = execute now | none (not in espp) | **V**: cmd 65/66 `[Accel, Speed, Deccel, Position, Buffer]` [M pp.93–101]. Over CAN: **B\*, self-contradictory** [ED-D]. First "accepts every write and moves nothing", later "does move the motor, but runs outside the CiA 402 state machine: quick stop, disable voltage and a duty of 0 did not stop it; only a CiA 402 move to the current position took the loop back." See C3. |
| Stored default speed (cap for pp moves) | mirror 70/71 → **0x2046 / 0x2047** sub 0, u16 | 0–32767, a fraction of QPPS per [RC]; 32768 = "no limit" sentinel seen at power-up [PR910] | `set_default_speed()` in **PR910 only** | **V-RC only**: [RC p.82] "default speed ... used by the position commands". **Not in the MCP manual's command list [M pp.71–72].** **B** [PR910]: write at sub 0 reads back through 72; a write at sub 1 aborts. |
| Stored default accel | mirror 68/69 → **0x2044 / 0x2045**, sub 1 accel u32 (sub 2 decel accepted, no effect) | counts/s² (per PR910) | `set_default_accel()` in **PR910 only** | **V-RC only**: [RC p.82] 68/69 = default *duty* accel/decel used by commands 32–34. espp's own enum calls 68 `SetM1DefaultDutyAccel`. **B** [PR910]: a stored accel of 50 "visibly slowed a move's ramp". But [ED-D] "even with the stored accel at 1000 a CiA 402 move was a jolt". See C2. |
| Read default speeds / accels | mirror 72 → 0x2048; 81 → 0x2051 (4 fields) | | PR910 only | **V-RC** [RC p.83, p.85]. **B** [PR910] |

### 1c. Velocity and duty

| What | Object | Payload, units | espp API | Grade and evidence |
|---|---|---|---|---|
| Signed duty, M1 / M2 | mirror 32/33 → **0x2020 / 0x2021** sub 0, i16 | ±32767 = ±100 % [M p.93]. The manual says −32768 on pp.94 and 99. | `drive_duty(axis, duty)`, which first enables the axis in **pv** mode | **V** for cmd 32/33 over packet serial. **T** address. **B: accepted but inert over CAN** [espp README; PR758 "remains inert"; ED-D]. Untried: other drive states, mode 0, csv, NMT pre-op (see E5). |
| Duty both motors | mirror 34 → 0x2022 subs 1, 2 | | none | **V** packet serial; CAN **?** |
| Duty with accel | mirror 52/53 → 0x2034/0x2035 (Duty, Accel) | accel 0–655359 = "−100 % to 100 % in 100 ms" | none | **V** [M p.99, cmd 52/53; the manual gives the field width inconsistently]. CAN **?** |
| Signed speed (velocity PID) | mirror 35/36 → **0x2023 / 0x2024** sub 0, i32 | QPPS | `drive_speed(axis, qpps)` | **V** packet serial. **T** address. **B: inert over CAN** [espp README] |
| Speed with accel | mirror 38/39 → 0x2026/0x2027 (Accel, Speed) | QPPS/s, QPPS | none | **V** [M]. CAN **?** |
| CiA 402 profile velocity | mode 3, 0x60FF (0x68FF) i32 | counts/s | `Ds402Drive::set_target_velocity()` | **B: inert** [espp README: "The standard target objects ... leave the velocity generator idle"]. Not advertised in 0x6502. |
| CiA 402 cyclic sync velocity | mode 9, 0x60FF | counts/s | none | **?** Advertised in 0x6502 (bit 8). **Never tried.** espp README *guesses* it needs SYNC and PDO. |
| CiA 402 cyclic sync torque | mode 10, 0x6071 target torque (per-mille of 0x6076) | | none | **?** Advertised (bit 9). Never tried. Whether the MCP has a current loop to back it is unknown. |

### 1d. Encoder, persistence and settings

| What | Object | Payload, units | espp API | Grade and evidence |
|---|---|---|---|---|
| Set encoder count | mirror 22/23 → **0x2016 / 0x2017** sub 0, i32 | counts | `set_encoder(axis, count)` | **V** cmd 22/23 "Useful when homing" [M pp.89–91]. **T** address. **B** [ED-D]: rewriting the count while the loop is active makes the loop chase its old target ("set to 0 at 20000: the motor drove back to 20000"). So the write takes effect, but disable the loop first. [PR842] says not hardware-verified; the teammate's note postdates it. |
| Reset both encoders | mirror 20 → **0x2014**:0, written as u8 1 (u32 fallback) | none | `reset_encoders()` | **V** cmd 20. **T** address. Hardware **?** (the SDO width is a guess) |
| Velocity PID | mirror 28/29 → 0x201C/0x201D set (D, P, I, QPPS, 16.16); 55/56 → **0x2037**/0x2038 read (P, I, D, QPPS) | | `espp::Basicmicro` only (serial) | **V**. 0x2037 is one of the "verified anchors" in espp's `mcp266_core.hpp` comment (B). Writing it over CAN: **?** |
| Battery cut-off limits | 57/58 set, 59/60 read → 0x2039–0x203C | 0.1 V | none ([DBG] lists them) | **V** cmd 57–60, "can be set using ... packet serial/CANopen commands" [M p.12 §1.0.11]. CAN mirror **?** |
| Current limits | 133/134 set, 135/136 read → 0x2085–0x2088 (max, min) | 10 mA | none ([DBG]) | **V** [M pp.76–77]. CAN **?** |
| Pin functions (limit / home / E-stop) | 74/75 per [DBG], or cmd 143 actions bitmask [M p.79] | | none | **V**, partly: limit actions "stop at the maximum deceleration rate allowed" and block that direction; E-Stop stops both motors at maximum deceleration [M p.53]; wiring in §1.5.5 [M p.36]. CAN **?** |
| Write settings to EEPROM | mirror 94 → 0x205E | the manual shows `[Addr,94]` with no payload; [DBG] writes u32 0xE22EAB7A (a RoboClaw-style magic word) | none | **V** cmd 94 [M p.75]. **B\*** [PR910]: "accepted", but default speed and accel were back at 32768 after a power cycle. See C6. |
| Restore factory defaults | mirror 80 → 0x2050 | | none | **V** cmd 80. Do not use. |
| Serial / comm timeout | packet serial: cmd 14 (RoboClaw) or priority-level timeout (MCP); CANopen: 0x1016 / 0x100C / 0x6007 | | none | See §3.4. **No documented CAN comm timeout** [M p.45]. |

## 2. Feedback table

Every read below is **one SDO per field**: an expedited transfer carries at
most 4 bytes, and a multi-field mirror object exposes one sub-index per field.
No PDO has ever been configured or seen on this controller, and espp has no
PDO mapping code ("whether the MCP266 can push its statusword and position as
TPDOs is untested" [ED-D]). Heartbeat: `can_scan` and `bus_test` sniff
0x700+id frames, but no log records whether the MCP produces a periodic
heartbeat (0x1017) or only boot-up. The forum thread above saw a boot-up
`0x764 = 00` from a factory unit (B, third party).

Rate columns assume the SDO budget in §5. "Per-node SDO/s" means one SDO
in flight per node.

| Signal | Object | Units, resolution | espp API | Grade and evidence | Max rate, 1 field, 1 node |
|---|---|---|---|---|---|
| Position (encoder count) | **0x6064** (0x6864) i32 | counts | `read_encoder()` | **B** [PR758, bus_test, ED-D] | ≈ 1/RTT (§5) |
| Encoder count (mirror) | 16/17 → 0x2010/0x2011 sub 1 count, sub 2 status byte (underflow/dir/overflow) | | none | **V** packet serial [M p.89]. CAN **?** | same |
| Both encoders | 78 → 0x204E subs 1, 2 | | none | **V** packet serial. CAN **?** [DBG unverified] | still 2 SDOs |
| Velocity actual | **0x606C** (0x686C) i32 | counts/s; filtering and window unknown | `read_speed()` | **?** espp reads it, but nobody has recorded a value against a known speed | ≈ 1/RTT |
| Encoder speed (mirror) | 18/19 → 0x2012/0x2013 (pulses/s, direction byte); 30/31 raw speed over 1/300 s; 79 → 0x204F instantaneous speed | | none | **V** packet serial [M pp.89–91]. CAN **?** | |
| Motor PWM (applied duty) | **48 → 0x2030** sub 1 (M1), sub 2 (M2), i16 | ±32767 → ÷327.67 = % [M p.73] | none | **V** packet serial. CAN **B\***: [ED-c `46cc6e1`] says release was "verified by the controller's PWM and current readbacks", but no code in the branch reads them (it was done by hand with the example's `sdor`). Sub layout from [DBG], unverified. | |
| Motor current | **49 → 0x2031** sub 1, sub 2, i16 | 10 mA (÷100 = A) [M p.73] | none | same as PWM: **V** serial, **B\*** CAN | |
| Main battery | **24 → 0x2018**:0 u16 | 0.1 V | `read_main_battery_voltage()` | **V** [M p.72]. **T** address. **B** 20.1 V [PR758] | |
| Logic battery | 25 → 0x2019:0 u16 | 0.1 V | none | **V**. CAN **?** | |
| Temperature 1 / 2 | **82 → 0x2052**, 83 → 0x2053, u16 | 0.1 °C | `read_temperature()` (82 only) | **V** [M p.74]. **T**. **B** 25.1 °C [PR758]; 83 **?** | |
| Controller status / error flags | **90 → 0x205A**:0, u32 or u16 | bitmask; the two maps disagree, see C5 | none (teammate example `flags`) | **V** but internally inconsistent [M p.75]. **B\*** "always 0" with a limit pressed [ED-D] | |
| CiA 402 statusword | **0x6041** (0x6841) u16 | bit 10 target reached, bit 11 internal limit, bit 12 set-point ack | `read_statusword()`, `get_state()`, `is_target_reached()` | **T** decode vectors. **B** 0x0637 at target [PR758]. Bit 11: contradictory, see C4 | |
| Mode display | 0x6061 | | `Ds402Drive::get_mode_display()` | **B**: does not echo 0x6060 [espp] | |
| Error register / error code | 0x1001 u8; 0x603F u16; EMCY on 0x080+id | | `Ds402Drive::get_error_register()` | **?** No one has read them during a fault | |
| Position error / following error | RC cmd 114 "read position errors"; 0x60F4 | counts | none | **V-RC only** [RC p.77]; not in the MCP list. **?** on MCP | |
| Speed error | RC cmd 111 | | none | **V-RC only**. **?** | |
| Command buffer depth (for 65/66 moves) | 47 → 0x202F subs 1, 2 | 0 = last command executing, 0x80 = empty [M] | none | **V** packet serial. CAN **?** | |
| Limit-switch / input state | cmd 147 "Get Signal Values" [M p.82]; status bits | | none | **V** serial. Over CAN: [ED-D] says the switches are reported nowhere. See C4. | |
| "At target" | statusword bit 10 | uses the position PID deadzone (?) | `is_target_reached()` | **B** [PR758]. [ED-c] a P-only loop can stall 90–400 counts short and never raise bit 10 (bench P = 0.1) | |
| Position PID record / clamp | 0x203F subs 1..7 | | `read_position_pid()`, `read_position_limits()` | **B**. Doubles as the "controller rebooted" detector: the clamp returns to [0, 0] | |
| Device identity | 0x1000, 0x1008 (string), 0x1018:1–4; 21 → 0x2015 firmware string | | `read_device_info()` | **B** [bus_test discovery checks for "MCP266" in 0x1008] | |
| Full object dictionary | 0x1021 Store EDS (DOMAIN), or packet-serial 191 with node 0 over USB | | espp `ds402_panel.html` reads 0x1021 | **?** Never tried on an MCP. This is the cheapest way to settle most "?" rows (E2). | |

## 3. Controller-internal behaviour

### 3.1 Loops that run inside the MCP

| Element | Parameters | Settable over CAN? | Persistence | Grade |
|---|---|---|---|---|
| Velocity PID | P, I, D (16.16), QPPS (max counts/s) | read **B** (0x2037); write **?** | EEPROM via Motion Studio | **V** [M] |
| Position PID, outer loop on encoder counts, output = duty | P, I, D (×1024), MaxI, Deadzone, MinPos/MaxPos clamp | **yes**: 0x203D/0x203E (B) | RAM over CAN. Reverts to EEPROM at power-up (B, espp/PR758/ED-D). Factory values are all zero → clamp [0, 0] (**V** [M p.100]). | **V** structure |
| Position-move trajectory | packet serial 65/66 carry their own accel/speed/decel; 119/120 and 122/123 use the stored defaults (68–72) [RC p.101] | defaults via 0x2044/0x2046 (B, PR910); 0x2041 path contested | **not** persisted by cmd 94 on the tested unit (B\*, PR910) | **V-RC** |
| CiA 402 pp trajectory | 0x6081/0x6083/0x6084 | written; only 0x6081 has a (claimed) effect | n/a | **B\*** |
| Deadzone | position PID field 5 | yes | as PID | **V**: "the area around the specified position the controller will consider to be equal to the position specified" [M p.88] |
| Duty accel for duty commands | 68/69 (RC semantics), 52/53 inline | **?** | | **V-RC** |
| Current limit | 133–136, 10 mA | **?** | EEPROM | **V** [M pp.76–77] |
| Battery cut-offs | 57–60, 0.1 V | **?** | EEPROM | **V** [M p.12]. **B**: a min of 29 V on a 30 V supply tripped "main battery low" at motor start → CiA 402 FAULT, which self-cleared when the rail recovered [ED-D] |
| Limit / home / E-stop input actions | pin config (Motion Studio), cmd 143 bitmask | **?** | EEPROM | **V** [M p.53, p.79]. **B**: a *home*-configured pin latches "M1 home error"; a *limit*-configured pin blocks that direction without latching [ED-D] |
| PWM mode (locked antiphase / sign-magnitude) | 148/149 | **?** | EEPROM | **V** [M command list] |
| On-board MCL scripting | MCL program with stream access, including CAN ("GETS ... type 1 = CAN (CANOpen or raw CAN)") | upload via Motion Studio | flash | **V** [M p.161 and the MCL chapter]. Unused by us. It is a fallback for a custom CAN protocol with an on-controller watchdog. |

### 3.2 Things set only in Motion Studio (or packet serial), never over CAN so far

CAN mode, bitrate and node id; encoder pin enable and mode; velocity PID and
QPPS (autotune); battery limits; current limits; pin functions and polarity;
PWM mode; priority levels. All are **B** from [ED-D]'s "after a firmware
reset, check these" list. The table in §3.1 says which have a packet-serial
setter that *might* also be mirrored on CAN.

### 3.3 Observed behaviours worth designing around (all B or B\*, single unit)

- **An arrived loop hunts.** In Operation Enabled at its target, a geared motor dithers and whirs [ED-D]. The vendor explains this as I-gain hunting through gear slop, which deadzone suppresses [M p.88] (V).
- **A limit-switch stop leaves the position loop wound up**, and the next move away runs off. Restart the drive from Switch-On-Disabled first [ED-c `49506e8`].
- **Quick stop parks the drive** in Quick-Stop-Active [ED-D].
- **Writing the encoder with the loop active** makes the loop chase the old target [ED-D].
- **Disable voltage (cw 0x0000 → Switch-On-Disabled)** gives "zero duty, no loop" [ED-README]. This is the only CAN-side way we have to make the bridge output zero. The commit claims a PWM and current readback confirmed it; no artifact exists.

### 3.4 Comm loss: what happens when the CAN bus goes silent

- **Vendor:** packet-serial interfaces have a per-interface "Priority Timeout" ("Time in milliseconds new data must be received before timing out ... even if it is not a valid packet") [M p.44, p.46]. Priority levels fall back to the next input when one times out [M p.43, cmd 139 p.78]. For CANopen the manual says only **"Priority Timeout – Disabled, CANOpen functions will execute at all priority levels"** [M p.45]. RoboClaw's cmd 14 "Set Serial Timeout" ("both motors will stop automatically", 100 ms units) [RC p.79] does **not** appear in the MCP command list.
- **Implication (inference, ?):** there is no documented CAN watchdog. A pp target, once accepted, keeps being servoed indefinitely; the hunting observation implies the loop keeps running with no traffic. Standard CiA 301/402 comm-loss machinery may or may not exist: heartbeat consumer 0x1016, life guarding 0x100C/0x100D, abort-connection option 0x6007, error behaviour 0x1029. **Nobody has read them.**
- **The MIB sends no heartbeat today.** espp's `CanopenClient` consumes heartbeats but has no producer. Even if the MCP supports 0x1016, it has nothing to watch until we add one; that is a trivial periodic 0x700+id frame.

## 4. Open-loop duty mode for system identification

**Short answer: no confirmed way exists today to command raw per-channel duty
over CAN, and no documented CAN comm timeout exists either.**

| Candidate | Status | What it gives sysid |
|---|---|---|
| Mirror duty 0x2020/0x2021 in pv mode, Operation Enabled | **B: inert** (espp, PR758, ED-D) | none |
| Same mirror in other states: Switch-On-Disabled, mode 0, NMT pre-op, after changing priority levels | **?** never tried. Hypothesis: the CiA 402 layer overwrites the motor output each cycle, or the packet-serial duty path is gated by priority level (CANopen "executes at all priority levels", but the mirror may enter the packet-serial handler). | true open loop, if it works |
| csv (mode 9) with 0x60FF, streamed at 10–20 ms, with and without SYNC | **?** advertised, never tried | closed-loop *velocity* steps. Good for identifying the load and actuator given a known inner loop. |
| cst (mode 10) with 0x6071 | **?** advertised, never tried | current or torque steps, if backed by a real loop |
| **Packet serial over USB** (`espp::Basicmicro` or Basicmicro Python): cmd 32/33 duty, 52/53 duty+accel, while the MIB keeps CAN quiet | duty commands **V** [M p.93ff] and **T** (`basicmicro` golden frames). USB works while CAN is configured (B, ED-D). The **USB priority timeout** is the documented watchdog [M p.44] (V). Whether it zeroes the motors when no fallback level is set: **?**, needs a 1-minute test. | **true open-loop duty with a vendor-documented timeout.** The best bench sysid path today. Feedback (48 PWM, 49 current, 16/17 encoder, 18/19 speed) is available over the same link at packet-serial rates. |
| Position mode with logged PWM (closed-loop identification) | pp works (B). PWM readback 0x2030 **B\*** | the plant input (applied duty) is *measured*, so duty→position can be identified from closed-loop data with external excitation. No new firmware behaviour is needed. Read the controller gains (0x203F) for the controller model. |
| "Duty injection" through a P-only loop: stream target = pos + d/Kp each cycle | **?** hack; the Kp→duty scaling is unknown | an approximately constant duty within the MinPos/MaxPos clamp. Only worth trying if everything else fails. |
| On-board MCL script implementing a raw-CAN duty command with its own timeout | **V** capability [M MCL chapter]; **?** effort | a real duty mode on CAN with a watchdog that lives on the controller |

**Recommendation:** use USB packet serial for bench open-loop steps, and
closed-loop pp with PWM logging for in-chassis identification, while E3 and E5
settle whether a CAN-native velocity or duty path exists.

## 5. Timing

### 5.1 Fixed facts

- **Bitrate** 1 Mbit/s (B). The MCP supports up to 1 M (V).
- **One SDO = 2 frames.** An 8-byte classic frame with 11-bit id is 111 bits nominal and up to ~135 with worst-case bit stuffing, including inter-frame space. That is **0.11–0.135 ms per frame**, so **~0.27 ms of bus time per SDO**. Bus ceiling ≈ 3700 SDO/s at 100 % load. Plan for ≤ 50 % load: **~18 SDOs per 10 ms, ~37 per 20 ms, bus-wide.**
- **espp concurrency.** `CanopenClient` allows one SDO in flight per client (mutex + condition variable). One client per node, so different nodes can overlap only if separate tasks drive them. Every Actuator call is blocking, and `espp::Twai::transmit` waits for TX completion.
- **TX is single shot.** `tx_retry_count` defaults to 0 (espp `f2fd33d`/#806), so a frame lost to a bus error is not retried; the SDO then times out (100–200 ms in our configs). Consider `tx_retry_count` = 2–3 on the MIB.

### 5.2 Measured round trip: none recorded

- `bus_test` measures per-SDO latency (`lat_avg_us`, `lat_max_us`), but no output was saved.
- The teammate says "a few milliseconds each" and "two SDOs, a few milliseconds" [ED-D, README] (B\*, not measured).
- The wire floor is ~0.3 ms. The rest is MCP turnaround plus ESP task wake-up; the MCP firmware's turnaround is unknown and is the number that matters. **E1 measures it.**

### 5.3 Transactions per cycle

The table uses RTT r and assumes the bus is not the limit, which holds below ~18 SDO/10 ms.

| RTT r | 10 ms, one task for all nodes | 10 ms, one task per node (each node) | 20 ms, one task | 20 ms, per node (each) |
|---|---|---|---|---|
| 0.5 ms | 20, bus-capped at ~18 | 20 (bus-capped ~6 each for 3 nodes) | 40 (~37 cap) | 40 (~12 each for 3 nodes) |
| 1 ms | 10 | 10 | 20 | 20 (~12 each at the 50 % bus cap for 3 nodes) |
| 2 ms | 5 | 5 | 10 | 10 |
| 3 ms | 3 | 3 | 6 | 6 |

### 5.4 What a cycle needs

**Logging cost.** Per axis: position, PWM and current = 3 SDOs. That is 6 per
node, or 12 (2 nodes) and 18 (3 nodes) per cycle. With the statusword it is
16 and 24.

**Command cost.** A pp set-point costs ≥ 5 SDOs. A cyclic duty or velocity
write would be 1 SDO.

**Verdict at RTT ≈ 1 ms:**
- 20 ms with 2 nodes fits on one task.
- 20 ms with 3 nodes needs per-node tasks.
- 10 ms with full logging on 3 nodes does not fit over SDO at all.

Anything faster needs TPDOs, which are untested (E13), or fewer signals.

**Command-to-motion latency.**
- pp: target SDO + handshake ≈ 3–5 RTT before the set-point is acknowledged, plus the MCP loop period (unknown).
- Mode change: espp waits a fixed 25 ms after writing 0x6060.
- The enable walk is 3 controlword writes, each followed by statusword polls every 25 ms (`Mcp266::Config::poll_period`), so ≥ 75 ms from a disabled axis. Keep axes enabled for sysid, or account for this.
- **No measurement of command-to-first-encoder-motion exists. E1b measures it.**

## 6. Contradictions and unknowns, each with the experiment that resolves it

| # | Claim A | Claim B | Experiment |
|---|---|---|---|
| C1 | Teammate (Oct): "the MCP266 ignores the motion profile over CANopen" | Teammate (later, same doc): 0x6081 *is* honoured as a cap under the stored default speed; accel and decel are not | **E6.** Run a pp move of 20000 counts. Log 0x6064 every RTT and difference it to get velocity. Vary 0x6081 at {200, 1000, 2000}, with the default speed (0x2046) both above and below 0x6081. Then vary 0x6083 at {50, 500, 5000} and 0x2044:1 at {50, 500, 5000}. Cruise slope gives the effective speed; ramp slope gives the effective accel. |
| C2 | [PR910]: a stored default accel of 50 "visibly slowed a move's ramp" | [ED-D]: "even with the stored accel at 1000 a CiA 402 move was a jolt"; [RC p.82]: 68/69 are the *duty* accel for commands 32–34, not a position ramp | same **E6** grid. An effect seen only at very low accel may be duty-slew limiting, not a trapezoid. Look at PWM (0x2030) ramping versus a position-trajectory shape. |
| C3 | [ED-D]: 0x2041 (cmd 65) "accepts every write and moves nothing" | [ED-D] and [ED-c `46cc6e1`]: 0x2041 "does move the motor", outside CiA 402, and cannot be stopped by quick stop, disable voltage or duty 0 | **E7.** Bench with the motor free and a hand on the E-stop. Write 0x2041 subs 1–5 with buffer = 1 last. Check whether the order of sub writes or the buffer flag decides when it fires. Then test each stop path in turn: cw 0x0002, cw 0x0000, 0x2020 = 0, NMT stop, E-stop input. Read 47 (0x202F) for buffer state. **Until resolved, treat 0x2041 as unsafe.** |
| C4 | [ED-D]: limit switches are reported nowhere; statusword bit 11 is never set and cmd 90 is always 0 | [ED-c `49506e8`]: "statusword bit 11, which this controller does set once the pin is configured", and the shipped Actuator code relies on it (`limit_active()`) | **E8.** Configure a carriage pin as *limit*, then as *home*. Press it at idle, in Switch-On-Disabled, and in Operation Enabled while driving into it. Read 0x6041 and 0x205A (both u32 and u16) each time. Also try cmd 147 over USB. |
| C5 | espp `BasicmicroStatus` and [M p.75]: 16-bit map (0x0004 = E-stop, 0x0001/0x0002 = over-current warning) | Teammate example and [DBG]: 32-bit RoboClaw-style map (0x00000001 = E-stop, ...); [BM-lib] calls cmd 90 "GETERROR" (4 bytes) and 73 "GETSTATUS" | **E8b.** Read 0x205A as u32 and as u16 while forcing known conditions: E-stop input, battery-low via the min-voltage setting, temperature. Map the bits. Fix the losing decoder. |
| C6 | Teammate DESIGN: "the default-accel mirror (0x2044) is read-only" | [PR910]: 0x2044 sub 1 is writable and reads back through 81 | Probably a sub-index mistake (sub 0 aborts, sub 1 works). Recheck the abort code at sub 0 vs sub 1. This is low risk. |
| C7 | [PR910]: cmd 94 "accepted" but did not persist the default speed or accel | [DBG] writes 94 with u32 magic 0xE22EAB7A; [M p.75] shows `[Addr,94]` with no payload | **E10.** Write a distinctive PID deadzone (e.g. 7). Write 0x205E as u8 1, then as u32 magic. Power-cycle, then read 0x203F:5. Check whether 94 persists *some* settings but not the defaults; that would make the defaults RAM-only by design. |
| C8 | espp: supported modes 0x6502 = cyclic sync only, so "velocity likely needs csv + SYNC/PDO" | pp works although it is not advertised | **E3/E4.** Mode 9 with 0x60FF, then mode 10 with 0x6071. Stream asynchronously at 10 ms, then with SYNC (0x080) at 10 ms. Watch 0x606C, 0x2030 and the statusword. Read 0x60C2 (interpolation period) if present. |
| C9 | espp exposes relative pp moves (controlword bit 6) | no one has run one; the teammate defines relative on the MIB side instead | **E11b.** One relative move of +1000. Compare with absolute. Or simply never use bit 6. |
| C10 | espp and the teammate write both 0x607D and the MinPos/MaxPos clamp "to be safe" | whether they are one mechanism or two is undocumented | **E11.** Set the clamp wide and 0x607D narrow, command beyond 0x607D, and see where it stops. Then swap the two. |
| C11 | Teammate: `set_encoder` works (the loop chases the old target) | [PR842]: "not yet hardware-verified" | Already effectively resolved in favour of "works". Confirm with read-back in Switch-On-Disabled. |
| C12 | [DS p.7]: "CAN is available when the MCP is plugged into the USB" | the bench runs CAN without USB | Note only. Confirm once with USB unplugged during E1. |
| C13 | Brief: four actuated axes | teammate config: six legs on three MCP266s | Settle from the mechanical design. It changes §5's node and axis count. |
| U1 | SDO round trip: unknown | | **E1.** Flash `bus_test` with nodes {1,2,3}, run `can_bus_test.py --duration 60 --log ...`, and keep the log. Report `lat_avg_us`, `p50`, `lat_max_us`, then repeat with one node. **E1b:** time from the cw↑ SDO ack to the first 0x6064 change on a pp step. |
| U2 | Object dictionary contents: unknown | | **E2.** Segmented read of 0x1021 (and 0x1022 for its format) with espp `ds402_panel.html` over the CAN bridge. Failing that, run packet-serial 191 node 0 over USB across 0x1000–0x1FFF and 0x6000–0x6FFF, or run the panel's read-only scan. This settles PDO objects, 0x1016/0x1017, 0x6007, 0x6071, 0x60FF, 0x60F4 and 0x603F in one sitting. |
| U3 | Behaviour on CAN silence | | **E9.** (a) With the axis holding in pp, stop all MIB traffic and watch over USB (PWM, encoder) for 10 s. (b) If 0x1016 and 0x6007 exist, configure the consumer at 100 ms, add a MIB heartbeat, kill it, and observe the state and PWM. |
| U4 | PWM and current mirror layout | | **E12.** Run M1 at a known duty from Motion Studio or USB. Read 0x2030:0/1/2 and 0x2031:0/1/2 over CAN and compare with Motion Studio's display. |
| U5 | TPDO capability | | **E13.** Write 0x1A00 mapping (0x60640020, 0x20300110 ...) and 0x1800 (COB 0x180+id, transmission type 255 with event timer 10 ms, or type 1 on SYNC). Register an espp `register_tpdo_callback` and watch for frames. |
| U6 | Duty mirror in other states | | **E5.** 0x2020 = ±3000 in each of: NMT pre-op, Switch-On-Disabled, mode 0, mode −1, and after setting priority level 1 = packet serial with a timeout. Watch PWM over USB. |

## 7. Recommended sysid command and feedback set

### (a) Passive logging (no commands; safe on the chassis)

Per axis, every cycle, one task per node:
- **0x6064** position (i32)
- **0x2030:n** applied PWM (i16, ÷327.67 %), once E12 confirms the sub layout
- **0x2031:n** motor current (i16, ×10 mA)

**Cost:** 6 SDOs per node per cycle. That fits 20 ms on 3 nodes with per-node
tasks if RTT ≤ ~2 ms; at a 10 ms cycle drop current to every other cycle.

**Each sample** gets a MIB timestamp taken when its SDO reply arrives, not
when the cycle starts.

**Every 10th cycle:**
- 0x6041 statusword per axis (target reached, fault, state)
- 0x205A status flags, raw value logged (decode later per C5)

**At 1 Hz:**
- 0x2018 main battery
- 0x2052 temperature
- 0x203F:6/:7 clamp, as the reboot detector

**Velocity** comes from differenced position, not 0x606C, until 0x606C is
validated. Log 0x606C at 1 Hz alongside to check it.

### (b) Bounded open-loop step tests

**Bench, preferred today: USB packet serial on one MCP, CAN quiet.**
- **Command:** 32/33 signed duty (±32767), or 52/53 duty+accel to control slew. The step is a hold of N ms, then back to 0.
- **Watchdog:** the USB priority timeout set to ~100 ms [M p.44]. Verify first that it zeroes the bridge (U3-style one-minute test).
- **Feedback** over the same link: 16/17 encoder, 18/19 or 30/31 speed, 48 PWM, 49 current.
- **Bounds:**
  - MIB/host software stops on |Δpos| > travel window or |I| > limit.
  - Hardware current limit 133/134 set low.
  - Limit-switch pins configured as *limit* (stop at max decel, V [M p.53]).
  - E-stop input wired.

**In chassis, CAN only, works today: closed-loop pp excitation.**
- **Command:** 0x607A steps or a slow chirp of set-points within the calibrated range, with the clamp installed (`configure_position_loop`).
- **Read 0x203F once** to record the controller (P, I, D, deadzone).
- **Log the (a) set.** PWM is the measured plant input.
- **End every test** with cw 0x0000 (Switch-On-Disabled), not quick stop.

**If E3 or E5 succeeds: CAN-native steps.**
- **Command:** csv 0x60FF or duty 0x2020 written every cycle (1 SDO).
- **Feedback:** the (a) set.
- **Watchdog:** a MIB heartbeat plus 0x1016/0x6007 if E2 and E9 show they exist. Otherwise an MIB-side "write 0 and disable" on any missed cycle. That does not cover MIB death, so keep the E-stop in the loop.

### Do not use for sysid until resolved

- 0x2041/0x2042 buffered position (C3: reported unstoppable through CiA 402)
- relative pp (C9)
- 0x2014 reset encoders (width unknown)
- cmd 80 restore defaults
