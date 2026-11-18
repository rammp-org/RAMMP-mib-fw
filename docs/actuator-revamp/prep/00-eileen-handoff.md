# Eileen's handoff, condensed (pasted by Alex 2026-11-17)

Source: claude.ai artifact "Actuator branch handoff", Nov 17 2026, by Eileen. Condensed, not verbatim.
Treat every "verified" below as a bench observation without a log: one controller, one gearmotor, firmware version not recorded.

## Branch state
- `feature/2-actuator-control` at 665a303, rebased and force-pushed 2026-11-17, 7 commits behind main, no PR.
- Tested on the bench motor through the example only. Not tested: carriage homing with the new engine, two actuators on one controller, the MibSystem side.
- The bench loop stalls short of target with "the owner's preferred gains"; every move ends with the stall warning and a release after 3 s. Accepted as-is.
- MibSystem leg code (LEG_MOVE, LEG_JOG, calibration, e-stop hold) lives on `feat/home-position`, cut from this branch earlier. Its working tree (Eileen's machine) holds an uncommitted `hold_after_move` version and a bench edit moving caster rows to node 10.

## What the Actuator does
1. Move = MIB-generated trapezoid streamed from `poll()`; one `set_target_position` per poll, about four SDOs.
2. Arrival = controller reports target reached, or count within `tolerance` and still (3 counts for 500 ms). 500 ms later the drive is released with CiA 402 disable voltage.
3. Stall release: not arrived 3 s after last setpoint, release where it stalled.
4. Rest watch: released joint read every 200 ms; if it moves more than `tolerance` from rest, move back and release again, at most once a second.
5. `stop()` is a quick stop. Homing drives the controller directly, no streaming.
6. Boot installs clamp, stored default speed and stored default accel.

## MCP266 "facts" (bench, 2026-11-16/17)
| Claim | Her consequence |
|---|---|
| Profile-position target is a step; 0x6083/0x6084 accepted, no ramp; stored default speed is the only cap | MIB generates the trapezoid |
| Stored default speed = cmd 70/71, 0x2046/0x2047 sub 0, u16. Default accel = cmd 68/69, 0x2044/0x2045 sub 1, u32. Decel (sub 2) inert. Not shown on Motion Studio's position page | installed every boot |
| Write-settings cmd 94 (0x205E sub 0, key 0xE22EAB7A) accepted but did not persist those two (read back sentinel 32768). Position PID record incl. deadband does persist from Motion Studio | no EEPROM write from firmware |
| Disable voltage (controlword 0) after a move takes PWM and current to 0 | that is the release |
| Cmd 65 mirror (0x2041) moves the motor even in "switch on disabled"; quick stop, disable voltage and mirrored duty 0 do not stop it. Only a CiA 402 move to current position plus disable voltage recovered it, twice at 4 A | never use cmd 65 |
| A drive parked in "switch on disabled" by firmware blocks Motion Studio's PWM slider; controller must be power-cycled with the MIB off the bus | bench workflow |
| P-only loop at P 0.106 stalls 90 to 1200 counts short against stiction, never raises target reached; flagged reached ~90 short with deadband 10 | stall release, tolerance 200 |
| Opening the MIB's USB serial port resets the board (USB-Serial-JTAG), even with DTR/RTS low | fresh boot per scripted session |
| Example's raw `sdor`/`sdow` bypass the Actuator mutex; SDO replies can mismatch during polling | stop first |

Readbacks she used: PWM at `0x2030` sub 1/2, motor current at `0x2031` sub 1/2, controlword `0x6040`.

## espp dependency
Needs `set_default_speed` / `set_default_accel`, in esp-cpp/espp PR 910 (fork `ewang360/espp:feat/mcp266-default-speed-accel`). Manifests point `espp/mcp266` and `espp/motor_controller` at fork branch `rammp/mcp266-defaults` as git sources. Component-manager git cache (`~/Library/Caches/Espressif/ComponentManager/b_git_*`) must be cleared after a force-push to the fork.

## Bench setup
- One MCP266, CAN node 3, M1, 24 V 50 RPM gearmotor, absolute encoder, travel about 10000 to 18000 counts.
- CAN 1 Mbit/s on TWAI tx 17, rx 16.
- MIB console: USB-Serial-JTAG, `/dev/cu.usbmodem5C372451831`.
- Controller gains: position P 0.106, I 0, D 0, deadband 50.
- Motion Studio slider fields must be re-typed (accel 1000, speed 3000, decel 1000) after Read Settings; zeros mean no limit.

## Safety notes from the handoff
- A move past the end of travel was commanded once; power had to be cut.
- Live loop while "switch on disabled": command abs to current position, wait 2 s, then write 0 to `0x6040`. Or power-cycle the controller.

## Her open items
- Rebase, PR, re-run the seven-step bench test.
- Nothing calls `poll()` in main firmware; `feat/home-position` must call it in IDLE only.
- Verify on the chair's controllers (different units, firmware unknown) that disable voltage cuts PWM and the stored speed caps moves.
- Tune chair loops: "the owner does not want an integral term, so the lever is P and a little D".
- Carriage homing still unramped.
- TPDOs for statusword/position never tested.
