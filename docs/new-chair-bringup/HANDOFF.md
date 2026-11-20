# New chair bring-up: handoff (main legs MR / ML, rear caster next)

Written 2026-11-20 by Alex's agent for Eileen and her agents. Branch:
`alexchunlin/new-chair-bringup` (stacked on #87 `alexchunlin/mib-leg-actuators`, which is on
#85). Everything below was measured on the **new** chair on 2026-11-19 over its Ethernet. The
old chair (`docs/actuator-revamp/`) is retired; **none of its numbers apply here.**

## State when this was written

| | |
|---|---|
| MIB firmware | `301832e` (this branch), slot ota_1, confirmed (not pending) |
| Commissioned | node 3 M1 = **MR** and node 3 M2 = **ML** only, with the gentle bring-up tuning below |
| Not commissioned | carriages (node 2), casters (not in this build at all) |
| MR / ML count | ~58,970 / ~60,860 (chair roughly level; lowest spot is ~58,200 / ~61,670) |
| Ranges saved in the MIB's NVS | MR 58,000..64,000, ML 60,000..66,000 (override MIBconfig at boot) |
| Battery | 53.9 V at the MCPs (fresh pack) |

## Safety rules (keep these)

1. **Never commission a leg with values from the old chair.** The old config would have clamped
   MR to 10,000..18,000 with the leg at 59,586: a 41,000-count run on the first move.
2. **The MCP266 clamp does not stop a motor.** With the count outside its clamp ("internal
   limit active", statusword bit 11) the MCP still drove at 6.6 % PWM and ~4 A. Clamps only
   limit targets. A runaway is caught by the host guard (below) or the E-STOP, nothing else.
3. **Position mode only.** The MCP266 (fw 1.4.9) does take duty (0x2020/0x2021, with the
   CiA 402 drive left at Switch on disabled) and cyclic-sync torque (mode 10, 0x6871), but in
   both modes the output survives controlword 0 **and** a heartbeat fault, even with fault
   reaction / disable / quick-stop option codes set to 0. Only fault reset + torque 0 + duty 0
   stops it. One test crept a leg ~2,300 counts on the old chair. Basicmicro is unresponsive.
4. **Every move under the guard.** `newchair_leg.py` / `newchair_both.py` / the jog page poll
   count, PWM and current over SDO every ~50 ms and latch the panel E-STOP on: speed > 900
   counts/s, > 80 counts the wrong way, |PWM| > 30 %, current > 20 A, stall (PWM > 5 % with
   < 15 counts of motion over 2.5 s), or the count > 100 outside the range.
5. **Motors are rated 10 A / 24 V** on a ~54 V bus. Lifting the chair draws 15-19 A per leg
   in 1-3 s bursts; keep 3 s rests between lifting moves and feel the motors. 30 % PWM is ~16 V.
6. Someone at the physical E-STOP for every move. Confirm directions by eye, not by IMU alone
   (an IMU-based guess was wrong once on 2026-11-19).

## Access (Ethernet only; no USB to the MIB)

- Plug into the chair's switch; the MIB is the DHCP server at 192.168.4.1. Service link is
  tcp :5555 (`mibnet.py` binds to whatever 192.168.4.x address the laptop has).
- Python with `pyserial` is not needed; stdlib only. All tools are in
  `tools/mib_panel/bench/`:

| Tool | Does |
|---|---|
| `mibnet.py` | Minimal service-link client (frames, request/reply) |
| `newchair_leg.py status` / `read <leg>` / `move <leg> <counts> [abs]` / `range <leg> <min> <max>` / `estop` / `clear` | One guarded move at a time; prints peak PWM, current, speed and the IMU tilt change; traces to `/tmp/newchair_moves/` |
| `newchair_both.py down|up <counts>` | Both legs together in ~250-count chunks, kept level by the IMU roll (gravity z, reference -1.68) |
| `legs_jog/legs_jog.py` | Browser jog page on http://127.0.0.1:8793/ (lift/lower 100-300 per press, E-STOP on Space/Esc, tilt from a level reference, 3 s rest after > 10 A) |
| `newchair_dump.py` | Read-only dump of nodes 3 and 2: counts, statusword, window, clamps, both PID records, QPPS, encoder pin, PWM, current |
| `ota_push.py <rammp_mib.bin>` | Flash the MIB over Ethernet (BEGIN/DATA/FINISH), then read its version. Rollback is on: an image that never confirms loses to the previous one |

The MIB serves **one** client at a time: stop the jog page before running a script, and the
reverse. Build with IDF v6.0.1 and python3.11 on PATH (repo memory `idf-build-env`).

## What this branch changes

| Commit | Change |
|---|---|
| `5198700` | Every leg uncommissioned (nothing written at boot). Service link: `0x0E SDO_READ [node][index u16][sub][bytes]` -> `0x8E SDO_VALUE`, and `0x0F SCAN` -> `0x8D SCAN_REPLY` (`mib_panel_protocol.hpp`). |
| `901fe14` | MR / ML commissioned with `bringup_tuning` (MIBconfig.hpp); ranges the read count +-300; 200 counts/s, 400 counts/s^2, lead 1000 ms, tolerance 50. **SCAN and probe reads disabled** (see bugs). |
| `301832e` | `imu` in the panel STATUS JSON: gravity (m/s^2), gyro (rad/s), calibration. |
| (this commit) | The bench tools above, the stored-settings dump, every move trace, this file. |

`bringup_tuning`: window 10, position P 1.0 (1024) I/D 0, velocity P 1.0 (65536) I 1701, QPPS
7143, profile velocity 20000. That is ~1.4 % PWM per 100 counts of error: deliberately weak.

## Measured on the new chair (2026-11-19)

**Stored on the MCPs as found** (Motion Studio values; full dump in
`data/node2_node3_stored_settings_2026-11-19.json`):

| | node 3 M1 (MR) | node 3 M2 (ML) |
|---|---|---|
| Count as found | 59,586 | 60,320 |
| Stored clamp (PID record and 0x607D) | 58,000..64,000 | 60,000..66,000 |
| Position P / I / D (x1024) | 153 / 0 / 1101 | 106 / 0 / 702 |
| Velocity P / I, QPPS | 78,010 / 2,696, **119** | 81,783 / 3,361, **165** |
| Window 0x6067 | 100 | 100 |
| Encoder pin 0x205B | 6 | 8 |

The stored velocity loop is ~1 % PWM per count/s of speed error: a step would slam. The MIB
overwrites these in RAM at every boot for commissioned legs (a power cycle of the MCPs alone
restores them: restart the MIB after one, then check `newchair_dump.py`). Node 2 (carriages):
counts 0, stock-like values; never written.

**Directions and loads** (confirmed by eye):

| Leg | Lowers | Lifts | Lifting draws | Lowering draws |
|---|---|---|---|---|
| MR, node 3 M1 (right) | -counts | +counts | 11-14 % PWM, 15-18.6 A | < 7 % PWM, < 4 A |
| ML, node 3 M2 (left) | +counts | -counts | 12-18 % PWM, 15-19 A | < 9 % PWM, < 7 A |

- Both loops pull toward the target (no reversed wiring).
- ML first would not lift: 22 % PWM at only 6-8 A (motor spinning free) until Alex engaged
  something on the left EHA. After that it lifted normally.
- ML occasionally jumps > 900 counts/s for one sample while lifting (the guard tripped once).
- Lowest spot (Alex): about MR 58,200 / ML 61,670.
- A level chair reads IMU gravity z about -1.68 (z is the roll axis on this mounting). Lifting MR
  lowers z, lifting ML raises it, ~0.00037 per count. ~1,000 counts on one leg is ~2 degrees.
- **The IMU (BNO055) is mounted vertically**: gravity is on +y. The firmware's roll/pitch logs
  do not mean roll/pitch on this chair; use the gravity vector or a BNO055 axis remap.
- Encoders: MR ~25 counts p-p at rest, ML ~100 counts p-p. Neither froze or jumped here (the
  old chair's node 3 encoders froze together twice; check for sd 0 if anything looks stuck).
- Every move trace (count, PWM, current, statusword at ~20 Hz, IMU before/after) is in
  `data/moves/`.

## Known bugs

- **Service-link SCAN crashed the MIB** (reset reason `panic`) on the first chunk. It builds
  a short-lived `CanopenClient` per id from the service-link worker task (the console's
  `scan_bus()` does the same from a bigger stack). Not diagnosed (no serial to read the panic).
  SCAN now replies ERROR and SDO_READ works only for nodes that have a configured leg.
- ML's encoder noise (~100 p-p) makes the loop flicker PWM +-4 % at rest during a move.

## Next steps

1. **Lifting tuning.** The bring-up gains reach lifting PWM only slowly (stall-guard bursts).
   Try position P 2.0 (2048, lead 500 ms) for both, then raise the speed cap from 200 toward the
   old requirement of 800 counts/s, watching current against the 10 A rating (keep moves short,
   rest between). Do it in `MIBconfig.hpp` (`bringup_tuning` -> a measured `new_chair_leg_tuning`)
   and flash with `ota_push.py`.
2. **Real travel.** The ranges are the controllers' stored clamps, not measured ends. ML's floor
   of 60,000 stops its lift before any mechanical end; MR's top is unknown. Use LEG_CALIBRATE
   (lifts the clamp; only jogs accepted) with short guarded jogs to each end, then LEG_RANGE.
   The lowest spot above is the bottom.
3. **Seat height.** Map leg counts to seat height (tape measure at a few counts, both legs kept
   level with `newchair_both.py`), then set `config/poses.json` and the panel's Go to Home.
4. **Rear caster (when its encoder is online).** On the old chair the casters were node 10
   ("MCP 1"), M1 front / M2 rear. Here node 10 is not configured, so nothing can read it:
   - Add FC / RC legs on node 10, `commissioned = false`, to `MIBconfig.hpp` (the old-chair
     branch `alexchunlin/old-chair-ml-experiments` has the entries; `leg_count` 6). Flash.
   - Read it with `newchair_dump.py` (add node 10): count, stored clamp and PIDs.
   - Encoder good = several hundred reads with sd > 0, no frozen value, no read-to-read jumps of
     hundreds of counts, and it follows the caster when moved by hand (drive off).
   - Then as for MR / ML: gentle tuning, range = count +-300, +-100 moves under the guard with
     someone watching to learn which way is which, then the lifting and range work.
5. **Fix SCAN** (bug above): run the scan in its own task with a bigger stack, or heap-allocate
   the probe; then re-enable it and SDO_READ for unconfigured nodes.
6. Before any of this goes to `main`: replace the bring-up comments and ranges in
   `MIBconfig.hpp` with the measured ones, use the repo's PR template, and include the data and
   figures (repo practice).

## Old chair work (for reference)

Branch `alexchunlin/old-chair-ml-experiments`: the old chair's left-leg commissioning, an input
shaper in the Actuator (`MotionProfile::shaped_setpoint`, host-tested; it did **not** help the
old left leg), the example's `trace` command with IMU, the casters on node 10, and
`tools/encoder_scope` (USB-console encoder plotter with a speed guard). Findings are in the
repo memory notes and `docs/actuator-revamp/bench-findings.md`.
