# Bench findings, chair, 2026-11-17

Measured on the chair with `tools/actuator_sysid/firmware` (logs in `docs/actuator-revamp/runs/`). Chair on the ground. Each line says how it was established.

## Wiring and identity
- Nodes on the bus: 2 and 3 only (read-only `scan`). No node 1: this chair has no powered casters (Alex).
- Both are `MCP266 2x60A`, software version 1.4.9 (`dump`).
- **Node 3 M1 = RIGHT main leg.** +counts = EHA stroke retracts; -counts = stroke extends = wheel up = right side of the chair lowers (Alex watched runs 05; he corrected his first report, and the linkage inverts stroke vs wheel). Eileen's MIBconfig has M1 as left: wrong.
- **Node 3 M2 = LEFT main leg.** +counts = stroke extends = wheel up = left side lowers (Alex watched, run 05). **The two legs have opposite signs**: +counts lowers the left side but raises the right side.
- Node 2 = carriages. Never written. Both counts read 0 after a power cycle; M2 clamp [0, 0], M1 clamp [-80000, 80000].
- MIB console is the CH343 UART bridge (`/dev/cu.usbmodem5C372451831`), not USB-Serial-JTAG. Opening the port resets the board. 921600 baud carries 100 Hz rows with no drops.
- Bus voltage about 50 V.

## Stored controller settings (node 3, after a power cycle)
| | M1 (right) | M2 (left) |
|---|---|---|
| Count at rest | ~13,560 | ~30,040 |
| Clamp (PID record) and 0x607D | 10,000..18,000 | 25,000..32,000 |
| Position P / I / D (x1024) | 103 / 0 / 868 | 120 / 0 / 1086 |
| Deadband | 10 | 10 |
| Velocity PID P / I (16.16), QPPS | 55400 / 1701, 176 | 51182 / 737, 352 |

## Controller behaviour
- **Clamp read quirk**: 0x203F/0x2040 subs 6,7 return 0 unless the read starts at sub 1. espp `Mcp266::read_position_limits()` reads only 6,7 and so reads 0 on this firmware (worth an upstream issue).
- **0x2046/0x2047, 0x2044/0x2045** (Eileen's stored default speed/accel) return protocol error / abort 0x06010001 on read. 0x6007 not supported.
- **PDO** comm parameters exist with default COB-IDs; every mapping entry is 0.
- **Supported modes 0x6502 = 0x700** (cyclic-sync velocity/torque bits), yet profile position works.
- **A position target takes effect when the drive reaches OPERATION ENABLED**, from whatever is in 0x607A. Run 03: target written after enable + new-set-point handshake, controller acknowledged, PWM stayed exactly 0 for 2 s with "target reached" set and 85 counts of error. Runs 04/05: 0x607A written before enable, PWM started within 10 ms of enable. (Not yet separated: whether the handshake alone ever starts a move on this firmware.)
- **No ramp.** +400 step: 23% PWM and 11.8 A within 40 ms, 170 counts in ~0.15 s (peak ~1,600 counts/s), chassis jolt ~6 deg/s.
- **Stalls short.** Same step settled ~195 counts from target at ~3% PWM and ~3 A, not moving, for 4.5 s. Breakaway needs more than about 3 to 5% PWM. Statusword showed target reached while 200 counts away.
- **Disable voltage (controlword 0) stops it**: PWM 0 within 100 ms, leg stays put (EHA not back-drivable).
- **Heartbeat consumer works**: with 0x1016:1 set, losing the MIB heartbeat faulted the enabled drive ~470 ms later (statusword fault, flag 0x10000000). Writing 0 to 0x1016:1 did not stop it watching.
- **SDO transmit errors while the MIB heartbeat task ran**: about half of console-task SDOs failed (I/O error from the TWAI transmit). Cause not found. Heartbeat is off in the test firmware for now.
- Node 3 misses about 1 SDO reply in 400; node 2 about 1 in 25. No bit/stuff/ack errors; the bus error count is all arbitration-lost.

## Signals
- Encoder noise at rest: ~50 to 70 counts peak to peak (sd ~8) on both legs, against a 10-count deadband.
- M2 encoder glitched twice by ~1,800 counts, decaying over ~100 ms, chair untouched (run 02, t = 21.4 s and 23.4 s).
- 0x606C speed readback is unusable (toggles 0/±1000/±2000 at rest, sometimes INT_MIN). Use encoder differences.
- PWM (0x2030) and current (0x2031 x10 mA) read plausibly during motion.
- IMU: 200 counts of right-leg travel changed tilt by ~0.2 to 0.3 deg. Calibration status reads sys 0, accel 0.

## Test firmware lessons
- Stop must be an immediate disable voltage. The earlier "hold at current count, wait 2 s, then disable" (from the handoff's recovery recipe) let the leg run on and reversed it.
- A guard trip is queued behind the move's SDO sequence; fine so far (7 ms), unbounded in principle.

## Left leg (M2), +400 step, run 05 t=340 s
- PWM sat at ~13% for ~0.12 s before the leg broke away (current fell from 6.7 A to 0.5 A as it started), then it ran fast (order of 3,000+ counts/s), was braked at -14%, bounced, and overshot the target by 145 counts. The travel guard stopped it at 0.64 s. Net +306 counts.
- After the drive was disabled the encoder and the gyro kept ringing for ~0.3 s at roughly 3.5 Hz, +-130 counts. So there is a lightly damped mode (tyres / linkage / chassis on the ground) inside the position measurement. The loop's D term fights it at +-14% PWM.
- Tilt change for +306 counts on the left leg: about -0.4 deg and -0.65 deg on the two board axes. Right leg -240 counts: -0.23 and +0.41. One axis has the same sign for both legs, the other opposite.

## Left leg encoder glitches (hardware)
- M2's encoder jumps by +-1,500 to 2,000 counts and decays back over ~100 ms, about once every 9 to 12 s, with the drive off and the chair still: 3 in 30 s (run 02), 29 in 342 s (03), 32 in 358 s (04), 42 in 378 s (05). M1: none in ~1,100 s.
- A glitch that crossed the clamp made the controller raise "internal limit active" (statusword bit 11) for ~50 ms.
- With the loop closed a glitch is a 1,800-count error step: the controller would drive hard the wrong way for ~100 ms. The position clamp does not protect against this (it limits targets, not measurements).
- Encoder mode object 0x205B reads 6 for M1 and 7 for M2 (same on node 2). Meaning not decoded yet.

## Right leg gain experiments, run 06 (2026-11-17, ~17:10 to 17:25)
Position record written as P,I,D,MaxI,deadband in espp's readback order; profile velocity/accel 500/500 on every step.

| Gains (raw) | Step | Moved | Stopped short by | Peak PWM / current | PWM once stopped | Chassis rate |
|---|---|---|---|---|---|---|
| 103,0,868, db 10 (stock) | +300 | +197 | 105 | 17% / 10 A | ~3.8% | 5 deg/s |
| 205,0,0, db 10 | -300 | -211 (reached target in 0.12 s, bounced back 100) | 87 | 33% / 18 A | ~0% | 14 deg/s |
| 205,0,0, db 0 | +150 | +99 | 60 | 18% / 13 A | ~0.7% | 6 deg/s |

- With D = 0 the output is still a short burst at the start and about zero afterwards, with 60 to 95 counts of error left. So the burst is not a derivative kick, and the output is not P x error.
- The burst scales with step size and reaches about (P x step) / 176 as a fraction of full PWM; 176 is this channel's velocity-loop QPPS.
- Working hypothesis (not yet tested): the controller runs its own trajectory and feeds planned speed forward as PWM = speed / QPPS. QPPS = 176 counts/s is about 50x too low for this leg (it moves at 1,000 to 3,000 counts/s on 17 to 33% PWM), so a planned 30 to 60 counts/s becomes a 17 to 33% burst, the leg arrives almost at once, the plan ends, and the weak position gains cannot close the remainder.
- Tests that would settle it: same step with a lower profile acceleration (burst should shrink in proportion); then a realistic QPPS.

## CAN transmit failures (found and worked around)
- The ESP-IDF TWAI driver reports some frames as not sent (TX buffer freed without success), about 1.6 per second on this bus, with no bit/stuff/ACK errors counted. espp's CanopenClient turns each into an SDO I/O error. That was the cause of the refused clamp/gains commands and of most "SDO timeouts" in polling (node 2: 4% of passes).
- Resending the same frame up to 4 times in the test firmware's transmit function took failed poll passes to 0 on both nodes. `tx_retry_count` did not help (3 or 15); 30 is rejected by espp's own range check (-1..15).
- The earlier link to the heartbeat task was coincidence.

## Decisive tests on the right leg, run 06 (~17:45 to 17:55)
Position gains P,I,D raw; velocity record P,I,D,QPPS raw; profile = 0x6081 velocity / 0x6083 = 0x6084 acceleration.

| Pos P | QPPS | Profile vel / acc | Step | Onset PWM (first 100 ms) | Peak PWM / current | Left short by | Chassis rate |
|---|---|---|---|---|---|---|---|
| 205 | 176 | 500 / 500 | +150 | 10.6% | 18.2% / 12.8 A | 60 | 5.8 deg/s |
| 205 | 176 | 500 / 500 | -150 | n/a | 16.9% / 10.0 A | 37 | 7.2 deg/s |
| 205 | 176 | 500 / **100** | +150 | 10.3% | 17.6% / 6.7 A | 27 | 4.5 deg/s |
| **1024** | **1760** | 500 / 500 | -150 | 7.5% | 11.9% / 6.0 A | 11 | 4.1 deg/s |
| 1024 | 1760 | **100** / 500 | +300 | 5.9% | 12.5% / 9.8 A | 28 (still closing at ~60 counts/s) | 3.1 deg/s |
| 1024 | 1760 | **2000** / 500 | -300 | 12.2% | 20.3% / 13.5 A | 23 | 8.1 deg/s |

- **Profile acceleration (0x6083/0x6084) has no visible effect**: 500 vs 100 gave the same burst. The "internal trajectory with a ramp" idea is refuted in that form.
- **The loop is cascaded.** Position error x P is a speed command in counts/s; PWM is roughly speed command / QPPS plus the velocity loop's own PI on (commanded - measured speed). Onset PWM matches P x error / QPPS in every row (e.g. 1.0 x 300 / 1760 = 17%, measured 20% peak).
- **Profile velocity (0x6081) caps the speed command.** With the cap at 100 counts/s the onset PWM was 100 / 1760 = 5.7% predicted, 5.9% measured, and the leg then crawled toward the target at roughly 60 counts/s on a steady ~5% PWM: the first controlled-speed motion seen today.
- **Why stock settings jolt and stall**: with QPPS = 176 the controller believes full speed is 176 counts/s, while the leg does ~1,000 to 2,000 counts/s on 10 to 20% PWM. Speed commands of a few tens of counts/s become large PWM, the leg overshoots the commanded speed at once, the velocity loop brakes and its integrator winds the wrong way, and the output ends near zero (sometimes the wrong sign) with error left.
- State left in controller RAM on node 3 M1: position 1024,0,0,0,0, clamp 13264..14064; velocity 55400,1701,0,1760. A controller power cycle restores the stored values.

## Velocity loop structure and plant map (run 06, ~18:00 to 18:15)
- Velocity record with P = I = 0: PWM stayed exactly 0 on a 500-count error. So there is **no feedforward**; PWM = (velocity P / QPPS) x (speed command - measured speed) + integral. The onset bursts seen earlier were the velocity P term, and the wrong-sign output after a burst is that loop braking.
- Right leg, going up (+counts, lifts that side): stalls below about 4.5% PWM drawing 4 to 5 A; ~500 to 800 counts/s at 7%; ~1,500 counts/s at 9.5%. Roughly 280 counts/s per % above a 4.5% dead zone. Going down: only a few samples; it ran up to ~3,700 counts/s on a burst (load assists).
- With velocity P 65536, I 0, QPPS 7143 and position P 2048 (PWM = 0.028% per count of error, less a speed term): +500 step cruised at ~650 counts/s on 6.9% PWM, 4 deg/s chassis rate, and stalled ~100 short at 3.5 to 4.5%.

## Live target updates (option 2), run 06 ~18:20
Target walked -300 counts at 100 counts/s, gains as in the line above.
- **0x607A written alone, every 20 ms: ignored.** 244 writes accepted (slowest 1.2 ms), PWM stayed 0 with 280 counts of error.
- **0x607A + new-set-point handshake (espp `set_target_position`): followed.** The leg tracked the ramp about 85 counts behind in small stick-slip hops, peak PWM 5.9%, current mostly ~1 A, chassis rate at most 1.5 deg/s (steps today gave 4 to 14). It ended 44 counts short.
- Update cost with espp's handshake: ~50 ms each (97 updates in 5 s), because it sleeps 25 ms between statusword polls. A leaner handshake has not been tried.
- **Wake-up dead zone**: no output at all until the error passed ~95 counts, although the configured deadband was 0; once moving it kept driving down to ~45 counts. This also explains run 03 (100-count step, zero output) better than the earlier "target only latches at enable" reading, which should be treated as unproven. Matches Eileen's "flagged reached about 90 counts short".

## The stall zone is the CiA 402 position window (run 06, ~18:35)
- `0x6067` (position window) reads **100 counts** on node 3. Inside it the controller stops commanding speed: that is the "stalls ~90 short", the 100-count step that did nothing, and the wake-up dead zone. It is writable over CAN (set to 10, read back 10). 0x6068 (window time) and 0x6066 are not implemented.
- Other objects that answer: 0x6062 position demand, 0x60FC position demand internal, 0x606B velocity demand, 0x60F4 following error, 0x6080 max motor speed (reads the profile velocity last written), 0x607F max profile velocity = 32768, 0x6085 = 655360, 0x60C5/0x60C6 = 500000. Worth logging 0x6062 and 0x606B during a move to see the controller's internal demand directly.

### First clean moves: streamed ramp, right leg, 300 counts at 100 counts/s
Settings: position window 10; position P 2048 (2.0), I 0, D 0, deadband 0; velocity P 65536 (1.0), I 1701, D 0, QPPS 7143; target + handshake every 50 ms from the MIB.

| Direction | Tracking lag | Speed during ramp | PWM / current during ramp | Ended | Peak chassis rate |
|---|---|---|---|---|---|
| Down (-300, load assists) | ~35 counts | 70 to 116 counts/s | ~1.5% / ~1 A | 11 counts from target | 2.0 deg/s |
| Up (+300, lifting) | ~50 counts | 95 to 132 counts/s | ~5.2% / ~5 A | 2 counts from target | 1.7 deg/s |

For comparison, the same up ramp with the window at 100 lagged ~90 counts and stopped 57 to 85 short, holding 5.5% PWM. Stock-gain steps today: 4 to 14 deg/s, 10 to 18 A peaks, 60 to 200 counts short.
- At rest against the load the loop holds ~4 to 5% PWM and ~4.5 A while enabled, so releasing the drive after arrival (as Eileen's code does) is still right for these non-back-drivable EHAs.
- Not yet tried: stock gains with only the window changed; faster ramps; an acceleration-limited ramp; moves longer than 300 counts; the left leg.

### Ramp speed sweep and the stock-gains comparison (run 06, ~18:45)
300-count streamed ramps, right leg, position window 10, 20 Hz target updates.

| Gains | Ramp | Direction | Lag | Ended (counts from target) | Peak chassis rate |
|---|---|---|---|---|---|
| Tuned | 100 | down / up | 35 / 50 | 11 short / 2 short | 2.0 / 1.7 deg/s |
| Tuned | 200 | down / up | 65 / 105 | 4 over / 9 over | 1.5 / 1.5 deg/s |
| Tuned | 400 | down / up | 100 / 140 | 12 over / 16 over | 2.7 / 1.7 deg/s |
| Stock + window 10 | 100 | down / up | grows to 155 / 165 | 140 short / 147 short | 1.3 / 1.2 deg/s |

Tuned = position 2048,0,0,0,0; velocity 65536,1701,0,7143. Stock = position 103,0,868,0,10; velocity 55400,1701,0,176.
- The tuned loop holds the commanded speed at all three rates and ends within encoder noise (+-30 counts).
- Stock gains with only the window changed are gentle under a streamed ramp but cannot keep up (about 60 counts/s) and stop ~145 short. So the window and the gains are both needed.
- Up moves hold ~5 to 6% PWM / ~5 A during the ramp; down moves ~2 to 3% / ~1 A.
- Left in controller RAM on node 3 M1: tuned gains, window 10, clamp 13264..14064. M2: stock gains, clamp 29890..30590.

### 1,000-count moves at 800 counts/s (run 06, ~19:00)
Right leg, between ~12,840 and ~13,840. MIB streams a trapezoid (800 counts/s, 1,600 counts/s^2) with the handshake every 50 ms; position window 10; velocity 65536,1701,0,7143.

| Position P | Direction | Peak speed | Lag at speed | Ended (counts from target) | Settled by | PWM / current at speed | Peak chassis rate |
|---|---|---|---|---|---|---|---|
| 2.0 | down | 685 | ~290 | 4 | ~3.8 s | 4.6% / 1.2 A | 3.1 deg/s |
| 2.0 | up | 717 | ~300 | 5 | ~4.0 s | 8.7% / 5.5 A (peak 13%, 8.8 A) | 2.3 deg/s |
| 4.0 | down | 824 | ~150 | 1 | ~3.1 s | 5.5% / 1.5 A | 2.9 deg/s |
| 4.0 | up | 800 | ~150 | 3 | ~3.2 s | 9.1% / 6 A (peak 14.5%, 10 A) | 2.8 deg/s |

- P = 4.0 reaches the required 800 counts/s, halves the lag and shows no ringing at the 3.5 Hz mode.
- Requirement from Alex: 800 counts/s maximum.
- Left in controller RAM, node 3 M1: position 4096,0,0,0,0, clamp 12700..14000, window 10, velocity 65536,1701,0,7143.

### Full-range check, right leg (run 06, ~19:15 to 19:30)
20 streamed trapezoid moves between 10,530 and 17,420 (stored clamp is 10,000..18,000), tuned settings with position P 4.0. Slower moves (200 to 400 counts/s) were used only for the first pass upward and the approaches to each end.

| Region | Direction | Speed | End error (counts) | Lag at speed | Peak PWM / current | Peak chassis rate |
|---|---|---|---|---|---|---|
| 10,530 to 13,560 | up | 800 | 8 to 13 over | 155 to 208 | 17.6% / 14.3 A | 2.9 deg/s |
| 13,580 to 16,630 | up | 800 | 8 to 22 over | 186 to 195 | 18.3% / 14.3 A | 3.4 deg/s |
| 17,420 to 13,390 | down | 800 | 2 to 9 short | 154 to 163 | 10.7% / 6.0 A | 3.4 deg/s |
| 13,380 to 11,350 | down | 800 | 16 to 17 short | 146 to 163 | 10.2% / 5.9 A | 2.9 deg/s |
| 16,915 to 17,417 | up | 200 | 2 over | 54 | 11.1% / 10.4 A | 1.8 deg/s |
| 10,872 to 10,534 | down | 200 | 12 over | 50 | 7.5% / 5.1 A | 1.6 deg/s |

- No guard tripped, no fault, no ringing anywhere in the range. Up moves consistently end 10 to 20 counts past the target; down moves 2 to 17 short of it (i.e. both end slightly high in counts).
- Peak current going up is ~14 A at 800 counts/s (it was ~9 A in the first patch at lower speed). The test firmware's guard is at 20 A.
- Chassis tilt at rest against right-leg count (BNO055 gravity vector, board axes): tilt_y +3.75 deg at 10,534, 0.0 at 12,368, -2.1 at 13,571, -4.1 at 14,406, -6.1 at 15,397, -8.1 at 16,419, -9.9 at 17,417: close to linear at about -2.0 deg per 1,000 counts. tilt_x moves the same way by about 0.7 deg per 1,000 counts (-3.05 to +1.84 over the range).
- Leg returned to 13,551 (it started the day at ~13,560).

## Evening session: single-target mode, lag, handshake, watchdog (runs 06 end and 07)
- **Single target + controller speed cap (0x6081 = 800), tuned gains, window 10**: -1,000 counts ran at ~2,060 counts/s (cap not held), 21.8% PWM, 9 deg/s chassis rate; it did land 2 counts from target. So a step in the speed command is violent even with everything else fixed. Streaming a ramp stays the method. (The earlier single-target tests were confounded by the 100-count window and a 3.4x hotter velocity gain; this is the fair repeat.)
- **Only gain ratios matter**: PWM = (velocity P / QPPS) x (position P x error - speed) + integral. Tuned vs stock: velocity P / QPPS is 1.4e-4 vs 4.8e-3 (stock 34x higher); position P is 4.0 vs 0.10 (stock 40x lower). "QPPS is 50x too low" was one way of saying the first.
- **Lag fixed by leading the target**: writing the target ahead of the profile by speed x 250 ms (= 1 / position P) holds the leg within ~10 counts of the profile at 800 counts/s (it was 150 to 200 behind). End error unchanged (within ~13 counts); settles ~0.3 s sooner.
- **Lean handshake**: target + controlword 0x003F on one 10 ms tick, 0x002F on the next (each bit level held a full tick), no statusword polling. 50 Hz target updates, each SDO under 2 ms, 0 failures in ~380 writes per move. Writing 0x3F and 0x2F back to back worked for hundreds of updates and then for none of one move's 180: do not pulse it that short.
- **Heartbeat watchdog works**: consumer 0x1016:1 = 0x007F01F4 (node 127, 500 ms). Stopping the MIB heartbeat mid-move faulted the drive after 456 ms and 451 ms (two trials); the leg stopped where it was. The drive then sat in FAULT REACTION ACTIVE holding ~2 to 6% PWM / 1.5 to 3 A for ~11.5 s before going to FAULT with PWM 0; a disable-voltage write during that time is ignored. `clearfaults` then a normal move recovers it.
- **CAN unsent frames are a configuration matter**: `tx_retry_count = -1` gave 0 unsent frames in the first 31 s (81 lost arbitrations) and 2 in the whole 120 s run (cause of those 2 not captured); 3 and 15 gave ~1.6/s. espp's default is 0. Root cause: on the P4's SJA1000-style controller ESP-IDF's `twai_hal_v1.c` sets single shot for every value but -1 (espressif/esp-idf#19195, open). Draft PR: esp-cpp/espp#911.
- Heartbeat and consumer are left running after the first arm, because writing 0 to 0x1016:1 does not stop the controller watching.

## Acceptance of the ported Actuator (2026-11-17, ~20:00)
`components/Actuator/example` flashed to the chair's MIB, right main leg, settings as in `MIBconfig.hpp`. Boot wrote the window, both PID records, the clamp and the heartbeat consumer and read every one back equal; nothing was sent to node 2 or to node 3 M2.

| Move | Tolerance | Start -> target | Ended (from target) | Time | Peak speed (median-based) |
|---|---|---|---|---|---|
| +200 | 40 | 13,600 -> 13,793 | -24 | 1.08 s | 586 |
| +1,000 | 40 | 13,780 -> 14,783 | -19 | 2.32 s | 943 |
| -1,000 | 40 | 14,788 -> 13,788 | +31 | 2.35 s | 938 |
| +1,000 | 15 | 14,786 -> 15,790 | -12 | 2.43 s | 900 |
| -1,000 | 15 | 15,804 -> 14,799 | -9 | 2.74 s | 852 |
| +1,000 | 15 | 14,790 -> 15,797 | -5 | 2.87 s | 895 |
| -1,000 | 15 | 15,801 -> 14,797 | +6 | 2.84 s | 971 |

- At tolerance 40 the drive was released while the leg was still converging; 15 fixed it (applied to a median of 9 reads).
- The leg briefly runs up to ~970 counts/s on an 800 counts/s profile: the 250 ms target lead plus the velocity loop overshooting at the end of the acceleration ramp. Bench moves showed the same (cruise 790 to 930).
- `stop` 1.6 s into a 3,000-count move: disable voltage acknowledged, PWM 0.0% 110 ms later, the leg coasted ~70 counts.
- `abs` / `rel` / `home` on ML (left leg) and LCarr (carriage) refused with "not commissioned"; nothing sent.
