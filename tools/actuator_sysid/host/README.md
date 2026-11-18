# Actuator system identification: host tools

> **Status:** written before the 2026-11-17 chair session and tested on simulated logs only. The session itself used the smaller scripts in `../bench`. The parser reads the real logs; the closed-loop analyses (`pstep`, `plant`) have not been checked against them.


> **Safety. Only four axes may be moved: node 1 M1 (front casters), node 1 M2 (rear
> casters), node 3 M1 (main left leg), node 3 M2 (main right leg). Node 2 drives the
> two carriage sliders and is never commanded.** `capture.py` refuses `arm`, `pstep`
> and `step` naming node 2 (the firmware blocks them too). The actuators are
> electro-hydraulic and not back-drivable: a stopped joint stays where it is, so a
> wrong sign does not fall back on its own. Keep a hand near the keyboard: **any key
> while a move is running sends `stop`**, and so do Ctrl-C and quitting.

Capture and analysis for the sysid logger in `../firmware/`. The logger streams the
BNO055 and every MCP266 channel as text over the MIB's USB-C console. These tools
record it, check it, and turn each experiment into a plot and a JSON of numbers in a
few seconds. The IMU is the independent witness: it shows what the chassis really did,
whatever the motor controller reports.

## Setup

```
cd tools/actuator_sysid/host
uv sync                                   # numpy, scipy, matplotlib, pyserial (+ pytest)
echo '{"n1c1":"FC","n1c2":"RC","n2c1":"LCarr","n2c2":"RCarr","n3c1":"ML","n3c2":"MR"}' > runs/axis_names.json
```

`runs/axis_names.json` names the channels in every plot under `runs/`. The node and
channel map above is the lead's; check it once in step 1.

The console is a WCH CH343 USB-UART bridge (`/dev/cu.usbmodem5C372451831`, VID 0x1A86
PID 0x55D3) at 921600 baud. Its DTR/RTS lines drive the ESP32-P4's reset and boot
pins. `capture.py` clears both before opening, but macOS raises them for an instant
when the port opens, so **opening the port may reset the board once**. The ROM's boot
lines then arrive at another baud as garbage; they are kept in `raw.log`, counted, and
ignored. So start `capture.py` once and leave it running for the whole session. Start
each experiment's file with `/run <label>` rather than restarting capture.

## The three tools

| Command | What it does |
|---|---|
| `uv run capture.py --label <name>` | Records to `runs/<timestamp>_<label>/raw.log` with a live status line. Type firmware commands or notes. |
| `uv run analyze.py <analysis> [run]` | Writes `<analysis>.png` and `<analysis>.json` into the run folder. `run` defaults to the latest; a name fragment works. `--open` shows the PNG. |
| `uv run simulate.py <scenario>` | Writes a simulated run in the exact wire format, with `sim_truth.json`. Use it to rehearse. |

The capture status line shows rows/s, dropped and garbled rows, stray lines, CAN
staleness per node, pitch and roll, every encoder, and `MOVING ...` while a target is
live. Typing:

```
hdr | rate <hz> | nodes | stop | mark <text> | dump <node> | clearfaults <node>
arm <node> <ch>              then within the arm timeout:
pstep <node> <ch> <delta>    a closed-loop step of <delta> counts by the controller's own PID
/run <label>                 new run folder; the port stays open
/send <text>                 a raw line (the node 2 guard still applies)
/quit                        or Ctrl-D: sends stop and exits
anything else                an operator note, stored with a host timestamp
```

`raw.log` holds the device bytes exactly as they arrived. Host lines
(`#host,<unix_s>,<t_us_est>,<note|cmd|sys>,<text>`) are inserted only between whole
lines. `meta.json` records the port, start and end time, `git describe`, label,
notes, every command sent or refused, and the controller config from `dump`.

Unplugging the cable does not lose the file: capture says `PORT LOST`, retries every
0.5 s and carries on in the same file after replug. The board probably resets, and
the analysis treats the clock going back to zero as a new segment.

## Runbook

Do the steps in this order. Each one checks something the next one relies on.

### 1. Passive sanity: signs, units, staleness (2 min, nothing moves)

```
uv run capture.py --label session          # leave it running all session
/run passive
mark chair level, nobody on it
(after 30 s) mark operator presses the front of the seat down
(after 30 s) mark operator lifts the left armrest
/run level                                 # next step's file; analyse this one meanwhile
uv run analyze.py overview passive --open
```

What good looks like:
- the flags list is empty or says only `note:`
- `|g|` is 9.81 m/s², `quat ... (x deg)` agrees within 3°, and BNO055 calibration has gyro and accel at 3
- `age p95` is under about 2 CAN poll periods for every node
- dropped rows are under 1 %
- pressing the front down gives **pitch +** (REP-103: nose down), and lifting the left armrest gives **roll +**

If a number looks wrong:
- If `|g|` is about 1.0 or 980, the firmware is sending g or cm/s², not m/s².
- If pitch moves the wrong way, or pitch and roll are swapped, the board is turned on the chassis. Give `--mount roll,pitch,yaw` (for example `--mount 0,0,90`) to every analyze command, or bake it into step 2.
- If one node's age is large or its fields are empty, that controller is offline. Check `nodes`, the CAN cable and termination.
- If dropped rows exceed 1 %, lower `rate` (try 50 Hz) or close other USB traffic.

### 2. Level reference (1 min)

Park on the flattest floor available, all actuators where they sit for driving.

```
mark level floor
/run rest
uv run analyze.py level-ref level [--mount 0,0,90]
```

This writes `runs/level_ref.json`, which every later analysis of a run under `runs/`
uses. A good result is a mounting angle under a few degrees. If it reads 10° or more,
the floor isn't level or the board is mounted at an angle. Measure the floor with a
spirit level before trusting it.

### 3. Hold test with the drive released (5 min or more, nothing moves)

```
mark drive released, nobody on the chair
(wait 5 minutes)
/run dump
uv run analyze.py rest rest
```

The analysis reports encoder creep per channel (counts/min ± standard error), chassis
pitch and roll drift (mdeg/min), and the noise floor of every signal. Use the encoder
noise and gyro noise numbers as the floor for everything later.

A good result is creep within ±0.1 counts/min (within its error) on all four EHAs.
`note: ... creeps` means the EHA leaks down under load. Check that the IMU drift agrees
in sign through the kinematics map from step 8.

### 4. Controller configuration (instant)

```
dump 1
dump 3
/run fc-small
```

The `#cfg` lines go into `meta.json` under `controller_config`. They record the PID
gains, limits and duty cap in force for the steps that follow. Dump again whenever you
change a setting in Motion Studio.

### 5. Plant identification: small closed-loop steps, one axis (about 3 min per axis)

The firmware has no open-loop duty. Excitation is the controller's own position loop,
and the identification uses the applied PWM it reports (`_duty`) as the input. Step
sizes go from "will not move" to "clearly moves", both ways, back to the start:

```
arm 1 1
pstep 1 1 30        (wait until MOVING clears, about 4 s)
arm 1 1
pstep 1 1 -30
... repeat with 50, 65, 80, 120, 200 and their negatives
/run rc-small       and the same for 1 2, then 3 1, 3 2
uv run analyze.py plant fc-small --open
uv run analyze.py pstep fc-small
```

`plant` fits, per direction:
- gain (counts/s per % PWM)
- deadband (the PWM below which a moving joint stops)
- breakaway (the PWM needed to start from rest)
- dead time and pressure build-up lag

It also reports three versions of the stall threshold, which should agree within a
percent or so:
- the fitted deadband
- the PWM held at rest after a step stopped short
- the loop gain times the stall error

What good looks like:
- the fit residual is about 1 count RMS
- `gain +` differs from `gain -`, with the larger one in the direction the load helps
- deadband below breakaway
- `kp x stall err` within 0.3 % of `stall PWM` for a P-only loop

If a number looks wrong:
- A fit residual of several counts means the plant isn't this model. Look at the "Model against encoder" panel to see where it misses: often a limit switch, a step cut short, or a second axis moving.
- If no step moved, raise the step sizes. If every step moved, the breakaway bracket is open at the bottom: add smaller steps.
- If the stall PWM sits well above the deadband, it is stick-slip or the steps were dropped before the joint stopped. Hold longer.
- If the loop reads `PI` when you expect P, check the `dump` output for an integral gain.

### 6. Step response (same runs, or larger steps)

`analyze.py pstep` gives, per step:
- rise time (10 to 90 % of the distance actually travelled)
- overshoot and settling time
- steady-state error in counts
- PWM and current at rest
- whether it stalled, never moved, or was still moving when the target was dropped

A good result has a small steady-state error and no overshoot. A P-only loop against
an EHA deadband stops short by about deadband ÷ kp counts. That is the "stalls short
of target" behaviour, and the `Where it stops` panel shows it as points on the kp × error line.

If a step is `STILL MOVING`, hold the target longer. If a step shows `NO MOTION` at a
PWM above the breakaway from step 5, the controller limited the duty: look for
`duty cap seen`.

### 7. Latency (same runs)

```
uv run analyze.py latency fc-small
```

It splits the delay from command to chassis into:
- CAN/command: from the `_tgt` edge to the first PWM readback, bracketed across steps
- mechanical dead time: from PWM to motion, from the plant fit
- pressure lag
- encoder to IMU (least-squares cross-correlation of gyro against encoder speed)

It also reports `_age_us` staleness per channel.

A good result is CAN latency of a few ms and staleness p95 under 2 poll periods.

The encoder-to-IMU lag needs the chassis to turn at about 1 °/s or more. At the
speeds of small steps the gyro noise wins, and the tool says
`encoder->IMU lag uncertain (+-N ms)` rather than a number. Use the step 8 or step 9
runs, or larger steps, for that one.

### 8. Kinematics: the map a levelling controller needs (about 3 min per axis)

Move one axis at a time with rests between moves: three steps up, six down, three up.

```
/run kinematics
arm 1 1
pstep 1 1 150      (x3, then -150 x6, then +150 x3, each after MOVING clears)
... then 1 2, 3 1, 3 2
uv run analyze.py kinematics kinematics --open
```

The JSON has `jacobian_matrix`: degrees of pitch and roll per encoder count for each
axis. It also gives linearity residuals, the curvature a quadratic term would add over
the travelled range, and backlash in counts with a standard error.

What good looks like:
- residuals under 0.1° RMS
- front and rear casters mostly pitch, with opposite signs
- the main legs mostly roll, with opposite signs
- backlash a few counts, standard error under 2

If the residual is large, something else moved (the rider, a caster swivel) or the
joint isn't linear over that range. Check `nonlinearity_deg`. If backlash comes out
negative or with a large standard error, there weren't enough rests after reversals:
add more up-down cycles.

### 9. Profile comparison: rider comfort

Repeat the same moves under each controller setting you want to compare, one run each
(`dump` after every change), then:

```
uv run analyze.py smoothness profile-a
uv run analyze.py smoothness profile-b
uv run analyze.py compare smoothness profile-a profile-b --open
uv run analyze.py compare smoothness profile-a profile-b --metric summary.peak_rate_dps --metric summary.rms_ang_acc_corrected
```

`smoothness` reports RMS and peak jerk and angular acceleration at the seat per move,
low-passed at 5 Hz (`--fc`), with the at-rest noise floor removed in quadrature
(`*_corrected`).

These EHAs move slowly. Seat jerk is close to the BNO055's noise floor, and the tool
flags moves within 1.5× of it. When it does, compare `peak_rate_dps` and
`rms_ang_acc_corrected`, which stay above the noise. Repeating each profile 5 to 10
times also helps.

## Wire format notes

- Columns are parsed by name in every header. A header may change mid-run.
- An empty field is NaN.
- A row with the wrong number of fields or a non-number is counted as garbled and skipped.
- `#log,` lines (ESP-IDF logs) and anything unrecognised are counted.
- `#cfg,<node>,<name>,<index>,<sub>,<value>` is stored.
- A 32-bit `t_us` wrap is unwrapped. Any other backwards jump of more than 50 ms is a reset and starts a new segment, logged as an event.
- The time of each CAN value is `t_us - <ch>_age_us`, so encoder, PWM, speed and current use their own read times, not the row time.
- Command edges (`_cmd`, `_tgt`) are refined to the exact time of a firmware event between the two rows, when there is one.

### Checked against the real logs (`docs/actuator-revamp/runs/*.log`, 17 Nov)

All three parse with no garbled rows:
- `#cfg` dumps are stored (528 and 220 entries).
- `#meta` keys including `can_errors`, `can_bus_errors` and `n<id>_sdo`/`_poll` are kept in the run's `meta`.
- ROM garbage after the reset is counted as stray, and `_tgt` columns are read.

The firmware logs `_duty` raw (−32767..32767), and the parser converts it to percent
whenever `#meta:unit_duty` says so. What to know before trusting numbers from these logs:
- **The motion session holds no `pstep`.** It was `arm 3 1`, then `clamp`, then a disarm after 10 s, and `_duty` stayed 0. So `pstep` and `plant` correctly report nothing.
- **`n3c2` moved 4047 counts with zero PWM.** Something outside the MIB moved it. Confirm before reading anything into it.
- **`_spd` contains ±2147483647 sentinels**, and n3c1 `_spd` sits on ±3000. Treat the speed readback as unverified, as `unit_spd` already says. The analyses rely on the encoder; `overview`'s speed-sign warning on these logs comes from those sentinels.
- **Row jitter p99 is about 3.3 ms** (the time-base check wants under 2.5 ms at 100 Hz), so `overview` flags it. One row in about 8000 was dropped.
- **The chair runs on about 50 V.** `overview`'s battery check assumes a 24 V bench (21 to 30 V), so ignore that warning.
- **BNO055 calibration reads sys 0, accel 0.** Recalibrate (or check the fusion mode) before the kinematics run.

capture.py also sends `scan`, `clamp` and `disarm`, and refuses `clamp` naming node 2.

## Simulator and tests

`simulate.py` writes any scenario with the plant it was made from (`sim_truth.json`):
`passive`, `rest`, `pstep`, `kinematics`, `profile_fast`, `profile_slow`, and the
legacy open-loop `steps` and `deadband`. Options: `--clean` (no noise),
`--external` (open loop driven from outside the MIB), `--seed`.

The plant is electro-hydraulic:
- pressure builds with a lag after a mechanical dead time
- motion starts above a breakaway PWM and stops below a smaller deadband
- gain and current depend on direction (gravity)
- slow creep at rest, and backlash between encoder and chassis
- a P or PI position loop in the simulated controller

The chassis is a rigid plane on four supports. The IMU sits on the seat at a mounting
rotation with fusion delay and noise. The stream has CAN staleness, empty fields,
dropped rows, a USB stall, garbage bytes, truncated rows, stray and `#log` lines, a
reordered header, and `#cfg` lines.

```
uv run pytest            # about 2 minutes
```

Each test checks an analysis against the simulator's truth within the tolerance
written in the test. `capture.py` is tested on a replay and on a pseudo-terminal with
garbage, an unplug and a replug. The pty test uses `--baud 115200` because macOS sets
921600 with an ioctl a pty rejects.

The open-loop `step` and `deadband` analyses remain for data driven by a duty from
outside the MIB (Motion Studio). On closed-loop runs they decline and point at `plant`
and `pstep`.

## Why not the existing code

The MIB panel (`tools/mib_panel/`) already reads the MIB over the same USB-C console.
It speaks the binary stream_frame telemetry of the main firmware at 50 Hz, decimated
and schema-driven, through a browser. The sysid firmware emits a different, text
protocol (row, `#evt`, `#cfg`) built for complete, timestamped capture to disk. The
panel's link layer wasn't reusable here, but one piece of it was: holding DTR/RTS low
on the CH343 (`mib_link.js`), which `capture.py` does the same way.
