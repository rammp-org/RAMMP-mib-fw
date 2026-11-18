# Actuator component

`mib::Actuator` drives one leg's linear actuator (an EHA) on one channel of a Basicmicro
MCP266 over CANopen. It speaks encoder counts only; units and kinematics belong above the
BSP, which owns one Actuator per leg in `MIBconfig.hpp`.

It drives the controller the way the chair measurements of 2026-11-17 showed works
([`docs/actuator-revamp/bench-findings.md`](../../docs/actuator-revamp/bench-findings.md));
[DESIGN.md](DESIGN.md) explains the controller as measured and the component's design:

- `initialize()` writes the leg's controller settings to controller RAM (position window
  0x6067, position and velocity PID records, clamp, 0x607D, profile velocity, heartbeat
  consumer) and reads every one back. A mismatch leaves the leg not ready.
- A move is a trapezoid generated on the MIB (`MotionProfile`) and streamed from `poll()`
  every 10 ms: the target, led ahead of the profile by 1 / position P and never past the
  final target, with the new-set-point handshake held a tick at each level. A new target
  mid-move continues from the profile's position and speed.
- Arrival is the median encoder count settled within `tolerance` of the target after the
  profile has ended; then the drive is released (disable voltage). A leg that has not
  arrived by the profile time plus `arrival_margin` is stopped, released and reported
  FAILED with how far short.
- Faults are never cleared as a side effect; `clear_faults()` is explicit. `stop()` is
  disable voltage, retried.
- A leg with `commissioned = false` is never written to and refuses every move.

## API

| Method | Purpose |
|---|---|
| `initialize()` | Write and read back the controller settings. Nothing for a leg not commissioned |
| `ready()`, `commissioned()`, `online()` | Commissioned, settings verified, homed if needed |
| `move_absolute(counts[, profile])` | Start a move (clamped to the range); returns once accepted |
| `move_relative(delta[, profile])` | From the median count now, or from the target mid-move |
| `increment()`, `decrement()` | One jog step |
| `stop()` | Disable voltage now, retried |
| `poll()` | The move engine: call every `Actuator::kTickPeriod` (10 ms) from one task |
| `moving()`, `move_result()` | Whether a move is in progress; how the last one ended |
| `read_status(status)` | Count, PWM, current, statusword, drive state, fault |
| `clear_faults()` | CiA 402 fault reset of this channel |
| `read_controller_settings(readback)` | Every setting `initialize()` writes, read from the controller |
| `read_position(counts)`, `read_position_median(median, spread)` | One read; a median of 9 |
| `set_calibration_mode`, `set_range`, `home`, `restore_position`, `save_position`, `set_encoder` | Calibration (absolute encoders) and homing (incremental) |

Methods return true on success and log the reason on failure. Every method that talks to
the controller blocks for one or more SDOs (under 2 ms each, up to the 20 ms SDO timeout)
and must be called from a task that may wait, never from an RTPS callback.

The owner runs the ticks: `mib::bsp::MIB::poll_actuators()` every 10 ms on a fixed-period
schedule (deadline += period), as MibSystem runs its IMU task. Nothing is sent when no
move is in progress.

## Example: the bench console

[`example/`](example) brings the legs up through the BSP exactly as the firmware does (CAN
with `tx_retry_count = -1`, the MIB heartbeat, every commissioned leg's settings written and
read back) and opens a console on the selected leg, the right main leg (MR) by default.
Boot prints the configured and read-back settings. Commands: `legs`, `leg <name>`,
`settings`, `status`, `result`, `abs <counts> [speed]`, `rel <delta> [speed]`, `inc`, `dec`,
`stop`, `clear`, `wait`, `init`, `scan`, `sdor`, `sdow`, `flags`, and the calibration and
homing commands. Moves return at once, so `stop` works mid-move; the Actuator logs how each
move ended and the PWM after the release.

Build and run it from that directory (ESP-IDF v6.0.1):

```
idf.py set-target esp32p4
idf.py build flash
idf.py -b 921600 monitor        # the CH343 UART on the USB-C; opening it resets the board
```

### Acceptance test on the chair (right main leg)

```
settings                 # every line "configured / read back" equal, "all match"
status                   # PWM 0.0 %, no FAULT, "ready"
rel 1000                 # up 1,000 counts at 800 counts/s; the log reports
                         #   "arrived at ... (target ..., +N counts) ... peak ~800 counts/s,
                         #    released 0.x s after the profile ended"
                         #   then "released: PWM 0.0 %, ... mA"
result                   # the same, as a summary
rel -1000                # and back down
rel 3000                 # then, while it moves:
stop                     # halts it at once; prints PWM 0.0 % after 150 ms
result                   # "stopped", where it stopped
abs 13560                # back to where it started
leg ML
abs 30000                # REFUSED: ML is not commissioned
leg LCarr
rel 100                  # REFUSED: LCarr is not commissioned
leg MR
status                   # after several minutes idle: still no FAULT (the heartbeat)
```

Before `rel 3000`, check `status` that the count is at least 3,000 below 18,000 (the clamp
would cut the move short otherwise).

## Tests

`test/host/run.sh` builds and runs the motion-profile checks on the host (no ESP-IDF
needed): the trapezoid, the lead, never past the target, and continuing a move in flight.
CI runs it after the firmware build.

## Dependency

`espp/mcp266` and `espp/motor_controller` come from the branch of esp-cpp/espp#911 (the
clamp read from subindex 1), on top of #910; return them to a released version once #911
is released. See the comment in `idf_component.yml`.
